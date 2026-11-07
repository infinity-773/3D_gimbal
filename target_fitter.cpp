#include "target_fitter.h"

#include <opencv2/core.hpp>

#include <cmath>
#include <numbers>
#include <vector>

std::optional<PhaseFit> FitPhase(const std::vector<double>& phi_obs,
                                 const std::vector<double>& times, double omega_hint) {
    const std::size_t count = phi_obs.size();
    if (count < 3 || times.size() != count || !std::isfinite(omega_hint)) return std::nullopt;

    std::vector<double> phases(count);
    double time_mean = 0.0, phase_mean = 0.0;
    
    for (std::size_t index = 0; index < count; ++index) {
        if (!std::isfinite(phi_obs[index]) || !std::isfinite(times[index]) ||
            (index > 0 && times[index] < times[index - 1])) return std::nullopt;
        phases[index] = index == 0 ? phi_obs[index] :
            phases[index - 1] + TargetModel::WrapPi(phi_obs[index] - phases[index - 1]);
        time_mean += times[index] / static_cast<double>(count);
        phase_mean += phases[index] / static_cast<double>(count);
    }
    double numerator = 0.0, denominator = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const double delta_t = times[index] - time_mean;
        numerator += delta_t * (phases[index] - phase_mean);
        denominator += delta_t * delta_t;
    }
    const double velocity = denominator > 1e-12 ? numerator / denominator : omega_hint;
    PhaseFit result;
    result.t_ref = times.front();
    result.phi_ref = phase_mean + velocity * (result.t_ref - time_mean);
    result.omega = std::abs(velocity);
    result.direction = velocity < 0.0 ? -1 : 1;
    double squared_error = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const double error = TargetModel::WrapPi(phases[index] - result.phi_ref - velocity * (times[index] - result.t_ref));
        squared_error += error * error;
    }
    result.rms_deg = std::sqrt(squared_error / static_cast<double>(count)) * 180.0 / std::numbers::pi;
    return result;
}

GeomRefine RefineGeometryStep(const TargetGeometry& cur,
                              const std::vector<frc::Translation3d>& pts_base,
                              double k_center, double k_radius,
                              double max_center_shift, double max_radius_shift) {
    GeomRefine result{cur.center, cur.radius, false};
    if (!cur.valid || pts_base.empty() || !std::isfinite(k_center) || !std::isfinite(k_radius) ||
        !std::isfinite(max_center_shift) || !std::isfinite(max_radius_shift) ||
        max_center_shift < 0.0 || max_radius_shift < 0.0) return result;
    frc::Translation3d point;
    for (const auto& item : pts_base) {
        if (!item.IsFinite()) return result;
        point = point + item;
    }
    point = point / static_cast<double>(pts_base.size());
    const auto delta = point - cur.center;
    const double norm = delta.Norm().value();
    if (norm < 1e-12) return result;
    const auto radial = delta / norm;
    const auto error = point - (cur.center + radial * cur.radius);
    const auto tangential = error - cur.normal * error.Dot(cur.normal).value();
    auto shift = tangential * k_center;
    const double shift_norm = shift.Norm().value();
    if (shift_norm > max_center_shift) shift = shift * (max_center_shift / shift_norm);
    result.center = cur.center + shift;
    result.radius = cur.radius + std::clamp(k_radius * error.Dot(radial).value(), -max_radius_shift, max_radius_shift);
    result.valid = result.radius > 0.0;
    return result;
}

static std::optional<FitResult> FitCircle(const std::vector<frc::Translation3d>& pts_base,
                                         const std::vector<double>& times,
                                         double omega_hint, const TargetGeometry* prior) {
    const std::size_t count = pts_base.size();
    if (count < 8 || times.size() != count || !std::isfinite(omega_hint)) return std::nullopt;
    frc::Translation3d mean;
    double time_mean = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        if (!pts_base[index].IsFinite() || !std::isfinite(times[index]) ||
            (index > 0 && times[index] < times[index - 1])) return std::nullopt;
        mean = mean + pts_base[index];
        time_mean += times[index] / static_cast<double>(count);
    }
    mean = mean / static_cast<double>(count);
    cv::Mat cov = cv::Mat::zeros(3, 3, CV_64F);
    for (const auto& point : pts_base) {
        const auto delta = point - mean;
        const double values[] = {delta.X().value(), delta.Y().value(), delta.Z().value()};
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                cov.at<double>(row, col) += values[row] * values[col] / static_cast<double>(count);
    }
    FitResult result;
    cv::SVD plane(cov, cv::SVD::FULL_UV);
    if (plane.w.at<double>(1) < 1e-12) return std::nullopt;
    if (prior) {
        if (!prior->valid || prior->normal.Norm().value() < 1e-12) return std::nullopt;
        result.geom.normal = prior->normal;
        result.geom.source = GeomSource::REFINED;
    } else {
        result.geom.normal = frc::Translation3d{plane.vt.at<double>(2, 0),
                                                plane.vt.at<double>(2, 1),
                                                plane.vt.at<double>(2, 2)};
    }
    result.geom.BuildBasis();
    cv::Mat matrix(static_cast<int>(count), 3, CV_64F);
    cv::Mat rhs(static_cast<int>(count), 1, CV_64F);
    std::vector<double> xs(count), ys(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto delta = pts_base[index] - mean;
        xs[index] = delta.Dot(result.geom.u).value();
        ys[index] = delta.Dot(result.geom.v).value();
        matrix.at<double>(static_cast<int>(index), 0) = xs[index];
        matrix.at<double>(static_cast<int>(index), 1) = ys[index];
        matrix.at<double>(static_cast<int>(index), 2) = 1.0;
        rhs.at<double>(static_cast<int>(index)) = -xs[index] * xs[index] - ys[index] * ys[index];
    }
    cv::Mat coefficients;
    if (!cv::solve(matrix, rhs, coefficients, cv::DECOMP_SVD)) return std::nullopt;
    const double cx = -coefficients.at<double>(0) / 2.0;
    const double cy = -coefficients.at<double>(1) / 2.0;
    const double radius_squared = cx * cx + cy * cy - coefficients.at<double>(2);
    if (!std::isfinite(radius_squared) || radius_squared <= 0.0) return std::nullopt;
    result.geom.radius = std::sqrt(radius_squared);
    result.geom.center = mean + result.geom.u * cx + result.geom.v * cy;
    result.geom.valid = true;
    std::vector<double> phases(count);
    double phase_mean = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const double phi = std::atan2(ys[index] - cy, xs[index] - cx);
        phases[index] = index == 0 ? phi : phases[index - 1] + TargetModel::WrapPi(phi - phases[index - 1]);
        phase_mean += phases[index] / static_cast<double>(count);
    }
    double numerator = 0.0, denominator = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const double delta_t = times[index] - time_mean;
        numerator += delta_t * (phases[index] - phase_mean);
        denominator += delta_t * delta_t;
    }
    const double omega_fit = denominator > 1e-12 ? numerator / denominator : omega_hint;
    result.phase.t_ref = times.front();
    result.phase.phi_ref = phase_mean + omega_fit * (result.phase.t_ref - time_mean);
    result.phase.omega = std::abs(omega_fit);
    result.phase.direction = omega_fit < 0.0 ? -1 : 1;
    double squared_error = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const double error = TargetModel::WrapPi(phases[index] - result.phase.PhaseAt(times[index]));
        squared_error += error * error;
    }
    result.rms_deg = std::sqrt(squared_error / static_cast<double>(count)) * 180.0 / std::numbers::pi;
    result.phi_target_offset = TargetModel::WrapPi(phases.front() - result.phase.PhaseAt(times.front()));
    return result;
}

std::optional<FitResult> FitAll(const std::vector<frc::Translation3d>& pts_base,
                                const std::vector<double>& times, double omega_hint) {
    return FitCircle(pts_base, times, omega_hint, nullptr);
}

std::optional<FitResult> FitWithPlane(const std::vector<frc::Translation3d>& pts_base,
                                      const std::vector<double>& times, double omega_hint,
                                      const TargetGeometry& prior) {
    return FitCircle(pts_base, times, omega_hint, &prior);
}