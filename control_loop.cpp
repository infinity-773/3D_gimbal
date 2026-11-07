#include "control_loop.h"
#include "target_fitter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace {
ServoCallback servo_callback;
constexpr std::size_t kFitMinFrames = 12;
constexpr double kFitMinDuration = 0.5;
constexpr double kFitMinTravel = 10.0 * std::numbers::pi / 180.0;
constexpr double kFitWindow = 10.0;
constexpr double kGeomUpdateInterval = 0.5;
constexpr double kGeomMaxCenterStep = 0.02;
constexpr double kGeomMaxRadiusStep = 0.02;

bool IsFitUsable(const FitResult& fit, const std::vector<frc::Translation3d>& points,
                 const std::vector<double>& times) {
    if (!fit.geom.valid || !std::isfinite(fit.geom.radius) || fit.geom.radius <= 0.0 ||
        fit.rms_deg >= 1.5 || fit.phase.omega <= 0.05 || fit.phase.omega >= 10.0 ||
        fit.phase.omega * (times.back() - times.front()) < kFitMinTravel) return false;
    double radial_error_squared = 0.0;
    double plane_error_squared = 0.0;
    for (const auto& point : points) {
        const auto delta = point - fit.geom.center;
        const double x = delta.Dot(fit.geom.u).value();
        const double y = delta.Dot(fit.geom.v).value();
        const double radial_error = std::hypot(x, y) - fit.geom.radius;
        const double plane_error = delta.Dot(fit.geom.normal).value();
        radial_error_squared += radial_error * radial_error;
        plane_error_squared += plane_error * plane_error;
    }
    return std::sqrt(radial_error_squared / points.size()) <= 0.02 &&
           std::sqrt(plane_error_squared / points.size()) <= 0.05;
}
}

FireController::FireController(double shot_interval, double pulse_duration)
    : shot_interval_{std::max(shot_interval, 0.0)},
      pulse_duration_{std::clamp(pulse_duration, 0.0, shot_interval_)},
      last_shot_{-std::numeric_limits<double>::infinity()},
      pulse_until_{-std::numeric_limits<double>::infinity()} {}

void FireController::SetEnabled(bool enabled) { enabled_ = enabled; }
bool FireController::IsEnabled() const { return enabled_; }

int FireController::Update(double t, const TargetModel& model, bool servo_ready) {
    if (!enabled_ || !servo_ready || model.Lock() != LockState::LOCKED || !model.FireEnabled()) {
        pulse_until_ = -std::numeric_limits<double>::infinity();
        return 1;
    }
    if (t >= last_shot_ + shot_interval_) {
        last_shot_ = t;
        pulse_until_ = t + pulse_duration_;
    }
    return t < pulse_until_ ? 0 : 1;
}

void SetServoCallback(ServoCallback callback) { servo_callback = std::move(callback); }
void SendToServo(double yaw, double pitch) {
    if (servo_callback) servo_callback(yaw, pitch);
}

void ControlTick(double t_now, double dt_ctrl, TargetModel& model, const frc::Transform3d&) {
    model.Tick(t_now, dt_ctrl);
    if (model.Lock() != LockState::IDLE) {
        const auto aim = model.Predict(t_now);
        const double x = aim.X().value(), y = aim.Y().value(), z = aim.Z().value();
        SendToServo(std::atan2(y, x), std::atan2(z, std::hypot(x, y)));
    }
}

void OnVisionFrame(const VisionFrame& vf, const CameraConfig& cam,
                   const frc::Transform3d& baseToCam, TargetModel& model,
                   std::vector<frc::Translation3d>& pts_base_accum,
                   std::vector<double>& times_accum, double& geom_residual_accum_time) {
    if (!vf.detected) { model.OnObservation({vf.t, 0.0, false}); return; }
    const auto points_cam = PnPBlade(vf.pts_px, cam, baseToCam);
    if (!points_cam) { model.OnObservation({vf.t, 0.0, false}); return; }
    const auto points = ToBase(*points_cam, baseToCam);
    const double previous_time = model.state.last_obs_time;
    const auto obs = MakeObservation(vf, points, model.geom);
    model.OnObservation(obs);
    frc::Translation3d representative;
    for (const auto& point : points) representative = representative + point;
    representative = representative / 4.0;
    if (model.Lock() == LockState::IDLE) {
        if (pts_base_accum.size() != times_accum.size() ||
            (!times_accum.empty() && vf.t <= times_accum.back())) {
            pts_base_accum.clear();
            times_accum.clear();
        }
        pts_base_accum.push_back(representative);
        times_accum.push_back(vf.t);
        while (times_accum.size() > kFitMinFrames && vf.t - times_accum.front() > kFitWindow) {
            pts_base_accum.erase(pts_base_accum.begin());
            times_accum.erase(times_accum.begin());
        }
        if (pts_base_accum.size() >= kFitMinFrames &&
            times_accum.back() - times_accum.front() >= kFitMinDuration) {
            const auto fit = model.geom.valid
                ? FitWithPlane(pts_base_accum, times_accum, model.phase.OmegaAt(vf.t), model.geom)
                : FitAll(pts_base_accum, times_accum, model.phase.OmegaAt(vf.t));
            if (fit && IsFitUsable(*fit, pts_base_accum, times_accum)) {
                model.ApplyFit(fit->geom, fit->phase, fit->phi_target_offset);
            }
        }
        if (model.Lock() != LockState::LOCKED) return;
        model.state.last_obs_time = vf.t;
        pts_base_accum.clear();
        times_accum.clear();
        geom_residual_accum_time = 0.0;
        return;
    }
    if (model.Lock() != LockState::LOCKED || model.state.switch_count != 0) {
        pts_base_accum.clear();
        times_accum.clear();
        geom_residual_accum_time = 0.0;
        return;
    }
    if (pts_base_accum.size() != times_accum.size() ||
        (!times_accum.empty() && vf.t <= times_accum.back())) {
        pts_base_accum.clear();
        times_accum.clear();
        geom_residual_accum_time = 0.0;
    }
    pts_base_accum.push_back(representative);
    times_accum.push_back(vf.t);
    while (times_accum.size() > kFitMinFrames && vf.t - times_accum.front() > kFitWindow) {
        pts_base_accum.erase(pts_base_accum.begin());
        times_accum.erase(times_accum.begin());
    }
    const double elapsed = vf.t - previous_time;
    if (elapsed > 0.0 && elapsed <= 0.1) geom_residual_accum_time += elapsed;
    if (geom_residual_accum_time >= kGeomUpdateInterval && pts_base_accum.size() >= kFitMinFrames &&
        times_accum.back() - times_accum.front() >= kFitMinDuration) {
        const auto fit = FitWithPlane(pts_base_accum, times_accum, model.phase.OmegaAt(vf.t), model.geom);
        if (fit && IsFitUsable(*fit, pts_base_accum, times_accum)) {
            auto shift = fit->geom.center - model.geom.center;
            const double shift_norm = shift.Norm().value();
            if (shift_norm > kGeomMaxCenterStep) shift = shift * (kGeomMaxCenterStep / shift_norm);
            const double radius_shift = std::clamp(fit->geom.radius - model.geom.radius,
                                                   -kGeomMaxRadiusStep, kGeomMaxRadiusStep);
            model.ApplyGeomRefine(model.geom.center + shift, model.geom.radius + radius_shift);
        }
        geom_residual_accum_time = 0.0;
    }
}