#include "vision_bridge.h"

#include <opencv2/calib3d.hpp>
#include <cmath>

std::optional<std::vector<frc::Translation3d>> PnPBlade(
    const std::vector<cv::Point2f>& pts_px, const CameraConfig& cam,
    const frc::Transform3d& baseToCam) {
    if (pts_px.size() != 4 || !std::isfinite(cam.blade_size) || cam.blade_size <= 0.0 ||
        cam.K(0, 0) <= 0.0 || cam.K(1, 1) <= 0.0) return std::nullopt;
    for (const auto& point : pts_px)
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
    const double radius = cam.blade_size / 2.0;
    const std::vector<cv::Point3d> object{{0, -radius, 0}, {radius, 0, 0},
                                          {0, radius, 0}, {-radius, 0, 0}};
    std::vector<cv::Point2f> undistorted;
    std::vector<cv::Mat> rotations, translations;
    try {
        cv::undistortPoints(pts_px, undistorted, cam.K, cam.D);
        if (cv::solvePnPGeneric(object, undistorted, cv::Matx33d::eye(), cv::noArray(),
                                rotations, translations, false, cv::SOLVEPNP_IPPE) <= 0)
            return std::nullopt;
    } catch (const cv::Exception&) { return std::nullopt; }
    double best_alignment = -1.0;
    std::vector<frc::Translation3d> best;
    for (std::size_t index = 0; index < rotations.size(); ++index) {
        cv::Mat rotation;
        cv::Rodrigues(rotations[index], rotation);
        const frc::Translation3d normal_cam{rotation.at<double>(0, 2), rotation.at<double>(1, 2), rotation.at<double>(2, 2)};
        const double alignment = std::abs(normal_cam.RotateBy(baseToCam.Rotation()).X().value());
        std::vector<frc::Translation3d> candidate;
        for (const auto& point : object) {
            const cv::Mat vector = rotation * (cv::Mat_<double>(3, 1) << point.x, point.y, point.z) + translations[index];
            const frc::Translation3d result{vector.at<double>(0), vector.at<double>(1), vector.at<double>(2)};
            if (!result.IsFinite() || result.Z().value() <= 0.0) { candidate.clear(); break; }
            candidate.push_back(result);
        }
        if (candidate.size() == 4 && alignment > best_alignment) {
            best_alignment = alignment;
            best = std::move(candidate);
        }
    }
    if (best.size() != 4) return std::nullopt;
    return best;
}

frc::Translation3d CamToBase(const frc::Translation3d& p_cam,
                              const frc::Transform3d& baseToCam) {
    return baseToCam.Translation() + p_cam.RotateBy(baseToCam.Rotation());
}

std::vector<frc::Translation3d> ToBase(const std::vector<frc::Translation3d>& pts_cam,
                                        const frc::Transform3d& baseToCam) {
    std::vector<frc::Translation3d> result;
    result.reserve(pts_cam.size());
    for (const auto& point : pts_cam) result.push_back(CamToBase(point, baseToCam));
    return result;
}

std::optional<cv::Point2d> ProjectToImage(const frc::Translation3d& point_base,
                                          const frc::Transform3d& baseToCam,
                                          const CameraConfig& cam) {
    const auto cameraToBase = baseToCam.Inverse();
    const auto point_cam = point_base.RotateBy(cameraToBase.Rotation()) + cameraToBase.Translation();
    if (!point_cam.IsFinite() || point_cam.Z().value() <= 0.05 ||
        !std::isfinite(cam.K(0, 0)) || !std::isfinite(cam.K(1, 1)) ||
        cam.K(0, 0) <= 0.0 || cam.K(1, 1) <= 0.0) return std::nullopt;
    const std::vector<cv::Point3d> points{{point_cam.X().value(), point_cam.Y().value(), point_cam.Z().value()}};
    std::vector<cv::Point2d> pixels;
    try {
        cv::projectPoints(points, cv::Vec3d{}, cv::Vec3d{}, cam.K, cam.D, pixels);
    } catch (const cv::Exception&) { return std::nullopt; }
    if (pixels.size() != 1 || !std::isfinite(pixels[0].x) || !std::isfinite(pixels[0].y) ||
        std::abs(pixels[0].x) > 1e7 || std::abs(pixels[0].y) > 1e7) return std::nullopt;
    return pixels[0];
}

Observation MakeObservation(const VisionFrame& vf,
                            const std::vector<frc::Translation3d>& pts_base,
                            const TargetGeometry& geom) {
    Observation obs{vf.t, 0.0, vf.detected && pts_base.size() == 4};
    if (!obs.detected) return obs;
    frc::Translation3d representative;
    for (const auto& point : pts_base) representative = representative + point;
    representative = representative / static_cast<double>(pts_base.size());
    const auto delta = representative - geom.center;
    obs.phi_obs = std::atan2(delta.Dot(geom.v).value(), delta.Dot(geom.u).value());
    return obs;
}