#include "PixelToWorld.h"

#include <cmath>

namespace
{
double dot3(const std::array<double, 3>& a,
            const turret_kinematics::Translation3d& b)
{
    return a[0] * b.X().value() + a[1] * b.Y().value() + a[2] * b.Z().value();
}

double norm3(const std::array<double, 3>& v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
} // namespace

PixelToWorldConfig::PixelToWorldConfig()
    : fx(1300.0),
      fy(1300.0),
      cameraInGimbal{
          turret_kinematics::Translation3d{0.0, 0.0, 0.0},
          turret_kinematics::Rotation3d::FromDegrees(-90.0, 0.0, -90.0)},
      gimbalOriginInWorld{0.0, 0.0, 0.0},
      targetPlaneNormalWorld{0.0, 0.0, 1.0},
      targetPlanePointWorld{0.0, 0.0, 2.0},
      yawOffset{0.0},
      pitchOffset{0.0},
      yawSign(1.0),
      pitchSign(1.0),
      requireIntersectionInFront(true)
      angles_in_degrees(true),
      require_intersection_in_front(true)
{
}

PixelToWorld::PixelToWorld(const PixelToWorldConfig& config)
    : config_(config)
{
}

void PixelToWorld::setConfig(const PixelToWorldConfig& config)
{
    config_ = config;
}

const PixelToWorldConfig& PixelToWorld::config() const
{
    return config_;
}

PixelToWorldConfig& PixelToWorld::config()
{
    return config_;
}

std::optional<turret_kinematics::Translation3d> PixelToWorld::projectPixelToWorld(
    const cv::Point2d& pixel,
    const GimbalAngles& gimbalAngles) const
{
    if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) ||
        !std::isfinite(gimbalAngles.pitch.value()) ||
        !std::isfinite(gimbalAngles.yaw.value()) ||
        std::fabs(config_.fx) < 1e-12 || std::fabs(config_.fy) < 1e-12)
    {
        return std::nullopt;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double yawRadians =
        config_.yawSign * (gimbalAngles.yaw.value() + config_.yawOffset.value()) * kPi / 180.0;
    const double pitchRadians =
        config_.pitchSign * (gimbalAngles.pitch.value() + config_.pitchOffset.value()) * kPi / 180.0;

    const turret_kinematics::Pose3d gimbalPoseInWorld{
        config_.gimbalOriginInWorld,
        turret_kinematics::Rotation3d{0.0, pitchRadians, yawRadians}};
    const turret_kinematics::Pose3d cameraPoseInWorld =
        gimbalPoseInWorld.TransformBy(config_.cameraInGimbal);

    const turret_kinematics::Translation3d rayInCamera{
        (pixel.x - config_.cx) / config_.fx,
        (pixel.y - config_.cy) / config_.fy,
        1.0};
    const turret_kinematics::Translation3d rayInWorld =
        rayInCamera.RotateBy(cameraPoseInWorld.Rotation());

    const double normalNorm = norm3(config_.targetPlaneNormalWorld);
    if (normalNorm < 1e-12)
    {
        return std::nullopt;
    }

    std::array<double, 3> normal{
        config_.targetPlaneNormalWorld[0] / normalNorm,
        config_.targetPlaneNormalWorld[1] / normalNorm,
        config_.targetPlaneNormalWorld[2] / normalNorm};
    const double denominator = dot3(normal, rayInWorld);
    if (std::fabs(denominator) < 1e-12)
    {
        return std::nullopt;
    }

    const auto cameraOriginInWorld = cameraPoseInWorld.Translation();
    const double distance = dot3(
        normal, config_.targetPlanePointWorld - cameraOriginInWorld) / denominator;
    if (config_.requireIntersectionInFront && distance <= 0.0)
    {
        return std::nullopt;
    }

    const auto worldPoint = cameraOriginInWorld + rayInWorld * distance;
    if (!worldPoint.IsFinite())
    {
        return std::nullopt;
    }

    return worldPoint;
}