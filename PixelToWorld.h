#ifndef PIXEL_TO_WORLD_H_
#define PIXEL_TO_WORLD_H_

#include <array>
#include <optional>

#include <opencv2/core.hpp>
#include <turret_kinematics/Pose3d.h>

struct PixelToWorldConfig
{
    // Camera intrinsics in pixels.
    double fx;
    double fy;
    double cx;
    double cy;
    std::array<double, 5> distortion{};

    // Camera pose expressed in the gimbal frame.
    turret_kinematics::Transform3d cameraInGimbal;

    // Gimbal rotation center expressed in the world frame.
    turret_kinematics::Translation3d gimbalOriginInWorld;

    // Target plane in world coordinates: normal dot (X - point) = 0.
    std::array<double, 3> targetPlaneNormalWorld;
    turret_kinematics::Translation3d targetPlanePointWorld;

    // Calibration offsets are in degrees.
    units::degree_t yawOffset;
    units::degree_t pitchOffset;

    // Direction corrections applied to feedback angles.
    double yawSign;
    double pitchSign;

    bool requireIntersectionInFront;

    PixelToWorldConfig();
};

struct GimbalAngles
{
    units::degree_t pitch{0.0};
    units::degree_t yaw{0.0};
};

class PixelToWorld
{
public:
    explicit PixelToWorld(const PixelToWorldConfig& config = PixelToWorldConfig());

    void setConfig(const PixelToWorldConfig& config);
    const PixelToWorldConfig& config() const;
    PixelToWorldConfig& config();

    std::optional<turret_kinematics::Translation3d> projectPixelToWorld(
        const cv::Point2d& pixel,
        const GimbalAngles& gimbalAngles) const;

    std::optional<cv::Point2d> worldToPixel(
        const turret_kinematics::Translation3d& point,
        const GimbalAngles& angles) const;

private:
    PixelToWorldConfig config_;
};

#endif // PIXEL_TO_WORLD_H_