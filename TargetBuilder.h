#ifndef TARGET_BUILDER_H
#define TARGET_BUILDER_H

#include <chrono>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

#include "turret_kinematics/TurretKinematics.h"

struct CameraIntrinsics
{
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    std::vector<double> distCoeffs;
};

struct CircularTargetPath
{
    // Path center in the base frame, in meters (+X forward, +Y left, +Z up).
    turret_kinematics::Translation3d pathCtrInBase;
    units::meter_t pathRad{0.0};
    // Maps the path's local XY plane (normal +Z) into the base frame.
    turret_kinematics::Rotation3d planeRotInBase;
};

struct TargetEstimate
{
    turret_kinematics::Translation3d posInBase;
    units::radian_t pathAngle{0.0};
    double reprojErrPx = 0.0;
    units::radian_t yawBearingTarget{0.0};
    units::radian_t pitchBearingTarget{0.0};
    units::radian_t yawTarget{0.0};
    units::radian_t pitchTarget{0.0};
    units::second_t flightTime{0.0};
};

class TargetBuilder
{
public:
    TargetBuilder(CameraIntrinsics camIntrinsics,
                  CircularTargetPath tgtPath,
                  units::meter_t tgtRad,
                  units::second_t targetLeadTime,
                  double yawZeroDeg,
                  double pitchZeroDeg,
                  double yawSign,
                  double pitchSign,
                  // Reject PnP fits above this RMS pixel error.
                  double maxReprojErrPx = 5.0,
                  // Reject detections this far from the configured path.
                  double maxPathErrM = 0.15,
                  // Fixed angular speed magnitude in rad/s; zero estimates it from frames.
                  double targetAngularSpeedRadPerSec = 0.0);

                  bool IsConfigured() const;

    // The box is assumed to enclose a circle; both poses are expressed in the base frame.
    std::optional<TargetEstimate> EstimateTarget(
        const cv::Rect& tgtBox,
        const turret_kinematics::Pose3d& camPose,
        const turret_kinematics::Pose3d& muzzlePose,
        std::chrono::steady_clock::time_point captureTime);

private:
    CameraIntrinsics camIntrinsics_;
    CircularTargetPath tgtPath_;
    units::meter_t tgtRad_;
    units::second_t targetLeadTime_;
    double yawZeroDeg_;
    double pitchZeroDeg_;
    double yawSign_;
    double pitchSign_;
    double maxReprojErrPx_;
    double maxPathErrM_;
    double targetAngularSpeedRadPerSec_;
    std::optional<units::radian_t> prevPathAngle_;
    std::chrono::steady_clock::time_point prevCaptureTime_{};
};

#endif // TARGET_BUILDER_H
