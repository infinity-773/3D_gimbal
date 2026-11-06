#ifndef TARGET_TRAJECTORY_DRAWER_H_
#define TARGET_TRAJECTORY_DRAWER_H_

#include "TargetBuilder.h"

class TargetTrajectoryDrawer
{
public:
    TargetTrajectoryDrawer(CameraIntrinsics camIntrinsics,
                           CircularTargetPath tgtPath);

    void Draw(cv::Mat& image,
              const turret_kinematics::Pose3d& camPose) const;

private:
    CameraIntrinsics camIntrinsics_;
    CircularTargetPath tgtPath_;

    bool IsConfigured() const;
};

#endif // TARGET_TRAJECTORY_DRAWER_H_
