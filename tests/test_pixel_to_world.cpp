#include "PixelToWorld.h"

#include <cmath>
#include <iostream>
#include <vector>

#include <opencv2/calib3d.hpp>

int main()
{
    PixelToWorldConfig config;
    config.fx = 1300.0;
    config.fy = 1250.0;
    config.cx = 640.0;
    config.cy = 360.0;

    const GimbalAngles angles{units::degree_t{8.0}, units::degree_t{-12.0}};
    const turret_kinematics::Pose3d gimbalPose{
        config.gimbalOriginInWorld,
        turret_kinematics::Rotation3d{0.0, 8.0 * CV_PI / 180.0, -12.0 * CV_PI / 180.0}};
    const auto cameraPose = gimbalPose.TransformBy(config.cameraInGimbal);
    const turret_kinematics::Translation3d pointInCamera{0.55, -0.4, 2.0};
    const auto expected = cameraPose.Translation() + pointInCamera.RotateBy(cameraPose.Rotation());
    config.targetPlanePointWorld = expected;

    const cv::Matx33d cameraMatrix{
        config.fx, 0.0, config.cx, 0.0, config.fy, config.cy, 0.0, 0.0, 1.0};
    const std::array<std::array<double, 5>, 2> coefficients{{
        {},
        {0.01986204680694741, 0.01055574619482257, -0.0002627896452722419,
         9.333222065562593e-05, -0.07626694416768789}}};
    for (const auto& values : coefficients)
    {
        config.distortion = values;
        const cv::Vec<double, 5> distortion{values[0], values[1], values[2], values[3], values[4]};
        std::vector<cv::Point2d> pixels;
        cv::projectPoints(std::vector<cv::Point3d>{{0.55, -0.4, 2.0}},
                          cv::Vec3d{}, cv::Vec3d{}, cameraMatrix, distortion, pixels);
        const PixelToWorld converter{config};
        const auto actual = converter.projectPixelToWorld(pixels[0], angles);
        if (!actual || (expected - *actual).Norm().value() > 1e-8)
        {
            std::cerr << "Pixel-to-world distortion round trip failed\n";
            return 1;
        }
        const auto projected = converter.worldToPixel(expected, angles);
        if (!projected || cv::norm(*projected - pixels[0]) > 1e-8)
        {
            std::cerr << "World-to-pixel distortion projection failed\n";
            return 1;
        }
        const auto behindCamera = cameraPose.Translation() +
            turret_kinematics::Translation3d{0.0, 0.0, -1.0}.RotateBy(cameraPose.Rotation());
        if (converter.worldToPixel(behindCamera, angles))
        {
            std::cerr << "Point behind camera was projected\n";
            return 1;
        }
    }
    return 0;
}