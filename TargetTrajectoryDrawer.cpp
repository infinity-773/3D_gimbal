#include "TargetTrajectoryDrawer.h"

#include <cmath>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
constexpr int kPathSegments = 128;
constexpr double kMinDepthM = 0.05;
constexpr double kMaxPixelCoord = 1.0e7;
const cv::Scalar kPathColor{255, 255, 0};
}

TargetTrajectoryDrawer::TargetTrajectoryDrawer(
    CameraIntrinsics camIntrinsics,
    CircularTargetPath tgtPath)
    : camIntrinsics_{std::move(camIntrinsics)},
      tgtPath_{std::move(tgtPath)}
{
}

bool TargetTrajectoryDrawer::IsConfigured() const
{
    const auto distCount = camIntrinsics_.distCoeffs.size();
    const bool validDistCount =
        distCount == 0 || distCount == 4 || distCount == 5 ||
        distCount == 8 || distCount == 12 || distCount == 14;
    for (double coeff : camIntrinsics_.distCoeffs)
    {
        if (!std::isfinite(coeff))
        {
            return false;
        }
    }

    return std::isfinite(camIntrinsics_.fx) && camIntrinsics_.fx > 0.0 &&
           std::isfinite(camIntrinsics_.fy) && camIntrinsics_.fy > 0.0 &&
           std::isfinite(camIntrinsics_.cx) &&
           std::isfinite(camIntrinsics_.cy) &&
           validDistCount && tgtPath_.pathCtrInBase.IsFinite() &&
           std::isfinite(tgtPath_.pathRad.value()) &&
           tgtPath_.pathRad > units::meter_t{0.0};
}

void TargetTrajectoryDrawer::Draw(
    cv::Mat& image,
    const turret_kinematics::Pose3d& camPose) const
{
    if (image.empty() || !IsConfigured())
    {
        return;
    }

    const turret_kinematics::Transform3d baseToCam{
        camPose, turret_kinematics::Pose3d{}};
    std::vector<cv::Point3d> visiblePts;
    std::vector<int> visibleIndices;
    visiblePts.reserve(kPathSegments);
    visibleIndices.reserve(kPathSegments);

    for (int i = 0; i < kPathSegments; ++i)
    {
        const double angle = 2.0 * CV_PI * i / kPathSegments;
        const turret_kinematics::Translation3d tgtPosOnPath{
            tgtPath_.pathRad.value() * std::cos(angle),
            tgtPath_.pathRad.value() * std::sin(angle),
            0.0};
        const auto tgtPosInBase = tgtPath_.pathCtrInBase +
            tgtPosOnPath.RotateBy(tgtPath_.planeRotInBase);
        const auto tgtPosInCam =
            tgtPosInBase.RotateBy(baseToCam.Rotation()) +
            baseToCam.Translation();
        const double zCam = tgtPosInCam.Z().value();
        if (std::isfinite(zCam) && zCam > kMinDepthM)
        {
            visiblePts.emplace_back(tgtPosInCam.X().value(),
                                    tgtPosInCam.Y().value(), zCam);
            visibleIndices.push_back(i);
        }
    }

    if (visiblePts.size() < 2)
    {
        return;
    }

    const cv::Mat camMatrix = (cv::Mat_<double>(3, 3) <<
        camIntrinsics_.fx, 0.0, camIntrinsics_.cx,
        0.0, camIntrinsics_.fy, camIntrinsics_.cy,
        0.0, 0.0, 1.0);
    const cv::Mat distCoeffs = camIntrinsics_.distCoeffs.empty()
        ? cv::Mat{}
        : cv::Mat{camIntrinsics_.distCoeffs};
    std::vector<cv::Point2d> projectedPts;
    cv::projectPoints(visiblePts, cv::Vec3d{0.0, 0.0, 0.0},
                      cv::Vec3d{0.0, 0.0, 0.0},
                      camMatrix, distCoeffs, projectedPts);

    std::vector<cv::Point2d> pathPixels(kPathSegments);
    std::vector<bool> isVisible(kPathSegments, false);
    for (std::size_t i = 0; i < projectedPts.size(); ++i)
    {
        const auto& pt = projectedPts[i];
        if (std::isfinite(pt.x) && std::isfinite(pt.y) &&
            std::abs(pt.x) <= kMaxPixelCoord &&
            std::abs(pt.y) <= kMaxPixelCoord)
        {
            const int pathIdx = visibleIndices[i];
            pathPixels[pathIdx] = pt;
            isVisible[pathIdx] = true;
        }
    }

    for (int i = 0; i < kPathSegments; ++i)
    {
        const int next = (i + 1) % kPathSegments;
        if (!isVisible[i] || !isVisible[next])
        {
            continue;
        }

        cv::Point p0{cvRound(pathPixels[i].x), cvRound(pathPixels[i].y)};
        cv::Point p1{cvRound(pathPixels[next].x), cvRound(pathPixels[next].y)};
        if (cv::clipLine(image.size(), p0, p1))
        {
            cv::line(image, p0, p1, kPathColor, 2, cv::LINE_AA);
        }
    }
}
