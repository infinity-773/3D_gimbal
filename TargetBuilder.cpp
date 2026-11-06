#include "TargetBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <opencv2/calib3d.hpp>

namespace
{
bool IsFinite(double val)
{
    return std::isfinite(val);
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
}

TargetBuilder::TargetBuilder(CameraIntrinsics camIntrinsics,
                             CircularTargetPath tgtPath,
                             units::meter_t tgtRad,
                             units::second_t targetLeadTime,
                             double yawZeroDeg,
                             double pitchZeroDeg,
                             double yawSign,
                             double pitchSign,
                             double maxReprojErrPx,
                             double maxPathErrM,
                             double targetAngularSpeedRadPerSec)
    : camIntrinsics_{std::move(camIntrinsics)},
      tgtPath_{std::move(tgtPath)},
      tgtRad_{tgtRad},
    targetLeadTime_{targetLeadTime},
      yawZeroDeg_{yawZeroDeg},
      pitchZeroDeg_{pitchZeroDeg},
      yawSign_{yawSign},
      pitchSign_{pitchSign},
      maxReprojErrPx_{maxReprojErrPx},
      maxPathErrM_{maxPathErrM},
      targetAngularSpeedRadPerSec_{targetAngularSpeedRadPerSec}
{
}

bool TargetBuilder::IsConfigured() const
{
    const auto distCount = camIntrinsics_.distCoeffs.size();
    const bool validDistCount =
        distCount == 0 || distCount == 4 || distCount == 5 ||
        distCount == 8 || distCount == 12 || distCount == 14;
    const bool finiteDistCoeffs = std::all_of(
        camIntrinsics_.distCoeffs.begin(), camIntrinsics_.distCoeffs.end(),
        [](double coeff) { return IsFinite(coeff); });

    return IsFinite(camIntrinsics_.fx) && camIntrinsics_.fx > 0.0 &&
           IsFinite(camIntrinsics_.fy) && camIntrinsics_.fy > 0.0 &&
           IsFinite(camIntrinsics_.cx) && IsFinite(camIntrinsics_.cy) &&
           validDistCount && finiteDistCoeffs &&
           tgtPath_.pathCtrInBase.IsFinite() &&
           IsFinite(tgtPath_.pathRad.value()) &&
           tgtPath_.pathRad > units::meter_t{0.0} &&
           IsFinite(tgtRad_.value()) && tgtRad_ > units::meter_t{0.0} &&
           IsFinite(targetLeadTime_.value()) &&
           targetLeadTime_ > units::second_t{0.0} &&
           IsFinite(yawZeroDeg_) && IsFinite(pitchZeroDeg_) &&
           IsFinite(yawSign_) && IsFinite(pitchSign_) &&
           std::abs(yawSign_) == 1.0 && std::abs(pitchSign_) == 1.0 &&
           IsFinite(maxReprojErrPx_) && maxReprojErrPx_ > 0.0 &&
           IsFinite(maxPathErrM_) && maxPathErrM_ > 0.0 &&
           IsFinite(targetAngularSpeedRadPerSec_) &&
           targetAngularSpeedRadPerSec_ >= 0.0;
}

std::optional<TargetEstimate> TargetBuilder::EstimateTarget(
    const cv::Rect& tgtBox,
    const turret_kinematics::Pose3d& camPose,
    const turret_kinematics::Pose3d& muzzlePose,
    std::chrono::steady_clock::time_point captureTime)
{
    if (!IsConfigured() || tgtBox.width <= 1 || tgtBox.height <= 1)
    {
        prevPathAngle_.reset();
        return std::nullopt;
    }

    const double pxRad = 0.5 * std::min(tgtBox.width, tgtBox.height);
    const double pxCtrX = tgtBox.x + 0.5 * tgtBox.width;
    const double pxCtrY = tgtBox.y + 0.5 * tgtBox.height;
    const std::array<cv::Point2d, 4> tangentPts{
        cv::Point2d{pxCtrX + pxRad, pxCtrY},
        cv::Point2d{pxCtrX, pxCtrY - pxRad},
        cv::Point2d{pxCtrX - pxRad, pxCtrY},
        cv::Point2d{pxCtrX, pxCtrY + pxRad}};

    const double tgtRad = tgtRad_.value();
    const std::vector<cv::Point3d> objPts{
        {tgtRad, 0.0, 0.0},
        {0.0, tgtRad, 0.0},
        {-tgtRad, 0.0, 0.0},
        {0.0, -tgtRad, 0.0}};
    const std::vector<cv::Point2d> imgPts(tangentPts.begin(),
                                          tangentPts.end());

    const cv::Mat camMatrix = (cv::Mat_<double>(3, 3) <<
        camIntrinsics_.fx, 0.0, camIntrinsics_.cx,
        0.0, camIntrinsics_.fy, camIntrinsics_.cy,
        0.0, 0.0, 1.0);
    const cv::Mat distCoeffs = camIntrinsics_.distCoeffs.empty()
        ? cv::Mat{}
        : cv::Mat{camIntrinsics_.distCoeffs};

    std::vector<cv::Mat> rotVecs;
    std::vector<cv::Mat> transVecs;
    const int solCount = cv::solvePnPGeneric(
        objPts, imgPts, camMatrix, distCoeffs,
        rotVecs, transVecs, false, cv::SOLVEPNP_IPPE);
    if (solCount <= 0)
    {
        prevPathAngle_.reset();
        return std::nullopt;
    }

    double bestErrSq = std::numeric_limits<double>::infinity();
    cv::Mat bestRotVec;
    cv::Mat bestTrans;
    for (std::size_t idx = 0; idx < rotVecs.size(); ++idx)
    {
        cv::Mat rotVec;
        cv::Mat transVec;
        rotVecs[idx].convertTo(rotVec, CV_64F);
        transVecs[idx].convertTo(transVec, CV_64F);

        cv::Mat rotMat;
        cv::Rodrigues(rotVec, rotMat);
        bool allInFront = true;
        for (const auto& pt : objPts)
        {
            const double zCam = rotMat.at<double>(2, 0) * pt.x +
                                rotMat.at<double>(2, 1) * pt.y +
                                rotMat.at<double>(2, 2) * pt.z +
                                transVec.at<double>(2);
            if (!IsFinite(zCam) || zCam <= 0.0)
            {
                allInFront = false;
                break;
            }
        }
        if (!allInFront)
        {
            continue;
        }

        std::vector<cv::Point2d> projPts;
        cv::projectPoints(objPts, rotVec, transVec,
                          camMatrix, distCoeffs, projPts);

        double errSq = 0.0;
        for (std::size_t i = 0; i < projPts.size(); ++i)
        {
            const cv::Point2d err = projPts[i] - imgPts[i];
            errSq += err.dot(err);
        }
        if (errSq < bestErrSq)
        {
            bestErrSq = errSq;
            bestRotVec = rotVec;
            bestTrans = transVec;
        }
    }

    if (bestRotVec.empty() || bestTrans.empty() || !IsFinite(bestErrSq))
    {
        prevPathAngle_.reset();
        return std::nullopt;
    }
    const double reprojErrPx =
        std::sqrt(bestErrSq / static_cast<double>(imgPts.size()));
    if (!IsFinite(reprojErrPx) || reprojErrPx > maxReprojErrPx_)
    {
        prevPathAngle_.reset();
        return std::nullopt;
    }

    const turret_kinematics::Translation3d tgtPosInCam{
        bestTrans.at<double>(0), bestTrans.at<double>(1),
        bestTrans.at<double>(2)};
    const double rotAngle = cv::norm(bestRotVec);
    turret_kinematics::Rotation3d tgtRotInCam;
    if (rotAngle > 1e-12)
    {
        tgtRotInCam = turret_kinematics::Rotation3d{
            std::array<double, 3>{
                bestRotVec.at<double>(0) / rotAngle,
                bestRotVec.at<double>(1) / rotAngle,
                bestRotVec.at<double>(2) / rotAngle},
            units::radian_t{rotAngle}};
    }
    const turret_kinematics::Transform3d tgtToCam{
        tgtPosInCam, tgtRotInCam};
    const turret_kinematics::Pose3d tgtPoseInBase =
        camPose.TransformBy(tgtToCam);
    const auto tgtCtrPosInBase = tgtPoseInBase.Translation();

    const auto tgtCtrOffsetInBase = tgtCtrPosInBase - tgtPath_.pathCtrInBase;
    const auto tgtCtrOffsetInPath =
        tgtCtrOffsetInBase.RotateBy(-tgtPath_.planeRotInBase);
    if (std::hypot(tgtCtrOffsetInPath.X().value(),
                   tgtCtrOffsetInPath.Y().value()) <= 1e-12)
    {
        prevPathAngle_.reset();
        return std::nullopt;
    }
    const units::radian_t pathAngle{
        std::atan2(tgtCtrOffsetInPath.Y().value(),
                   tgtCtrOffsetInPath.X().value())};
    const turret_kinematics::Translation3d tgtPosOnPath{
        tgtPath_.pathRad.value() * std::cos(pathAngle.value()),
        tgtPath_.pathRad.value() * std::sin(pathAngle.value()),
        0.0};
    const auto tgtPosInBase = tgtPath_.pathCtrInBase +
        tgtPosOnPath.RotateBy(tgtPath_.planeRotInBase);
    if (tgtCtrPosInBase.Distance(tgtPosInBase).value() > maxPathErrM_)
    {
        prevPathAngle_.reset();
        return std::nullopt;
    }

    if (!prevPathAngle_)
    {
        prevPathAngle_ = pathAngle;
        prevCaptureTime_ = captureTime;
        return std::nullopt;
    }

    const double dt = std::chrono::duration<double>(
        captureTime - prevCaptureTime_).count();
    const double dAngle = std::remainder(
        pathAngle.value() - prevPathAngle_->value(), 2.0 * kPi);
    prevPathAngle_ = pathAngle;
    prevCaptureTime_ = captureTime;
    if (!IsFinite(dt) || dt <= 1e-4 || dt > 0.5)
    {
        return std::nullopt;
    }

    const double omega = targetAngularSpeedRadPerSec_ > 0.0
        ? std::copysign(targetAngularSpeedRadPerSec_, dAngle)
        : dAngle / dt;
    const turret_kinematics::Translation3d tgtVelInPath{
        -tgtPath_.pathRad.value() * std::sin(pathAngle.value()) * omega,
        tgtPath_.pathRad.value() * std::cos(pathAngle.value()) * omega,
        0.0};
    const auto tgtVelInBase = tgtVelInPath.RotateBy(tgtPath_.planeRotInBase);
    const turret_kinematics::Velocity3d tgtVel{
        units::meters_per_second_t{tgtVelInBase.X().value()},
        units::meters_per_second_t{tgtVelInBase.Y().value()},
        units::meters_per_second_t{tgtVelInBase.Z().value()}};

    const auto tgtPosFromMuzzle = tgtPosInBase - muzzlePose.Translation();
    const turret_kinematics::Translation3d predictedTargetFromMuzzle{
        tgtPosFromMuzzle.X().value() +
            tgtVel[0].value() * targetLeadTime_.value(),
        tgtPosFromMuzzle.Y().value() +
            tgtVel[1].value() * targetLeadTime_.value(),
        tgtPosFromMuzzle.Z().value() +
            tgtVel[2].value() * targetLeadTime_.value()};
    const units::radian_t bearingYaw{
        std::atan2(tgtPosFromMuzzle.Y().value(),
                   tgtPosFromMuzzle.X().value())};
    const units::radian_t bearingPitch{
        std::atan2(tgtPosFromMuzzle.Z().value(),
                   std::hypot(tgtPosFromMuzzle.X().value(),
                              tgtPosFromMuzzle.Y().value()))};
    const units::radian_t leadYaw{
        std::atan2(predictedTargetFromMuzzle.Y().value(),
                   predictedTargetFromMuzzle.X().value())};
    const units::radian_t leadPitch{
        std::atan2(predictedTargetFromMuzzle.Z().value(),
                   std::hypot(predictedTargetFromMuzzle.X().value(),
                              predictedTargetFromMuzzle.Y().value()))};

    return TargetEstimate{
        tgtPosInBase,
        pathAngle,
        reprojErrPx,
        units::radian_t{yawZeroDeg_ / kRadToDeg +
                        yawSign_ * bearingYaw.value()},
        units::radian_t{pitchZeroDeg_ / kRadToDeg +
                        pitchSign_ * bearingPitch.value()},
        units::radian_t{yawZeroDeg_ / kRadToDeg +
                        yawSign_ * leadYaw.value()},
        units::radian_t{pitchZeroDeg_ / kRadToDeg +
                        pitchSign_ * leadPitch.value()},
        targetLeadTime_};
}
