#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include "Serial.h"
#include <iostream>
#include <mutex>
#include <thread>
#include <atomic>
#include <Constants.h>
class CenterEstimator {
public:
    CenterEstimator( frc ::Translation3d perior ,double r):
    periorData(perior),
    expectedCenter(periorData),
    res(std::numeric_limits<double>::infinity()),
    lambda(kLAMBDA),
    huber(kHUBER),
    radius(r) {};

    void calcCenter(frc::Translation3d targetInBase , double now){
        const double targetX = targetInBase.X().value();
        const double targetY = targetInBase.Y().value();
        const double targetZ = targetInBase.Z().value();
        if (!std::isfinite(targetX) || !std::isfinite(targetY) ||
            !std::isfinite(targetZ) || !std::isfinite(radius) || radius <= 0.0)
        {
            return;
        }

        //update targets

        if (targets
            .size() >= kPOINTS_NUM) {
            targets
            .pop_front();
        }
        targets
        .push_back(targetInBase);
        
        if (targets.size() < 10)
        {
            res = std::numeric_limits<double>::infinity();
            return;
        }

        //目前点数量
        int n = targets.size();
        int raws = n+2;
        // 最多迭代20次 ，每一次计算一个更优的Δ ，直到期望值C小于阈值
        for (int iter = 0 ; iter < 20; ++iter){
            //雅可比jacobain（每个点残差对各轴上个分量的导数列成矩阵） ：目标点与圆心距离在某个轴上分量 / 目标点到圆心距离 
            //行为点 ， 列为轴
            cv::Mat jacobian(raws, 2, CV_64F);

            //残差res：目标点与圆心距离在某个轴上分量 - 目标点到圆心距离
            //行为点 ， 列为轴
            cv::Mat residuals(raws, 1, CV_64F);

            //遍历每一个历史目标点
            for (int p = 0; p < targets.size(); ++p) {

                //点到C在轨迹平面的上的距离
                double dz = targets[p].Z().value() - expectedCenter.Z().value();
                double dy = targets[p].Y().value() - expectedCenter.Y().value();
                double dist = std::sqrt(dz * dz + dy * dy);
                //点的残差
                if (dist < 1e-12 ) {dist = 1e-12;}
                const double residual = dist - radius;
                //该点权重开根号
                const double residualMagnitude = std::abs(residual);
                const double sw = std::sqrt(
                    residualMagnitude <= huber
                        ? 1.0
                        : huber / std::max(residualMagnitude, 1e-15));

                //填入矩阵
                residuals.at<double>(p,0) = residual * sw;
                jacobian.at<double>(p,0) = (-dz/dist) * sw;
                jacobian.at<double>(p,1) = (-dy/dist) * sw;
            }

            //矩阵后两行填入先验数据
            //先验数据的权重开根号
            double sl = std::sqrt(lambda);
            residuals.at<double>(n,0) = (expectedCenter.Z().value() - periorData.Z().value()) * sl;
            residuals.at<double>(n+1,0) = (expectedCenter.Y().value() - periorData.Y().value()) * sl;
            jacobian.at<double>(n,0) = sl;
            jacobian.at<double>(n+1,1) = sl;

            //At*A Δ = -At*b, 求解增量Δ
            cv::Mat delta;
            if (!cv::solve(jacobian, -residuals, delta, cv::DECOMP_SVD) ||
                delta.rows != 2 || delta.cols != 1)
            {
                return;
            }

            const double deltaZ = delta.at<double>(0, 0);
            const double deltaY = delta.at<double>(1, 0);
            if (!std::isfinite(deltaZ) || !std::isfinite(deltaY))
            {
                return;
            }

            const double stepNorm = std::hypot(deltaY, deltaZ);
            constexpr double kMaxStepMeters = 1;
            const double stepScale = stepNorm > kMaxStepMeters
                ? kMaxStepMeters / stepNorm
                : 1.0;
            const double boundedDeltaZ = deltaZ * stepScale;
            const double boundedDeltaY = deltaY * stepScale;

            expectedCenter = frc::Translation3d{
                    expectedCenter.X().value(),
                    expectedCenter.Y().value() + boundedDeltaY,
                    expectedCenter.Z().value() + boundedDeltaZ
            };
            //Δ小于阈值，结束本周期迭代
            
            if(std::hypot(boundedDeltaY, boundedDeltaZ) < 1e-12) break;
        }

        double squaredResidualSum = 0.0;
        for (const auto& target : targets)
        {
            const double dz = target.Z().value() - expectedCenter.Z().value();
            const double dy = target.Y().value() - expectedCenter.Y().value();
            const double residual = std::hypot(dz, dy) - radius;
            squaredResidualSum += residual * residual;
        }
        res = std::sqrt(squaredResidualSum / static_cast<double>(targets.size()));
    };

    

    frc::Translation3d getExpCenter() const {
        return expectedCenter;
    }

    double getRes() const {
        return res;
    }

    bool hasEnoughSamples() const {
        return targets.size() >= 10;
    }

    void resetCenter()
    {
        targets.clear();
        expectedCenter = periorData;
        res = std::numeric_limits<double>::infinity();
    }


private:
    frc::Translation3d periorData;
    std::deque<frc::Translation3d> targets;
    frc::Translation3d expectedCenter;
    double res = 10; //几何残差
    double lambda ;
    double huber ;
    double radius ;

};

struct OmegaSample
{
    double timeSec;
    double phaseRad;
};

class OmegaEstimator
{
public:
    // slotId 是 target 当前对应的靶槽位，<0 表示未知。圆心拟合有误差时，不同槽位的观测角相对
    // 理想 72° 间隔会有突变，换槽位的那一帧不能把突变算进角速度斜率。
    bool updateOmega(int now, const frc::Translation3d& target,
                     const frc::Translation3d& center3D, int slotId = -1)
    {
        const auto angle = ToYZ(target, center3D).Angle();
        if (!angle)
        {
            reset();
            return false;
        }

        const double timeSec = static_cast<double>(now) * 0.001;
        const double wrappedPhase = angle->Radians().value();
        if (samples_.empty())
        {
            samples_.push_back({timeSec, wrappedPhase});
            lastWrappedPhase_ = wrappedPhase;
            lastSlotId_ = slotId;
            ready_ = false;
            return false;
        }

        constexpr double slotAngle = 2.0 * std::numbers::pi / 5.0;
        double delta = std::remainder(
            wrappedPhase - lastWrappedPhase_, slotAngle);
        const bool slotChanged = slotId >= 0 && lastSlotId_ >= 0 && slotId != lastSlotId_;
        if (slotChanged && ready_)
        {
            // 换靶：用上一段拟合的斜率外推，吸收掉两个槽位之间的角度突变。
            delta = lastFittedOmega_ * (timeSec - samples_.back().timeSec);
        }
        const double unwrappedPhase = samples_.back().phaseRad + delta;
        lastWrappedPhase_ = wrappedPhase;
        lastSlotId_ = slotId;
        samples_.push_back({timeSec, unwrappedPhase});
        // 按时间而不是按帧数取窗口，帧率高时才不会因窗口过短而噪声变大。
        const double kFitWindowSec = kFIT_WINDOW_SEC;
        while (samples_.size() > 8 &&
               samples_.back().timeSec - samples_.front().timeSec > kFitWindowSec)
        {
            samples_.pop_front();
        }
        if (samples_.size() < 8 ||
            samples_.back().timeSec - samples_.front().timeSec < 0.1)
        {
            ready_ = false;
            return false;
        }

        const double count = static_cast<double>(samples_.size());
        double meanTime = 0.0;
        double meanPhase = 0.0;
        for (const auto& sample : samples_)
        {
            meanTime += sample.timeSec / count;
            meanPhase += sample.phaseRad / count;
        }
        double timeVariance = 0.0;
        double covariance = 0.0;
        for (const auto& sample : samples_)
        {
            const double dt = sample.timeSec - meanTime;
            timeVariance += dt * dt;
            covariance += dt * (sample.phaseRad - meanPhase);
        }
        if (timeVariance <= 1e-12)
        {
            ready_ = false;
            return false;
        }

        const double fittedOmega = covariance / timeVariance;
        if (!std::isfinite(fittedOmega) || std::abs(fittedOmega) < 1e-6)
        {
            ready_ = false;
            return false;
        }
        lastFittedOmega_ = fittedOmega;
        measuredOmega_ = std::abs(fittedOmega);
        matchedOmega_ = matchOmega(measuredOmega_, now);
        ready_ = true;
        return true;
    }

    void reset()
    {
        samples_.clear();
        lastWrappedPhase_ = 0.0;
        lastSlotId_ = -1;
        lastFittedOmega_ = 0.0;
        measuredOmega_ = 0.0;
        pendingOmega_ = matchedOmega_;
        pendingFrames_ = 0;
        locked_ = false;
        ready_ = false;
    }

    // 累计检测到的角速度切换次数（首次锁定不算切换）。
    int switchCount() const
    {
        return switchCount_;
    }

    // 最近一次切换的起点：新角速度候选首次出现的时刻，已扣除去抖帧数。
    int lastSwitchOriginMs() const
    {
        return switchOriginMs_;
    }

    double getOmega() const
    {
        return matchedOmega_;
    }

    double getMeasuredOmega() const
    {
        return measuredOmega_;
    }

    bool ready() const
    {
        return ready_;
    }

private:
    double matchOmega(double measuredOmega, int now)
    {
        const double candidate = *std::min_element(
            kTARGET_OMEGAS.begin(), kTARGET_OMEGAS.end(),
            [measuredOmega](double lhs, double rhs) {
                return std::abs(lhs - measuredOmega) <
                       std::abs(rhs - measuredOmega);
            });
        const double kSwitchMargin = 0.20;
        const int kSwitchStableFrames = kSWITCH_STABLE_FRAMES;
        const int kSwitchStableMs = kSWITCH_STABLE_MS;
        // 初始值只是默认假设，首次拟合直接采用，不计为一次切换。
        if (!locked_)
        {
            locked_ = true;
            pendingOmega_ = candidate;
            pendingFrames_ = 0;
            return candidate;
        }
        const double improvement =
            std::abs(matchedOmega_ - measuredOmega) -
            std::abs(candidate - measuredOmega);
        if (candidate == matchedOmega_ || improvement < kSwitchMargin)
        {
            pendingOmega_ = matchedOmega_;
            pendingFrames_ = 0;
            return matchedOmega_;
        }

        if (std::abs(candidate - pendingOmega_) < 1e-9)
            ++pendingFrames_;
        else
        {
            pendingOmega_ = candidate;
            pendingFrames_ = 1;
            pendingStartMs_ = now;
        }
        if (pendingFrames_ >= kSwitchStableFrames &&
            now - pendingStartMs_ >= kSwitchStableMs)
        {
            ++switchCount_;
            switchOriginMs_ = pendingStartMs_;
            return candidate;
        }
        return matchedOmega_;
    }

    std::deque<OmegaSample> samples_;
    double lastWrappedPhase_ = 0.0;
    int lastSlotId_ = -1;
    double lastFittedOmega_ = 0.0;
    double measuredOmega_ = 0.0;
    double matchedOmega_ = kTARGET_OMEGAS[1];
    double pendingOmega_ = kTARGET_OMEGAS[1];
    int pendingFrames_ = 0;
    int pendingStartMs_ = 0;
    int switchOriginMs_ = 0;
    int switchCount_ = 0;
    bool locked_ = false;
    bool ready_ = false;
};




#endif // ESTIMATOR_H