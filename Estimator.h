#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include "Model.h"
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
class CenterEstimator {
public:
    CenterEstimator( frc ::Translation3d perior ,double r):
    periorData(perior),
    expectedCenter(periorData),
    res(std::numeric_limits<double>::infinity()),
    lambda(0.5),
    huber(0.05),
    radius(r) {};

    void calc(frc::Translation3d targetInBase , double now){
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
            .size() >= 20) {
            targets
            .pop_front();
        }
        targets
        .push_back(targetInBase);
        
        if (targets.size() < 3)
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
                if (dist < 1e-9 ) {dist = 1e-9;}
                const double residual = dist - radius;
                //该点权重开根号
                const double residualMagnitude = std::abs(residual);
                const double sw = std::sqrt(
                    residualMagnitude >= huber
                        ? 1.0
                        : huber / std::max(residualMagnitude, 1e-12));

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
            
            if(std::hypot(boundedDeltaY, boundedDeltaZ) < 1e-7) break;
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
        return targets.size() >= 3;
    }

    void reset()
    {
        targets.clear();
        expectedCenter = periorData;
        res = std::numeric_limits<double>::infinity();
    }


private:
    frc::Translation3d periorData{frc::Translation3d{2.0 , -0.07 , 0.275}};
    std::deque<frc::Translation3d> targets;
    frc::Translation3d expectedCenter;
    double res = 10; //几何残差
    double lambda = 0.3;
    double huber = 0.05;
    double radius = 0.0;

};




#endif // ESTIMATOR_H