#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include "Model.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <cmath>
#include <deque>
#include "Serial.h"
#include <iostream>
#include <mutex>
#include <thread>
#include <atomic>
class CenterEstimator {
public:
    CenterEstimator():
    periorData(frc::Translation3d{2.0 , 0 , 0}),
    expectedCenter(periorData),
    res(0),
    lambda(0.4),
    huber(0.05) {};

    void calc(frc::Transform3d targetInBase , double now){
        //update targets

        if (targets
            .size() >= 20) {
            targets
            .pop_front();
        }
        targets
        .push_back(targetInBase);
        
        if(targets
            .size() < 3) return;

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
                res = dist - radius;
                if (dist < 1e-9 ) {dist = 1e-9;}
                //该点权重开根号
                double sw = std::sqrt(std::abs(res) >= huber ? 1 : huber / std::abs(res));

                //填入矩阵
                residuals.at<double>(p,0) = res * sw;
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
            cv::Mat delta ;
            cv::solve(jacobian.t() * jacobian , -jacobian.t() * residuals, delta , cv::DECOMP_CHOLESKY);

            expectedCenter = frc::Translation3d{
                    expectedCenter.X().value(),
                    expectedCenter.Y().value() + delta.at<double>(1,0),
                    expectedCenter.Z().value() + delta.at<double>(0,0)
            };
            //残差小于阈值，结束本周期迭代
            if(cv::norm(delta) < 1e-5) break;
        }
    };

    

    frc::Translation3d getExpCenter() const {
        return expectedCenter;
    }

    double getRes() const {
        return res;
    }


private:
    frc::Translation3d periorData{frc::Translation3d{2.0 , -0.07 , 0.275}};
    std::deque<frc::Transform3d> targets;
    frc::Translation3d expectedCenter;
    double res = 0; //几何残差
    double lambda = 0.4;
    double huber = 0.05;
    const double radius = 0.275;

};




#endif // ESTIMATOR_H