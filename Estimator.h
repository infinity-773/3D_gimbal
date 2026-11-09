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
class Estimator {
public:
    Estimator();

    void calc(frc::Transform3d targetInBase , double now){
        //update history
        if (history.size() >= 20) {
            history.pop_front();
        }
        history.push_back(targetInBase);
        if(history.size() < 3) return;

        int n = history.size();
        //遍历每一个历史目标点
        for (int p = 0; p < history.size(); ++p) {
            int raws = n+3;

            //雅可比jacobain ：目标点与圆心距离在某个轴上分量 / 目标点到圆心距离 
            //行为点 ， 列为轴
            cv::Mat jacobian(raws, 2, CV_64F);

            //残差res：目标点与圆心距离在某个轴上分量 - 目标点到圆心距离
            //行为点 ， 列为轴
            cv::Mat residuals(raws, 1, CV_64F);

            //huber权重：if abs(res) < huber -> weight = 1,
            //else weight = huber / abs(res)
            //N*N对角矩阵，对角元素是每个目标点的权重
            cv::Mat weights(raws, 1, CV_64F);

            //遍历每一个点在轨迹平面的俩轴
            for(int i = 1 ; i <= 2; ++i) {
                //点到C距离
                double dx = history[p].X().value();
                double dy = history[p].Y().value();
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < 1e-9 ) {dist = 1e-9;}
                //
            }


        }
    };


private:
    ModelData priorData{frc::Translation3d{}, 0.275, PI / 5};
    std::deque<frc::Transform3d> history;
    double rms; //几何残差
    bool valid = false;
    double lambda = 0.3;
    double huber = 0.05;



};



#endif // ESTIMATOR_H