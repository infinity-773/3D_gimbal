#ifndef _CONSTANTS_H
#define _CONSTANTS_H

#include "TurretKinematics.h"
#include <MathUtils.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace frc {
using Translation2d = turret_kinematics::Translation2d;
using Translation3d = turret_kinematics::Translation3d;
using Rotation2d = turret_kinematics::Rotation2d;
using Rotation3d = turret_kinematics::Rotation3d;
using Transform3d = turret_kinematics::Transform3d;
}

//*************************************场地常量**********************************/
//坐标 frc为右手系 x前，y左，z上   
static const frc::Translation3d kCAM_IN_BASE{0.00934 , 0 , 0.1102};
static const frc::Translation3d kSHOOTER_IN_BASE{0.05139 , 0 , 0.0622};
static constexpr frc::Translation3d kCAM_IN_SHOOTER = kCAM_IN_BASE - kSHOOTER_IN_BASE;  //使用自定义 “-” 操作符
static const frc::Translation3d kCENTER_IN_BASE{2.0 , 0.0 , 0.135};
//轨迹半径
static const double kTRAJECTORY_RADIUS = 0.215;
static const double kOMEGA = std::numbers::pi / 3;

//********************************炮台零点偏移 ；feedBack + offset = trueDegs************************************/
//yaw左正 pitch下正
static const AxisData kSHOOTER_ZERO_1 = AxisData {.yaw=0.0 , .pitch=0.0};
static const AxisData kSHOOTER_ZERO_2 = AxisData {.yaw=0.0 , .pitch=0.0};
static const AxisData kSHOOTER_ZERO_3 = AxisData {.yaw=0.0 , .pitch=0.0};
static const AxisData kSHOOTER_ZERO_4 = AxisData {.yaw=0.0 , .pitch=0.0};
static const AxisData kSHOOTER_ZERO_5 = AxisData {.yaw=0.0 , .pitch=0.0};

//***************************************HSV****************************************** */
// 蓝色
static const cv::Scalar blue_low  = cv::Scalar(100, 180, 200);   
static const cv::Scalar blue_high = cv::Scalar(120, 255, 255);
// 红色 H 跨两头 分两段
static const cv::Scalar red_ranges_low[2]  = {cv::Scalar(0, 110, 137),   cv::Scalar(11, 255, 255)};  
static const cv::Scalar red_ranges_high[2] = {cv::Scalar(168, 110, 137), cv::Scalar(179, 255, 255)};

//************************************相机参数********************************** */
 // 畸变系数，无量纲
static constexpr std::array<double, 5> kCamDistD{0.01986204680694741, 0.01055574619482257, -0.0002627896452722419, 9.333222065562593e-05,  -0.07626694416768789};
 // 焦距等效像素
static const double kFocalPx = 1300;

//************************************火控参数*************************************** */
static const double kAIM_TOLERANCE = 1.9;
static const double kAIM_STABLE_FRAMES = 30;
static const int kPushRetractMs = 200; // 推杆回缩脉冲时长
static const double kCommandSlewRateDegPerSec = 15; // 绝对角度指令最大变化速度

//************************************拟合参数****************************************** */
static const int kMODEL_STABLE_FRAMES = 55;
static const double kHUBER = 0.08;
static const double kLAMBDA = 0.4;
static const double kMAX_RES_TOLERANCE = 0.2;
static const double kMODEL_RES_TOLERANCE = 0.0012;

#endif