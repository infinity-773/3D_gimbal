#ifndef _CONSTANTS_H
#define _CONSTANTS_H

#include <turret_kinematics/TurretKinematics.h>
#include <MathUtils.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace frc {
using Pose2d = turret_kinematics::Pose2d;
using Translation2d = turret_kinematics::Translation2d;
using Translation3d = turret_kinematics::Translation3d;
using Transform2d = turret_kinematics::Transform2d;
using Rotation2d = turret_kinematics::Rotation2d;
using Rotation3d = turret_kinematics::Rotation3d;
using Transform3d = turret_kinematics::Transform3d;
}

//*************************************场地常量**********************************/
//坐标 frc为右手系 x前，y左，z上   
static const frc::Translation3d kPITCH_AXIS_IN_BASE{0.0, 0.0, 0.0};
static const frc::Translation3d kYAW_AXIS_IN_BASE{0.010, 0.0, 0.0};
// Camera position relative to the yaw-axis origin.
static const frc::Translation3d kCAM_IN_YAW{0.00934 , 0 , 0.1102};
static const frc::Translation3d kSHOOTER_IN_BASE{0 , 0 , 0.0622};
static constexpr frc::Translation3d kCAM_IN_SHOOTER = kCAM_IN_YAW - kSHOOTER_IN_BASE;  //使用自定义 “-” 操作符
static const frc::Translation3d kCENTER_IN_BASE{2.0 , -0.03 , 0.290};
//轨迹半径
static const double kTRAJECTORY_RADIUS = 0.210;
static const double kSTATIC_OMEGA = std::numbers::pi / 3;

//********************************炮台零点偏移 ；feedBack + offset = trueDegs************************************/
//yaw左正 pitch下正
static const AxisData kSHOOTER_ZERO_1 = AxisData {.yaw=0.0 , .pitch=0.0};
static const AxisData kSHOOTER_ZERO_2 = AxisData {.yaw=0.0 , .pitch=0.0};
static const AxisData kSHOOTER_ZERO_3 = AxisData {.yaw=0.0 , .pitch=0.0};
static const AxisData kSHOOTER_ZERO_4 = AxisData {.yaw=0.0 , .pitch=0.0};
static const AxisData kSHOOTER_ZERO_5 = AxisData {.yaw=0.0 , .pitch=0.0};

//***************************************HSV****************************************** */
// 蓝色
static const cv::Scalar blue_low  = cv::Scalar(90, 90, 35);   
static const cv::Scalar blue_high = cv::Scalar(130, 255, 255);
// 红色 H 跨两头 分两段
static const cv::Scalar red_ranges_low[2]  = {cv::Scalar(0, 130, 40),   cv::Scalar(11, 255, 255)};  
static const cv::Scalar red_ranges_high[2] = {cv::Scalar(168, 130, 40), cv::Scalar(180, 255, 255)};
//黄色 
static const cv::Scalar yellow_low  = cv::Scalar(5, 50, 30);   
static const cv::Scalar yellow_high = cv::Scalar(50, 180, 200);

//************************************相机参数********************************** */
 // 畸变系数，无量纲
static constexpr std::array<double, 5> kCamDistD{0.01986204680694741, 0.01055574619482257, -0.0002627896452722419, 9.333222065562593e-05,  -0.07626694416768789};
 // 焦距等效像素
static const double kFocalPx = 1300;

//************************************火控参数*************************************** */
static const double kAIM_TOLERANCE = 1.5; // 允许的瞄准误差，单位 deg
static const double kAIM_STABLE_FRAMES = 20;// dont change this
static const int kPushRetractMs = 110; // 推杆回缩脉冲时长
// 跟随速度 = kp * 角度误差，再限制在 [下限, 上限]；离目标远时自动加速，靠近时降到下限。
static const double kCommandSlewKp = 5.0;               // 1/s，每 1° 误差对应的速度 deg/s
static const double kCommandSlewMinDegPerSec = 15;      // 跟随速度下限
static const double kCommandSlewMaxDegPerSec = 30;      // 跟随速度上限
// 每档角速度各自的提前时间系数，与 kTARGET_OMEGAS 一一对应；实际提前时间 = 基础提前时间 * 系数。
static constexpr std::array<double, 3> kLEAD_TIME_SCALES{1.0, 0.9, 1.0};

//************************************拟合参数****************************************** */
static const int kPOINTS_NUM = 60;
static const int kMODEL_STABLE_FRAMES = 55;
static const double kHUBER = 0.08;
static const double kLAMBDA = 0.3;
static const double kMAX_RES_TOLERANCE = 0.2;
static const double kMODEL_RES_TOLERANCE = 0.0050;
//目标匹配
static const units::degree_t kTARGET_MATCH_TOLERANCE = units::degree_t{15.0}; // 目标匹配容差
static const int kTARGET_HIT_CONFIRM_FRAMES = 20; // 发射后延迟确认当前目标已命中
static const double kACTIVE_CONFIRM_MS = 200;
static const int kSWITCH_STABLE_FRAMES = 10;
static const int kSWITCH_STABLE_MS = 400;
static const int kOMEGA_PERIOD_MS = 6100; // 角速度切换周期
static const double kFIT_WINDOW_SEC = 0.5;
static const int kOMEGA_FIRE_BLOCK_MS = 700; // 切换前禁用开火的时长
static const double kOMEGA_START_THRESHOLD = 0.6; // 拟合角速度(rad/s)由低升过该值，视为开局转动
static const int kOMEGA_START_STABLE_FRAMES = 5; // 角速度连续高于阈值的帧数，用于去抖
static const int kOMEGA_SETTLE_MS = 600; // 检测到角速度从 0 开始增大后，等待角速度从 0 爬升稳定再允许驱动云台和开火的时长
static constexpr double kFIXED_OMEGA = std::numbers::pi / 3.0; // 定速模式的开环角速度，单位 rad/s
static constexpr std::array<double, 3> kTARGET_OMEGAS{
    std::numbers::pi / 3.0,
    std::numbers::pi / 2.0,
    2.0 * std::numbers::pi / 3.0}; // 三档目标角速度，单位 rad/s，仅用于提前量


#endif