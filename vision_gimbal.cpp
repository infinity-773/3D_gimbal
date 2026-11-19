// “禁止修改”是例程基础设施；自己的代码写在“需要编写”区域。

/*---------禁止修改：已有头文件----------*/
#include "Serial.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <cmath>
#include <iostream>

#include <mutex>
#include <thread>
#include <atomic>
/*------------------------------------*/



/*---------可自行添加：算法需要的头文件----------*/
#include "Debouncer.h"
#include "MathUtils.h"
#include "CameraProjection.h"
#include "DashBoard.h"
#include "Estimator.h"
#include <string>
#include <vector>
#include <RingDetector.h>
#include "FireControl.h"
#include "TargetMatcher.h"
#include <AimSolver.h>
/*---------添加区域结束----------*/



/*---------------------可修改：相机和串口开关参数--------------------*/

// 相机编号
// 当前 /dev/video0 是图像节点。
// /dev/video1 是元数据节点，不是第二台相机。
int cameraIndex = 0;

// 图像分辨率
// 本例使用 MJPG，驱动支持以下宽×高组合（不能随意混搭）：
// 1280×720、800×600、640×480、352×288、320×240、176×144。
int width = 1280;
int height = 720;

// 请求帧率
// 上述 MJPG 分辨率均支持 25、30、60 fps；YUYV 仅列出 10 fps。
// 长曝光、USB 带宽等因素仍可能使实际帧率降低。
int fps = 60;

// 串口开关
// 没接下位机时改成 false，只调图像。
bool useSerial = true;

// 串口编号：1 = /dev/ttyUSB_vision，2 = /dev/ttyS1，3 = /dev/ttyS2。
// 使用其他设备路径时，修改 Serial/Serial.cpp 中的 openDevice。
int serialPort = 1;

// 波特率，必须与下位机一致。
int serialBaudRate = 230400;

/*---------参数修改区域结束----------*/



/*---------可修改：相机画面参数（改完重新编译并运行）----------*/

// 【参数来源】
// 2026-10-24 从当前 USB Camera（uvcvideo，/dev/video0）查询，未改动相机设置。
// 以下“默认”是驱动报告的默认值，不是程序选用的调参值。
// 换相机后要重新查询，复查命令：
//   v4l2-ctl -d /dev/video0 --list-ctrls-menus --list-formats-ext
//
// 范围使用 OpenCV 4.6 V4L2 默认的原始数值模式。
// 不要开启环境变量 OPENCV_VIDEOIO_V4L_RANGE_NORMALIZED。


// 【缓冲与延迟】

// 缓冲帧数
// 范围：OpenCV 后端允许 1～10 帧，默认 4；实际分配受驱动限制。
// 作用：设为 1 可以减少排队的旧画面。
// 注意：缓冲不是图像控制项，不能用 --list-ctrls 查询范围。1.0
int cameraBufferSize = 6;


// 【曝光与亮度】

// 曝光模式
// 可选：1 = 手动曝光；3 = 光圈优先自动曝光。
// 默认：3；这台相机不支持 0、2。
int cameraAutoExposure = 1;

// 曝光时间（仅手动曝光时有效）
// 范围：1～10000，步长 1，默认 166。
// 单位：100 us，即 19 = 1.9 ms，166 = 16.6 ms。
// 实际时间范围：0.1～1000 ms。
// 作用：增大通常更亮，但运动拖影更明显，过长还可能降低实际帧率。
int cameraExposure = 8;

// 亮度
// 范围：-64～64，步长 1，默认 0。
// 作用：调整画面整体明暗。
int cameraBrightness = 10;

// 对比度
// 范围：0～100，步长 1，默认 35。
// 作用：调整明暗区域的差别。
int cameraContrast = 55;

// 伽马
// 范围：100～500，步长 1，默认 300。
// 作用：调整中间亮度层次。
int cameraGamma = 300;

// 逆光补偿
// 范围：0～2，步长 1，默认 0。
// 作用：改善逆光环境下的主体亮度。
int cameraBacklight = 2;


// 【颜色与白平衡】

// 饱和度
// 范围：0～100，步长 1，默认 64。
// 作用：调整颜色浓淡，会影响 HSV 阈值。
int cameraSaturation = 100;

// 色调
// 范围：-180～180，步长 1，默认 0。
// 作用：改变画面颜色，通常保持 0。
int cameraHue = 0;

// 自动白平衡
// 可选：false / 0 = 关闭；true / 1 = 开启。
// 默认：开启。
bool cameraAutoWhiteBalance = true;

// 手动白平衡色温
// 范围：2800～6500 K，步长 10 K，默认 4600 K。
// 注意：必须先关闭自动白平衡；开启时驱动将此项标记为 inactive。
int cameraWhiteBalance = 4600;


// 【边缘细节】

// 锐度
// 范围：0～100，步长 1，默认 80。
// 作用：增强边缘；过大可能放大噪点、产生假边缘。
int cameraSharpness = 100;


// 以上是驱动声明的范围，未逐项实测效果。

/*---------相机画面参数修改区域结束----------*/



/*---------需要编写：自己的全局变量和算法参数----------*/
// 在这里添加 HSV 阈值,计时起点等。




// 瞄准点
static double kAimU = width * 0.5;  
static double kAimV = height * 0.5;  

static double kYawSign = 1.0;       // 图像往右 yaw 变大填 1 变小填 -1（需调参）
static double kPitchSign = 1.0;     // 图像往上 pitch 变大填 1 变小填 -1（需调参）
static double kRotationYawSign = 1.0;
static double kRotationPitchSign = 1.0;

static double kAimTolPx = 20.0; // 只用于画图


static std::chrono::steady_clock::time_point g_lastControlAt;
static bool g_modelAccepted = false;
static bool g_commandInitialized = false;
static double g_commandYaw = 0.0;
static double g_commandPitch = 0.0;
static bool hasYellow = true;
bool useOmega = true; // 是否使用模型预测
bool recordHitTargets = true; // 是否记录已击打目标；关闭后所有目标始终视为未激活



Debouncer modelLockDebouncer(kMODEL_STABLE_FRAMES, DebounceEdge::Rising);
static CenterEstimator estimator = CenterEstimator(kCENTER_IN_BASE, kTRAJECTORY_RADIUS);
static frc::Translation3d center = kCENTER_IN_BASE;
static double omega = kTARGET_OMEGAS[1];
static TargetMatcher targetMatcher;
static OmegaEstimator omegaEstimator;



static CameraProjectionConfig makeProjectionConfig()
{
    CameraProjectionConfig config;
    config.fx = kFocalPx;
    config.fy = kFocalPx;
    config.cx = kAimU;
    config.cy = kAimV;
    config.distortion = kCamDistD;

    // Pitch is the outer joint: pitch rotation moves the yaw-axis origin.
    // Camera coordinates below are measured from the yaw axis.
    config.gimbalOriginInWorld = kPITCH_AXIS_IN_BASE;
    config.yawOriginInPitchFrame = kYAW_AXIS_IN_BASE - kPITCH_AXIS_IN_BASE;
    config.gimbalToCamera = turret_kinematics::Transform3d{
        kCAM_IN_YAW,
        turret_kinematics::Rotation3d::FromDegrees(-90.0, 0.0, -90.0)};

    // Placeholder target plane 2 m forward; replace with the measured plane.
    config.targetPlaneNormalWorld = {1.0, 0.0, 0.0};
    config.targetPlanePointWorld = {2.0, 0.0, 0.0};

    config.yawOffset = units::degree_t{kSHOOTER_ZERO_3.yaw};
    config.pitchOffset = units::degree_t{kSHOOTER_ZERO_3.pitch};
    config.yawSign = kRotationYawSign;
    config.pitchSign = kRotationPitchSign;
    config.requireIntersectionInFront = true;
    return config;
}
CameraProjection projection(makeProjectionConfig());



/*---------全局变量编写区域结束----------*/



/*---------禁止修改：全局通信数据和收发线程----------*/
TRoMaC::Uart uart;
TRoMaC::VisionData sendData{};        // 只由主线程填写，修改这个变量即可。
TRoMaC::VisionData latestSendData{};  // 填好后复制到这里，供发送线程读取。
std::mutex sendMutex;
bool sendReady = false;              // 第一份数据填好之前不发送。
std::chrono::steady_clock::time_point sendUpdatedAt;
std::atomic<bool> sendFailed(false);
TRoMaC::GimbalFeedback receiveData{}; // 接收线程更新，读取时要加锁。
std::mutex receiveMutex;             // 保证 yaw、pitch 总是来自同一份数据。
bool hasFeedback = false;
std::chrono::steady_clock::time_point receivedAt;
std::atomic<bool> running(true);     // 主线程、发送线程、接收线程共用的退出开关。


// 独立发送线程：读取最近一次完整结果，每次循环等待 10 ms。
void sendLoop()
{
    while (running.load())
    {
        TRoMaC::VisionData data{};
        bool ready;
        {
            std::lock_guard<std::mutex> lock(sendMutex);
            data = latestSendData;
            // 超过 250 ms 没有新的图像处理结果时暂停发送，避免反复发送旧指令。
            ready = sendReady && std::chrono::steady_clock::now() - sendUpdatedAt <
                                     std::chrono::milliseconds(250);
        } // 复制完就解锁，不在锁内等待串口发送。
        if (ready && !uart.send(data))
        {
            std::cerr << "串口发送失败。\n";
            sendFailed.store(true);
            running.store(false);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// 独立接收线程：等待串口数据不会拖慢相机画面。
void receiveLoop()
{
    while (running.load())
    {
        if (!uart.ReadData())
            continue;

        // 下位机回传命令 0x01，两个 float 的顺序是 yaw、pitch，单位为度。
        TRoMaC::GimbalFeedback data{};
        if (uart.tryGetGimbalFeedback(data) &&
            std::isfinite(data.yaw_angle) && std::isfinite(data.pitch_angle))
        {
            std::lock_guard<std::mutex> lock(receiveMutex);
            receiveData = data;
            receivedAt = std::chrono::steady_clock::now();
            hasFeedback = true;
        } // 离开这对大括号，自动解锁。
    }
}
/*---------禁止修改区域结束----------*/



/*---------需要编写：自己的算法函数（按需添加）----------*/
// 在这里定义圆环识别、弹道解算等函数，再到主循环的动手区调用。



/*---------算法函数编写区域结束----------*/



/*---------禁止修改：主函数入口和相机初始化----------*/
int main()
{
    // ======== 1. 调参数：画面清楚 =========
    cv::VideoCapture camera(cameraIndex, cv::CAP_V4L2);
    if (!camera.isOpened())
    {
        std::cerr << "相机打开失败，请检查 cameraIndex。\n";
        return 1;
    }
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    camera.set(cv::CAP_PROP_FPS, fps);
    /*---------禁止修改区域结束----------*/



    /*---------禁止修改：应用全局相机参数----------*/
    // 调参只改上面的全局变量，这里负责把参数写入相机。
    if (!camera.set(cv::CAP_PROP_BUFFERSIZE, cameraBufferSize))
        std::cerr << "相机不支持设置缓冲帧数，需核对实际图像延迟。\n";
    camera.set(cv::CAP_PROP_BRIGHTNESS, cameraBrightness);
    camera.set(cv::CAP_PROP_CONTRAST, cameraContrast);
    camera.set(cv::CAP_PROP_SATURATION, cameraSaturation);
    camera.set(cv::CAP_PROP_HUE, cameraHue);
    // 先设置曝光模式，再设置曝光值，避免在自动模式下写入手动参数。
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, cameraAutoExposure);
    camera.set(cv::CAP_PROP_EXPOSURE, cameraExposure);
    camera.set(cv::CAP_PROP_SHARPNESS, cameraSharpness);
    camera.set(cv::CAP_PROP_GAMMA, cameraGamma);
    camera.set(cv::CAP_PROP_BACKLIGHT, cameraBacklight);
    // 先选择白平衡模式；开启自动模式时，驱动可能忽略下面的色温值。
    camera.set(cv::CAP_PROP_AUTO_WB, cameraAutoWhiteBalance);
    camera.set(cv::CAP_PROP_WB_TEMPERATURE, cameraWhiteBalance);
    /*---------禁止修改区域结束----------*/



    /*---------禁止修改：串口初始化、取图和反馈读取----------*/
    // 使用上方的全局串口参数。
    if (useSerial && !uart.Open(serialPort, serialBaudRate))
    {
        std::cerr << "串口打开失败，本次只显示图像。\n";
        useSerial = false;
    }

    cv::Mat image;
    int exitCode = 0;
    std::thread sender;
    std::thread receiver;
    if (useSerial)
    {
        receiver = std::thread(receiveLoop);
        sender = std::thread(sendLoop);
    }

    const auto controllerStart = std::chrono::steady_clock::now();
    DashBoard dashboard;
    g_lastControlAt = controllerStart;
    while (running.load())
    {
        // ===== 2. 取图：image 就是这一帧的 BGR 图像 =====
        camera >> image;
        if (image.empty())
        {
            std::cerr << "没有读到图像。\n";
            exitCode = 1;
            break;
        }
        const auto frameReceivedAt = std::chrono::steady_clock::now();

        // ===== 3. 复制接收结果：只在复制时加锁，算法放在锁外 =====
        TRoMaC::GimbalFeedback feedback{};
        bool feedbackReceived;
        bool feedbackValid;
        {
            std::lock_guard<std::mutex> lock(receiveMutex);
            feedback = receiveData;
            feedbackReceived = hasFeedback;
            // 超过 250 ms 没更新，就不再把旧角度当成有效反馈。
            feedbackValid = feedbackReceived &&
                std::chrono::steady_clock::now() - receivedAt < std::chrono::milliseconds(250);
        }
        // feedbackValid 为 true 才能使用 feedback，角度单位为度。

        // 每帧先清零控制量，推杆保持位由开火控制逻辑单独设置。
        sendData = TRoMaC::VisionData{};
        /*---------禁止修改区域结束----------*/

        /*---------需要编写：图像识别、控制计算和发送数据----------*/
        // 输入：image 是当前图像；feedback 是角度反馈，仅 feedbackValid 为 true 时可用。
        // 输出：修改 sendData 的字段；不用自己调用串口发送函数。
        // 控制量由 SuperStructure 状态机根据输出模式填写。
        sendData.Gimbal_Ctrl = 1;          // 2：增量控制；1：绝对角度控制。
        sendData.pitch_delta_angle = 0.0F;
        sendData.yaw_delta_angle = 0.0F;
        // 没有有效目标时保持当前反馈角，不能把绝对角度指令清成 0。
        sendData.pitch_angle = feedback.pitch_angle;
        sendData.yaw_angle = feedback.yaw_angle;
        // 当前固定协议：左右摩擦轮目标速度、蜂鸣器和推杆标志。
        sendData.left_speed = 480;
        sendData.right_speed = 480;
        sendData.buzzer = 0;
        sendData.Push_flag = 1;

        // 从这里往下写自己的代码：

        // 找环
        cv::Mat mask;
        std::vector<cv::Rect> rings = detectColorRing(image, mask,false);
        //取目标点
        std::vector<TargetPx> targetsPx =
            [&]() {
                if (rings.empty()) {
                    return std::vector<TargetPx>{};
                }
                std::vector<TargetPx> pts;
                for (const auto& box : rings) {
                    TargetPx t;
                    t.center = getTargetCenter(box);
                    t.radius = (box.width + box.height) / 4.0F;
                    t.box = box;
                    t.valid = true;
                    pts.push_back(t);
                }
                return pts;
            }();

        bool targetProjectionValid = false;
        std::vector<frc::Translation3d> targetsInBase= 
            [&]() {
                if (targetsPx.empty()) {
                    return std::vector<frc::Translation3d>{};
                }
                std::vector<frc::Translation3d> pts;
                for (const auto& p : targetsPx) {
                    const auto targetInWorld = projection.pixelToWorld(
                        cv::Point2d{p.center.x, p.center.y},
                        GimbalAngles{
                            units::degree_t{feedback.pitch_angle},
                            units::degree_t{feedback.yaw_angle}});
                    if (targetInWorld) {
                        pts.push_back(*targetInWorld);
                    }
                }
                targetProjectionValid = !pts.empty();
                return pts;
            }();

        static bool previousHasYellow = false;
        hasYellow = hasYellowRing(image);
        const bool yellowAppeared = hasYellow && !previousHasYellow;
        previousHasYellow = hasYellow;
        if (yellowAppeared)
        {
            g_modelAccepted = false;
            estimator.resetCenter();
            modelLockDebouncer.reset(false);
            targetMatcher.reset();
            omegaEstimator.reset();
            omega = kTARGET_OMEGAS[1];
            std::cout << "检测到黄色目标，模型解锁并清空拟合历史。\n";
        }

        const int controllerNow = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                frameReceivedAt - controllerStart).count());
        const double controlDt = clampValue(
            std::chrono::duration<double>(frameReceivedAt - g_lastControlAt).count(),
            0.001, 0.1);
        g_lastControlAt = frameReceivedAt;

        

        if (targetProjectionValid && !hasYellow)
        {
            estimator.calcCenter(targetsInBase[0], controllerNow);
            if (std::isfinite(estimator.getRes()) &&
                estimator.getRes() > kMAX_RES_TOLERANCE)
            {
                estimator.resetCenter();
                modelLockDebouncer.reset(false);
                std::cout << "拟合残差过大，已清空历史样本。\n";
            }

            const bool modelFitReady =
                estimator.hasEnoughSamples() &&
                std::isfinite(estimator.getRes()) &&
                estimator.getRes() < kMODEL_RES_TOLERANCE;

            if (!g_modelAccepted &&
                modelLockDebouncer.update(modelFitReady))
            {
                center = estimator.getExpCenter();
                targetMatcher.initialize(center, targetsInBase[0], controllerNow);
                omegaEstimator.reset();
                omega = kTARGET_OMEGAS[1];
                g_modelAccepted = true;
                std::cout << "模型首次接受，后续持续使用拟合圆心。\n";
            }
            else if (g_modelAccepted)
            {
                center = estimator.getExpCenter();
                targetMatcher.updateCenter(center);
            }
        }
        
        targetMatcher.setRecordHitTargets(recordHitTargets);
        std::optional<frc::Translation3d> currentTarget;
        if (g_modelAccepted && !hasYellow)
        {
            currentTarget = targetMatcher.select(targetsInBase, controllerNow, omega);
        }
        if (g_modelAccepted && currentTarget &&
            omegaEstimator.updateOmega(controllerNow, *currentTarget, center))
        {
            omega = omegaEstimator.getOmega();
        }

        frc::Translation3d expTarget = center;
        if (currentTarget)
        {
            if (useOmega)
            {
                const auto correctedTarget = AimSolver::buildPointByR(
                    *currentTarget, center, kTRAJECTORY_RADIUS);
                expTarget = AimSolver::getPhaseAtTime(
                    0.4, correctedTarget, center, omega, true);
            }
            else
            {
                expTarget = *currentTarget;
            }
        }
  
        AxisData angles = getShootAngles(expTarget);
        AxisData AnglesWithLead = withLead(angles, -2.5, -3.7);




        sendData.Gimbal_Ctrl = 1;
        const bool aimTargetValid = currentTarget.has_value() && g_modelAccepted &&
            std::isfinite(AnglesWithLead.yaw) && std::isfinite(AnglesWithLead.pitch);
        const double desiredYaw = aimTargetValid ? AnglesWithLead.yaw : 0.0;
        const double desiredPitch = aimTargetValid ? AnglesWithLead.pitch : 0.0;

        if (feedbackValid)
        {
            if (!g_commandInitialized)
            {
                g_commandYaw = feedback.yaw_angle;
                g_commandPitch = feedback.pitch_angle;
                g_commandInitialized = true;
            }

            const double maxCommandStep = kCommandSlewRateDegPerSec * controlDt;
            g_commandYaw += clampValue(
                desiredYaw - g_commandYaw, -maxCommandStep, maxCommandStep);
            g_commandPitch += clampValue(
                desiredPitch - g_commandPitch, -maxCommandStep, maxCommandStep);

            sendData.yaw_angle = static_cast<float>(g_commandYaw);
            sendData.pitch_angle = static_cast<float>(g_commandPitch);
        }

        const int shotsBeforeFire = g_shotsFired;
        sendData.Push_flag = updateFireControl(
            aimTargetValid,
            feedbackValid,
            AnglesWithLead,
            feedback,
            frameReceivedAt);
        if (g_shotsFired > shotsBeforeFire)
        {
            targetMatcher.markFiredAt(controllerNow);
        }


        //**************************************画图和字段log******************************************** */
        const DashboardFrame dashboardFrame{
            .target = targetsPx,
            .targetMarkers = g_modelAccepted ? targetMatcher.markers(kTRAJECTORY_RADIUS)
                                           : std::vector<TargetMarker>{},
            .aimPoint = {kAimU, kAimV},
            .aimTolerance = kAimTolPx,
            .aimedStable = g_aimedStable,
            .targetProjectionValid = targetProjectionValid,
            .feedbackReceived = feedbackReceived,
            .feedbackValid = feedbackValid,
            .fireEnabled = kEnableFire,
            .modelAccepted = g_modelAccepted,
            .shotsFired = g_shotsFired,
            .ringCount = rings.size(),
            .angles = angles,
            .leadAngles = AnglesWithLead,
            .residual = estimator.getRes(),
            .center = center,
            .fittedCenter = std::isfinite(estimator.getRes())
                ? std::optional<frc::Translation3d>{estimator.getExpCenter()}
                : std::nullopt,
            .radius = kTRAJECTORY_RADIUS,
            .measuredOmega = omegaEstimator.getMeasuredOmega(),
            .matchedOmega = omega,
            .omegaReady = omegaEstimator.ready(),
            .predictedTarget = expTarget,
            .gimbalAngles = {units::degree_t{feedback.pitch_angle}, units::degree_t{feedback.yaw_angle}},
            .feedback = feedback,
            .command = sendData,
            .targetColor = kTargetColor,
            .receivedAt = frameReceivedAt};
        dashboard.updateLog(dashboardFrame);
        dashboard.draw(image, dashboardFrame, projection);
        dashboard.show(image, mask);
        if (!dashboard.processKeys(image, mask, kEnableFire, kTargetColor))
            break;
            
        /*---------图像识别和控制编写区域结束----------*/

        /*---------禁止修改：发布发送数据、退出清理----------*/
        // ===== 4. 发布完整结果，由独立发送线程负责串口发送 =====
        // 主线程只改 sendData；加锁复制后，发送线程才能看到这一整份结果。
        if (useSerial)
        {
            std::lock_guard<std::mutex> lock(sendMutex);
            latestSendData = sendData;
            sendUpdatedAt = frameReceivedAt;
            sendReady = true;
        }
    }

    // 先通知两个通信线程退出并等它们结束，再关闭串口。
    running.store(false);
    if (sender.joinable())
        sender.join();
    if (receiver.joinable())
        receiver.join();
    if (sendFailed.load())
        exitCode = 1;
    uart.Close();
    camera.release();
    cv::destroyAllWindows();
    return exitCode;
}
/*---------禁止修改区域结束----------*/
