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
#include "control_loop.h"
#include "RingDetector.h"
#include "turret_kinematics/Pose3d.h"
#include <algorithm>
#include <array>
#include <numbers>
#include <string>
#include <vector>
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
int fps = 30;

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
// 注意：缓冲不是图像控制项，不能用 --list-ctrls 查询范围。
int cameraBufferSize = 1;


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
int cameraExposure = 17;

// 亮度
// 范围：-64～64，步长 1，默认 0。
// 作用：调整画面整体明暗。
int cameraBrightness = 17;

// 对比度
// 范围：0～100，步长 1，默认 35。
// 作用：调整明暗区域的差别。
int cameraContrast = 50;

// 伽马
// 范围：100～500，步长 1，默认 300。
// 作用：调整中间亮度层次。
int cameraGamma = 266;

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
constexpr double kBigPlaneDistanceM = 2.0; // 机关平面基座 X 坐标，米
constexpr double kCenterInPlaneYM = -0.08; // 圆心基座 Y 坐标，米
constexpr double kCenterInPlaneZM = 0.28; // 圆心基座 Z 坐标，米
constexpr double kTargetRadiusM = 0.28; // 叶片轨迹半径，米
constexpr double kPlaneNormalX = 1.0; // 法向 X 分量，无量纲
constexpr double kPlaneNormalY = 0.0; // 法向 Y 分量，无量纲
constexpr double kPlaneNormalZ = 0.0; // 法向 Z 分量，无量纲
constexpr double kBladeSizeM = 0.135; // 四切点圆的物理直径，米
constexpr std::array<double, 9> kCamMatrixK{1300, 0, 639.5, 0, 1300, 359.5, 0, 0, 1}; // 相机内参，像素
constexpr std::array<double, 5> kCamDistD{0, 0, 0, 0, 0}; // 畸变系数，无量纲
constexpr std::array<double, 3> kCamToBaseTranslationXYZ{0.00934, 0, 0.1102}; // 零位相机平移，米
constexpr std::array<double, 3> kCamToBaseRotationRPY{-std::numbers::pi / 2, 0, -std::numbers::pi / 2}; // 光学系到云台零位旋转，弧度
constexpr double kOmegaHint = std::numbers::pi / 5; // 初始角速度，弧度/秒
constexpr double kDefaultTOffsetS = 0.05; // 总延迟初值，秒
constexpr bool kEnableServo = true; // 是否发送伺服瞄准指令
constexpr double kYawLimitDeg = 80.0; // yaw 指令范围，度
constexpr double kPitchLimitDeg = 30.0; // pitch 指令范围，度
constexpr double kShotIntervalS = 0.8; // 两次击发的最短间隔，秒
constexpr double kPushPulseS = 0.2; // 推杆触发脉冲时长，秒
constexpr unsigned int kFlywheelSpeed = 800; // 摩擦轮命令速度，协议单位
static bool kShowDebug = true;
static int g_frameIndex = 0;
static std::chrono::steady_clock::time_point g_lastControlAt;
static RingDetector ringDetector;
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

        // 下位机回传固定15字节反馈，float顺序为 pitch、yaw，单位为度。
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

static void DrawTargetOverlay(cv::Mat& image, const TargetModel& model, double t,
                              const frc::Transform3d& baseToCam, const CameraConfig& camera) {
    if (image.empty() || !model.geom.valid) return;
    constexpr int segments = 128;
    const cv::Scalar pathColor = model.geom.source == GeomSource::KNOWN
        ? cv::Scalar(128, 128, 128) : cv::Scalar(255, 255, 0);
    std::array<std::optional<cv::Point2d>, segments> path;
    for (int index = 0; index < segments; ++index) {
        const double phi = 2.0 * std::numbers::pi * index / segments;
        path[index] = ProjectToImage(model.geom.Predict(phi), baseToCam, camera);
    }
    for (int index = 0; index < segments; ++index) {
        const auto& first = path[index];
        const auto& second = path[(index + 1) % segments];
        if (!first || !second) continue;
        cv::Point from{cvRound(first->x), cvRound(first->y)};
        cv::Point to{cvRound(second->x), cvRound(second->y)};
        if (cv::clipLine(image.size(), from, to))
            cv::line(image, from, to, pathColor, 2, cv::LINE_AA);
    }
    const cv::Point center{image.cols / 2, image.rows / 2};
    cv::drawMarker(image, center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 18, 2, cv::LINE_AA);
    if (model.Lock() == LockState::IDLE) return;
    const auto target = ProjectToImage(model.Predict(t), baseToCam, camera);
    if (!target) return;
    const cv::Point point{cvRound(target->x), cvRound(target->y)};
    cv::line(image, center, point, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::circle(image, point, 7, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
}

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

    TargetModel model;
    model.LoadKnownGeometry({kBigPlaneDistanceM, kCenterInPlaneYM, kCenterInPlaneZM},
                            kTargetRadiusM, {kPlaneNormalX, kPlaneNormalY, kPlaneNormalZ});
    model.phase.omega = kOmegaHint;
    model.state.t_offset = kDefaultTOffsetS;
    const CameraConfig cameraConfig{cv::Matx33d{kCamMatrixK.data()},
                                    cv::Vec<double, 5>{kCamDistD.data()}, kBladeSizeM};
    std::vector<frc::Translation3d> accumulated;
    std::vector<double> times;
    double residualTime = 0.0;
    bool commandReady = false;
    TRoMaC::GimbalFeedback previousFeedback{};
    std::chrono::steady_clock::time_point previousFeedbackAt{};
    bool hasPreviousFeedback = false;
    FireController fireControl{kShotIntervalS, kPushPulseS};
    SetServoCallback([&](double yaw, double pitch) {
        constexpr double degrees = 180.0 / std::numbers::pi;
        sendData.Gimbal_Ctrl = TRoMaC::kAbsoluteCommand;
        sendData.yaw_angle = static_cast<float>(std::clamp(yaw * degrees, -kYawLimitDeg, kYawLimitDeg));
        sendData.pitch_angle = static_cast<float>(std::clamp(-pitch * degrees, -kPitchLimitDeg, kPitchLimitDeg));
        commandReady = true;
    });



    /*---------禁止修改：串口初始化、取图和反馈读取----------*/
    // 使用上方的全局串口参数。
    if (useSerial && !uart.Open(serialPort, serialBaudRate))
    {
        std::cerr << "串口打开失败，本次只显示图像。\n";
        useSerial = false;
    }

    cv::namedWindow("Camera", cv::WINDOW_NORMAL);
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
        {
            std::lock_guard<std::mutex> lock(receiveMutex);
            if (hasFeedback && receivedAt <= frameReceivedAt &&
                frameReceivedAt - receivedAt < std::chrono::milliseconds(250)) {
                feedback = receiveData;
                feedbackValid = true;
            } else if (hasPreviousFeedback && previousFeedbackAt <= frameReceivedAt &&
                       frameReceivedAt - previousFeedbackAt < std::chrono::milliseconds(250)) {
                feedback = previousFeedback;
                feedbackValid = true;
            } else {
                feedbackValid = false;
            }
            if (hasFeedback) {
                previousFeedback = receiveData;
                previousFeedbackAt = receivedAt;
                hasPreviousFeedback = true;
            }
        }
        sendData.Gimbal_Ctrl = TRoMaC::kIncrementalCommand;
        sendData.Push_flag = 1;
        constexpr double radians = std::numbers::pi / 180.0;
        const frc::Rotation3d rotation{0.0, feedback.pitch_angle * radians, feedback.yaw_angle * radians};
        const frc::Transform3d mount{
            {kCamToBaseTranslationXYZ[0], kCamToBaseTranslationXYZ[1], kCamToBaseTranslationXYZ[2]},
            {kCamToBaseRotationRPY[0], kCamToBaseRotationRPY[1], kCamToBaseRotationRPY[2]}};
        const auto cameraPose = turret_kinematics::Pose3d{{}, rotation}.TransformBy(mount);
        const frc::Transform3d baseToCam{cameraPose.Translation(), cameraPose.Rotation()};
        cv::Mat mask;
        const auto boxes = ringDetector.Process(image, mask);
        VisionFrame frame;
        frame.t = std::chrono::duration<double>(frameReceivedAt - controllerStart).count();
        frame.detected = feedbackValid && !boxes.empty();
        if (frame.detected) {
            const auto box = *std::max_element(boxes.begin(), boxes.end(),
                [](const auto& left, const auto& right) { return left.area() < right.area(); });
            frame.bbox = box;
            const float x = box.x + box.width * 0.5F, y = box.y + box.height * 0.5F;
            const float radius = std::min(box.width, box.height) * 0.5F;
            frame.pts_px = {{x, y - radius}, {x + radius, y}, {x, y + radius}, {x - radius, y}};
        }
        const auto previousLock = model.Lock();
        OnVisionFrame(frame, cameraConfig, baseToCam, model, accumulated, times, residualTime);
        if (previousLock == LockState::IDLE && model.Lock() == LockState::LOCKED)
            model.state.t_offset = kDefaultTOffsetS;
        commandReady = false;
        const auto controlTime = std::chrono::steady_clock::now();
        const double controlT = std::chrono::duration<double>(controlTime - controllerStart).count();
        const double dt = std::chrono::duration<double>(controlTime - g_lastControlAt).count();
        g_lastControlAt = controlTime;
        ControlTick(controlT, dt, model, baseToCam);
        if (!kEnableServo || !feedbackValid || !commandReady) {
            sendData.Gimbal_Ctrl = TRoMaC::kIncrementalCommand;
            sendData.yaw_delta_angle = 0.0F;
            sendData.pitch_delta_angle = 0.0F;
        }
        if (feedbackValid) DrawTargetOverlay(image, model, controlT, baseToCam, cameraConfig);
        cv::putText(image, feedbackValid ? "feedback valid" : "feedback unavailable",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        std::string servoStatus;
        if (!useSerial) servoStatus = "SERVO: serial unavailable";
        else if (!feedbackValid) servoStatus = "SERVO: waiting for frame-time feedback";
        else if (!kEnableServo) servoStatus = "SERVO: disabled";
        else if (model.Lock() == LockState::IDLE)
            servoStatus = cv::format("SERVO: fitting trajectory, samples=%zu", accumulated.size());
        else if (!commandReady) servoStatus = "SERVO: no command";
        else servoStatus = cv::format("SERVO: tracking yaw=%.1f pitch=%.1f",
                                      sendData.yaw_angle, sendData.pitch_angle);
        cv::putText(image, servoStatus, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX,
                    0.6, cv::Scalar(0, 255, 255), 2);
        if (kShowDebug) cv::imshow("Mask", mask);
        cv::imshow("Camera", image);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) break;
        if (key == 's' || key == 'S') {
            const auto index = std::to_string(g_frameIndex++);
            cv::imwrite("cap_" + index + ".png", image);
            cv::imwrite("cap_" + index + "_mask.png", mask);
        }
        if (key == 'd' || key == 'D') {
            kShowDebug = !kShowDebug;
            if (!kShowDebug) cv::destroyWindow("Mask");
        }
        if (key == 'f' || key == 'F') {
            fireControl.SetEnabled(!fireControl.IsEnabled());
            std::cout << "开火开关: " << (fireControl.IsEnabled() ? "ON" : "OFF") << '\n';
        }
        if (key == 'c' || key == 'C') ringDetector.CycleColorMode();
        if (fireControl.IsEnabled()) {
            sendData.left_speed = kFlywheelSpeed;
            sendData.right_speed = kFlywheelSpeed;
        }
        sendData.Push_flag = static_cast<std::uint8_t>(
            fireControl.Update(controlT, model, kEnableServo && feedbackValid && commandReady && useSerial));
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
