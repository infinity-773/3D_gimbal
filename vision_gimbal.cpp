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
#include "SuperStructure.h"
#include "Debouncer.h"
#include "MathUtils.h"
#include "CircularMotionPredictor.h"
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
// 在这里添加 HSV 阈值,计时起点等。

// 控制台
static bool kShowDebug = true;    // 调试窗口 按 d 关
static bool kEnableFire = false;  // 开火总开关 调参时一定关着

// 形态学核
static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

// 颜色 
enum class TargetColor
{
    Red,
    Blue,
    Any
};
static TargetColor kTargetColor = TargetColor::Any;   // 

// 红色 H 跨两头 分两段
static cv::Scalar red_ranges_low[2]  = {cv::Scalar(0, 110, 137),   cv::Scalar(11, 255, 255)};   // 
static cv::Scalar red_ranges_high[2] = {cv::Scalar(168, 110, 137), cv::Scalar(179, 255, 255)};   // 

// 蓝色一段
static cv::Scalar blue_low  = cv::Scalar(100, 120, 80);   // 
static cv::Scalar blue_high = cv::Scalar(130, 255, 255);   // 

// 识别到的目标
struct BladeTarget
{
    cv::Point2f center;          // 框中心
    cv::Rect box;                 // 选中的框
    float radius = 0.0F;         // 框长边的一半
    double score = 0.0;          // 框的面积 调试用
    bool valid = false;          // 这帧有没有找到
};

// 瞄准点 打完看偏哪边就往反方向挪
static double kAimU = width * 0.5 - 30;   // （需调参）
static double kAimV = height * 0.5;   // （需调参）

// 丢几帧还算有目标
static int    kHoldFrames = 8;

// 像素转角度
static double kFocalPx = 1300;     // （需调参）
static double kYawSign = 1.0;       // 图像往右 yaw 变大填 1 变小填 -1（需调参）
static double kPitchSign = 1.0;     // 图像往上 pitch 变大填 1 变小填 -1（需调参）
static double kRotationYawSign = 1.0;
static double kRotationPitchSign = 1.0;
static double kPredictionSeconds = 0.15;
static double kYawBias = 0.0;       // 固定的角度偏差（需调参）
static double kPitchBias = 0.0;     // 弹道下坠 远了要加（需调参）
static double kp = 0.6;          // PID 比例项
static double ki = 1.6; // PID 积分项
static double kd = 0.0; // PID 微分项
static double kIntegralLimit = 1.5; // 积分限幅，单位：度·秒
static double kControlDeadbandPx = 2; // 小于此像素误差时停止微调，避免识别抖动
static double kMaxStep = 4.0;       // PID 输出限幅，单位度
// 可选 Position（绝对角度）或 Incremental（角度增量）。
static ControlOutputMode kControlOutputMode = ControlOutputMode::Position;

// 开火
static double kAimTolPx = 13;         // 差多少像素算对准（需调参）
static int    kAimFrames = 3;           // 连着几帧对上才打（需调参）
static int    kShotIntervalMs = 800;    // 两发之间隔多久（需调参）
static int    kPushRetractMs = 200;     // 发射时推杆回缩时间，之后恢复伸出并保持
static double kIntegralEnableOffsetPx = 40.0; // 目标距中心不超过此值时启用积分
static double kFireRearmOffsetPx = 50.0; // 目标偏离中心超过此值时重新允许开火

// 跨帧要记住的
static int  g_aimedFrames = 0;
static int  g_shotsFired = 0;
static bool g_shotArmed = true;
static bool g_pushRetracting = false;
static int  g_frameIndex = 0;
static std::chrono::steady_clock::time_point g_lastControlAt;
static std::chrono::steady_clock::time_point g_lastShotAt =
    std::chrono::steady_clock::now() - std::chrono::seconds(10);   // 开机就能打
static std::chrono::steady_clock::time_point g_pushRetractStartedAt;


//初始化
Debouncer aimedDebouncer(10, DebounceEdge::Rising);
static SuperStructureConfig makeSuperStructureConfig()
{
    SuperStructureConfig config;
    config.aimU = kAimU;
    config.aimV = kAimV;
    config.focalLengthPx = kFocalPx;
    config.yawSign = kYawSign;
    config.pitchSign = kPitchSign;
    config.yawBias = kYawBias;
    config.pitchBias = kPitchBias;
    config.deadbandPx = kControlDeadbandPx;
    config.aimTolerancePx = kAimTolPx;
    config.integralEnableOffsetPx = kIntegralEnableOffsetPx;
    config.yawPid = PIDConfig(kp, ki, kd,
                              -kIntegralLimit, kIntegralLimit,
                              -kMaxStep, kMaxStep);
    config.pitchPid = config.yawPid;
    return config;
}
SuperStructure superStructure(makeSuperStructureConfig(), kControlOutputMode);
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

static uint8_t updateFireControl(bool aimed, std::chrono::steady_clock::time_point now)
{
    if (aimed)
    {
        ++g_aimedFrames;
    }
    else
    {
        g_aimedFrames = 0;
        g_shotArmed = true;
    }
    if (aimed)
    {
        const auto sinceLastShot =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - g_lastShotAt).count();
        if (kEnableFire && g_aimedFrames >= kAimFrames &&
            g_shotArmed && sinceLastShot >= kShotIntervalMs)
        {
            g_shotArmed = false;
            g_lastShotAt = now;
            ++g_shotsFired;
            g_pushRetracting = true;
            g_pushRetractStartedAt = now;
            std::cout << "[FIRE] 第 " << g_shotsFired << " 发\n";
        }
    }

    if (g_pushRetracting &&
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - g_pushRetractStartedAt).count() >= kPushRetractMs)
    {
        g_pushRetracting = false;
    }

    return g_pushRetracting ? 0 : 1;
}

// 圆环检测
namespace
{
using namespace cv;

// 按当前颜色生成二值图
void buildColorMask(const Mat& hsv, Mat& mask)
{
    mask = Mat::zeros(hsv.size(), CV_8U);

    if (kTargetColor == TargetColor::Red || kTargetColor == TargetColor::Any)
    {
        // 红的 H 跨 0 分两段
        Mat red1, red2;
        inRange(hsv, red_ranges_low[0], red_ranges_low[1], red1);
        inRange(hsv, red_ranges_high[0], red_ranges_high[1], red2);
        bitwise_or(red1, red2, mask);
    }

    if (kTargetColor == TargetColor::Blue || kTargetColor == TargetColor::Any)
    {
        Mat blue;
        inRange(hsv, blue_low, blue_high, blue);
        bitwise_or(mask, blue, mask);
    }
}

// 屏幕上显示颜色模式用
const char* targetColorName()
{
    switch (kTargetColor)
    {
        case TargetColor::Red:  return "red";
        case TargetColor::Blue: return "blue";
        case TargetColor::Any:
        default:                return "any";
    }
}

std::vector<Rect> detectColorRing(const Mat& frame, Mat& maskOut)
{
    std::vector<Rect> targetBoxs;

    Mat hsv, mask0;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;

    // BGR to HSV
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // 按当前颜色取二值图
    buildColorMask(hsv, mask0);

    // 闭开去噪
    morphologyEx(mask0, mask0, MORPH_CLOSE, kernel);
    morphologyEx(mask0, mask0, MORPH_OPEN, kernel);
    maskOut = mask0;   // 这里不开窗口 二值图拿回主循环显示

    // 两层轮廓
    findContours(mask0.clone(), contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    for (int cnt = 0; cnt < static_cast<int>(contours.size()); ++cnt)
    {
        // 太小的不要
        if (contourArea(contours[cnt]) < 80)
        {
            continue;
        }
        // 只要最外层的
        if (hierarchy[cnt][3] != -1)
        {
            continue;
        }
        // 没内孔的不是环
        if (hierarchy[cnt][2] == -1)
        {
            continue;
        }

        // 取最大子轮廓当内孔
        int next = hierarchy[cnt][2];
        double inAera = 0;
        int in_cnt = -1;
        while (next != -1)
        {
            if (contourArea(contours[next]) < 80)
            {
                next = hierarchy[next][0];
                continue;
            }
            if (contourArea(contours[next]) > inAera)
            {
                inAera = contourArea(contours[next]);
                in_cnt = next;
            }
            next = hierarchy[next][0];
        }
        if (inAera < 80 || in_cnt < 0)
        {
            continue;
        }

        // 外圈圆度
        double experi = arcLength(contours[cnt], true);
        double exArea = contourArea(contours[cnt]);
        if (experi <= 1e-6)
        {
            continue;
        }
        double circularity = 4.0 * CV_PI * exArea / (experi * experi);
        if (circularity < 0.7 || circularity > 1.3)
        {
            continue;
        }

        // 内孔也卡一遍
        double inPeri = arcLength(contours[in_cnt], true);
        if (inPeri <= 1e-6)
        {
            continue;
        }
        double inCircularity = 4.0 * CV_PI * inAera / (inPeri * inPeri);
        if (inCircularity < 0.7 || inCircularity > 1.3)
        {
            continue;
        }

        // 内外形状相似
        if (matchShapes(contours[cnt], contours[in_cnt], CONTOURS_MATCH_I1, 0) > 0.3)
        {
            continue;
        }

        // 同心
        Point2f oc, ic;
        float orad = 0, irad = 0;
        minEnclosingCircle(contours[cnt], oc, orad);
        minEnclosingCircle(contours[in_cnt], ic, irad);
        if (orad <= 1e-6)
        {
            continue;
        }
        double dx = oc.x - ic.x, dy = oc.y - ic.y;
        double offset = std::sqrt(dx * dx + dy * dy) / orad;
        if (offset > 0.4)
        {
            continue;
        }

        // 粗细
        double rRatio = static_cast<double>(irad) / orad;
        if (rRatio < 0.3 || rRatio > 0.9)
        {
            continue;
        }

        Rect Box = boundingRect(contours[cnt]);

        // 长宽比
        double rate = static_cast<double>(Box.width) / Box.height;
        if (rate > 1.2 || rate < 0.8)
        {
            continue;
        }

        targetBoxs.push_back(Box);
    }

    return targetBoxs;
}

cv::Point2f getTargetCenter(const cv::Rect& box)
{
    return cv::Point2f(box.x + box.width * 0.5F, box.y + box.height * 0.5F);
}

// 多个环直接选最大的
BladeTarget pickTarget(const std::vector<Rect>& boxes)
{
    BladeTarget best;
    double bestArea = 0.0;
    for (std::size_t i = 0; i < boxes.size(); ++i)
    {
        const Point2f c(boxes[i].x + boxes[i].width * 0.5F,
                        boxes[i].y + boxes[i].height * 0.5F);
        const double area = static_cast<double>(boxes[i].area());
        if (best.valid == false || area > bestArea)
        {
            best.center = c;
            best.box = boxes[i];
            if (boxes[i].width > boxes[i].height)
            {
                best.radius = boxes[i].width * 0.5F;
            }
            else
            {
                best.radius = boxes[i].height * 0.5F;
            }
            best.score = area;
            best.valid = true;
            bestArea = area;
        }
    }
    return best;
}
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
        // 输入：image 是当前图像；feedback 是角度反馈，仅 feedbackValid 为 true 时可用。
        // 输出：修改 sendData 的字段；不用自己调用串口发送函数。

        // 控制量由 SuperStructure 状态机根据输出模式填写。
        sendData.Gimbal_Ctrl = 1;          // 2：增量控制；1：绝对角度控制。
        sendData.pitch_delta_angle = 0.0F;
        sendData.yaw_delta_angle = 0.0F;
        sendData.pitch_angle = 0.0F;
        sendData.yaw_angle = 0.0F;
        // 当前固定协议：左右摩擦轮目标速度、蜂鸣器和推杆标志。
        sendData.left_speed = 660;
        sendData.right_speed = 660;
        sendData.buzzer = 0;
        sendData.Push_flag = 1;

        // 从这里往下写自己的代码：
        // 检测目标并交给状态机；PID 和位置/增量输出均由状态机负责。

        // 找环
        cv::Mat mask;
        std::vector<cv::Rect> rings = detectColorRing(image, mask);
        BladeTarget target = pickTarget(rings);

        const int controllerNow = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                frameReceivedAt - controllerStart).count());
        const double controlDt = clampValue(
            std::chrono::duration<double>(frameReceivedAt - g_lastControlAt).count(),
            0.001, 0.1);
        g_lastControlAt = frameReceivedAt;
        static CircularMotionPredictor motionPredictor(
            kFocalPx, image.cols * 0.5, image.rows * 0.5,
            kRotationYawSign, kRotationPitchSign);
        CircularPrediction prediction;
        if (target.valid && feedbackValid)
        {
            prediction = motionPredictor.update(
                target.center, feedback.yaw_angle, feedback.pitch_angle,
                std::chrono::duration<double>(frameReceivedAt - controllerStart).count(),
                kPredictionSeconds);
        }
        else
        {
            motionPredictor.reset();
        }
        const bool predictionUsable = target.valid && feedbackValid && prediction.valid &&
            prediction.predicted.x >= 0.0 && prediction.predicted.x < image.cols &&
            prediction.predicted.y >= 0.0 && prediction.predicted.y < image.rows;
        std::vector<cv::Rect> selectedBoxes;
        if (predictionUsable)
        {
            const int predictedX = cvRound(prediction.predicted.x);
            const int predictedY = cvRound(prediction.predicted.y);
            selectedBoxes.emplace_back(predictedX - 1, predictedY - 1, 2, 2);
        }
        superStructure.updateBoxes(BoxContainer(selectedBoxes));
        superStructure.periodic(feedback, controllerNow, controlDt, feedbackValid);

        const OutputData& controlOutput = superStructure.getoutput();
        if (!feedbackValid)
        {
            sendData.Gimbal_Ctrl = 2;
        }
        else if (controlOutput.mode == ControlOutputMode::Position)
        {
            sendData.Gimbal_Ctrl = 1;
            sendData.yaw_angle = static_cast<float>(controlOutput.axis.yaw);
            sendData.pitch_angle = static_cast<float>(controlOutput.axis.pitch);
        }
        else
        {
            sendData.Gimbal_Ctrl = 2;
            sendData.yaw_delta_angle = static_cast<float>(controlOutput.axis.yaw);
            sendData.pitch_delta_angle = static_cast<float>(controlOutput.axis.pitch);
        }

        // 对准了就开火
        const auto now = std::chrono::steady_clock::now();

        const bool targetMovedAway = predictionUsable &&
            std::hypot(controlOutput.targetOffsetPx.yaw,
                       controlOutput.targetOffsetPx.pitch) > kFireRearmOffsetPx;
        bool aimed;
        if (!predictionUsable || targetMovedAway)
        {
            aimedDebouncer.reset(false);
            aimed = false;
        }
        else
        {
            aimed = aimedDebouncer.update(superStructure.isAimed());
        }
        sendData.Push_flag = updateFireControl(aimed, now);

        // 画图
        cv::circle(image, cv::Point(static_cast<int>(kAimU), static_cast<int>(kAimV)),
                   6, cv::Scalar(0, 255, 0), 2);
        if (prediction.valid)
        {
            for (std::size_t index = 1; index < prediction.trajectory.size(); ++index)
            {
                cv::line(image, prediction.trajectory[index - 1], prediction.trajectory[index],
                         cv::Scalar(255, 255, 0), 2);
            }
            cv::drawMarker(image, prediction.center, cv::Scalar(255, 255, 0),
                           cv::MARKER_CROSS, 14, 2);
            cv::drawMarker(image, prediction.predicted, cv::Scalar(255, 0, 255),
                           cv::MARKER_CROSS, 20, 2);
            cv::putText(image,
                        cv::format("phase=%.1f deg speed=%.1f deg/s dt=%.3fs",
                                   prediction.phase * 180.0 / CV_PI,
                                   prediction.angularVelocity * 180.0 / CV_PI,
                                   kPredictionSeconds),
                        cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(255, 255, 0), 1);
        }
        else
        {
            cv::putText(image, "TRACK: waiting for valid trajectory",
                        cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(0, 165, 255), 1);
        }
        if (target.valid)
        {
            const cv::Point center(static_cast<int>(target.center.x),
                                   static_cast<int>(target.center.y));
            cv::Scalar color;
            if (aimed == true)
            {
                color = cv::Scalar(0, 255, 0);
            }
            else
            {
                color = cv::Scalar(0, 0, 255);
            }
            cv::circle(image, center, static_cast<int>(target.radius), color, 2);
            cv::line(image, center,
                     cv::Point(static_cast<int>(kAimU), static_cast<int>(kAimV)),
                     cv::Scalar(0, 255, 255), 1);
            cv::putText(image,
                        "du=" + std::to_string(
                            static_cast<int>(controlOutput.targetOffsetPx.yaw)) +
                            " dv=" + std::to_string(
                            static_cast<int>(controlOutput.targetOffsetPx.pitch)) +
                            " yaw=" + std::to_string(
                            static_cast<int>(controlOutput.targetAngleError.yaw)) +
                            " pitch=" + std::to_string(
                            static_cast<int>(controlOutput.targetAngleError.pitch)),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 255), 2);
        }
        else
        {
            cv::putText(image, "NO TARGET", cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
        }
        std::string fireText;
        cv::Scalar fireColor;
        if (kEnableFire == true)
        {
            fireText = " FIRE ON";
            fireColor = cv::Scalar(0, 0, 255);
        }
        else
        {
            fireText = " FIRE OFF";
            fireColor = cv::Scalar(0, 255, 0);
        }
        cv::putText(image, "shots=" + std::to_string(g_shotsFired) + fireText,
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, fireColor, 2);
        const std::string feedbackText = feedbackReceived
            ? cv::format("feedback yaw=%.2f pitch=%.2f%s",
                         static_cast<double>(feedback.yaw_angle),
                         static_cast<double>(feedback.pitch_angle),
                         feedbackValid ? "" : " (stale)")
            : "feedback not received";
        cv::putText(image, feedbackText, cv::Point(10, 120),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    feedbackValid ? cv::Scalar(255, 255, 255)
                                  : (feedbackReceived ? cv::Scalar(0, 165, 255)
                                                      : cv::Scalar(0, 0, 255)), 2);
        cv::putText(image, std::string("state=") + superStructure.getstateName(),
                    cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    cv::Scalar(255, 255, 255), 2);
        cv::putText(image, std::string("color: ") + targetColorName(),
                    cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 0), 1);

        if (kShowDebug == true)
        {
            // 瞄准点和接受圈
            const cv::Point aimPoint(static_cast<int>(kAimU), static_cast<int>(kAimV));
            cv::drawMarker(image, aimPoint, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 1);
            cv::circle(image, aimPoint, static_cast<int>(kAimTolPx),
                       cv::Scalar(0, 255, 0), 1);
            cv::putText(image, "AIM", aimPoint + cv::Point(14, -12),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);

            // 图例
            const int lx = image.cols - 216;
            const int ly = image.rows - 96;
            cv::rectangle(image, cv::Rect(lx - 10, ly - 10, 216, 96),
                          cv::Scalar(0, 0, 0), cv::FILLED);
            cv::circle(image, cv::Point(lx + 4, ly + 6), 4, cv::Scalar(0, 255, 0), -1);
            cv::putText(image, "AIM   impact point",
                        cv::Point(lx + 16, ly + 11), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                        cv::Scalar(0, 255, 0), 1);
            cv::circle(image, cv::Point(lx + 4, ly + 30), 4, cv::Scalar(0, 0, 255), -1);
            cv::putText(image, "TGT   detected ring",
                        cv::Point(lx + 16, ly + 35), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                        cv::Scalar(0, 0, 255), 1);
            cv::putText(image, "yellow line = error",
                        cv::Point(lx + 4, ly + 60), cv::FONT_HERSHEY_SIMPLEX, 0.38,
                        cv::Scalar(200, 200, 200), 1);

            // 帧率 框数
            static std::chrono::steady_clock::time_point prevFrameAt = frameReceivedAt;
            const double frameDt =
                std::chrono::duration<double>(frameReceivedAt - prevFrameAt).count();
            prevFrameAt = frameReceivedAt;
            static double fps = 0.0;
            if (frameDt > 1e-6)
            {
                if (fps <= 0.0)
                {
                    fps = 1.0 / frameDt;
                }
                else
                {
                    fps = 0.9 * fps + 0.1 / frameDt;
                }
            }
            cv::putText(image,
                        "fps=" + std::to_string(static_cast<int>(fps + 0.5)) +
                            "  rings=" + std::to_string(static_cast<int>(rings.size())) +
                            "  off=" + std::to_string(static_cast<int>(
                                controlOutput.targetOffsetPx.yaw)) + "," +
                            std::to_string(static_cast<int>(
                                controlOutput.targetOffsetPx.pitch)),
                        cv::Point(10, image.rows - 14), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 255), 1);

            cv::imshow("Mask", mask);
        }

        cv::imshow("Camera", image);
        int key = cv::waitKey(1); // 也负责刷新窗口，不能删掉。
        if (key == 'q' || key == 'Q' || key == 27)
        {
            break;
        }
        if (key == 's' || key == 'S')       // 按 s 存图
        {
            const std::string index = std::to_string(g_frameIndex);
            cv::imwrite("cap_" + index + ".png", image);
            cv::imwrite("cap_" + index + "_mask.png", mask);
            std::cout << "已保存 cap_" << index << ".png\n";
            ++g_frameIndex;
        }
        if (key == 'd' || key == 'D')       // 按 d 开关二值图窗口
        {
            kShowDebug = !kShowDebug;
            if (kShowDebug == false)
            {
                cv::destroyWindow("Mask");
            }
        }
        if (key == 'f' || key == 'F')       // 按 f 开关开火
        {
            kEnableFire = !kEnableFire;
            if (kEnableFire == true)
            {
                std::cout << "开火开关: ON" << '\n';
            }
            else
            {
                std::cout << "开火开关: OFF" << '\n';
            }
        }
        if (key == 'c' || key == 'C')       // 按 c 循环切 红 蓝 任意
        {
            if (kTargetColor == TargetColor::Red)
            {
                kTargetColor = TargetColor::Blue;
            }
            else if (kTargetColor == TargetColor::Blue)
            {
                kTargetColor = TargetColor::Any;
            }
            else
            {
                kTargetColor = TargetColor::Red;
            }
            std::cout << "目标颜色: " << targetColorName() << '\n';
        }
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
