#ifndef RING_DETECTOR_H_
#define RING_DETECTOR_H_

#include <opencv2/core.hpp>

#include <vector>
#include <Constants.h>

enum class TargetColor
{
    Red,
    Blue,
    Any
};

// 识别到的目标
struct TargetPx
{
    cv::Point2f center;          // 框中心
    cv::Rect box;                 // 选中的框
    float radius = 0.0F;         // 框长边的一半
    double score = 0.0;          // 框的面积 调试用
    bool valid = false;          // 这帧有没有找到
};

static TargetColor kTargetColor = TargetColor::Any;   // 

static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

namespace
{
using namespace cv;

// 按当前颜色生成二值图
void buildColorMask(const Mat& hsv, Mat& mask ,bool isYellow)
{
    mask = Mat::zeros(hsv.size(), CV_8U);
    if (isYellow)
    {
        inRange(hsv, yellow_low, yellow_high, mask);
        return;
    }

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

std::vector<Rect> detectColorRing(const Mat& frame, Mat& maskOut, bool isYellow)
{
    std::vector<Rect> targetBoxs;

    Mat hsv, mask0;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;

    // BGR to HSV
    cvtColor(frame, hsv, COLOR_BGR2HSV);

    // 按当前颜色取二值图
    buildColorMask(hsv, mask0, isYellow);

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


bool hasYellowRing(const cv::Mat& frame){

    cv::Mat mask;
    std::vector<cv::Rect> boxes = detectColorRing(frame, mask, true);
    return !boxes.empty();
}

}

#endif // RING_DETECTOR_H_
