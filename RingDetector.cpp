#include "RingDetector.h"

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace
{
const cv::Scalar kRedRangesLow[2]{
    cv::Scalar(0, 110, 137), cv::Scalar(11, 255, 255)};
const cv::Scalar kRedRangesHigh[2]{
    cv::Scalar(168, 110, 137), cv::Scalar(179, 255, 255)};
const cv::Scalar kBlueLow{100, 120, 80};
const cv::Scalar kBlueHigh{130, 255, 255};
const cv::Scalar kBoxColor{255, 0, 255};
}

RingDetector::RingDetector()
    : kernel_{cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3))}
{
}

std::vector<cv::Rect> RingDetector::Process(cv::Mat& frame, cv::Mat& maskOut)
{
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    BuildColorMask(hsv, maskOut);
    cv::morphologyEx(maskOut, maskOut, cv::MORPH_CLOSE, kernel_);
    cv::morphologyEx(maskOut, maskOut, cv::MORPH_OPEN, kernel_);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(maskOut.clone(), contours, hierarchy,
                     cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Rect> boxes;
    for (int cnt = 0; cnt < static_cast<int>(contours.size()); ++cnt)
    {
        if (cv::contourArea(contours[cnt]) < 80 ||
            hierarchy[cnt][3] != -1 ||
            hierarchy[cnt][2] == -1)
        {
            continue;
        }

        int inContour = -1;
        double inArea = 0.0;
        for (int child = hierarchy[cnt][2]; child != -1;
             child = hierarchy[child][0])
        {
            const double area = cv::contourArea(contours[child]);
            if (area >= 80 && area > inArea)
            {
                inArea = area;
                inContour = child;
            }
        }
        if (inArea < 80 || inContour < 0)
        {
            continue;
        }

        const double outPerimeter = cv::arcLength(contours[cnt], true);
        const double outArea = cv::contourArea(contours[cnt]);
        const double inPerimeter = cv::arcLength(contours[inContour], true);
        if (outPerimeter <= 1e-6 || inPerimeter <= 1e-6)
        {
            continue;
        }
        const double outCircularity =
            4.0 * CV_PI * outArea / (outPerimeter * outPerimeter);
        const double inCircularity =
            4.0 * CV_PI * inArea / (inPerimeter * inPerimeter);
        if (outCircularity < 0.7 || outCircularity > 1.3 ||
            inCircularity < 0.7 || inCircularity > 1.3 ||
            cv::matchShapes(contours[cnt], contours[inContour],
                            cv::CONTOURS_MATCH_I1, 0) > 0.3)
        {
            continue;
        }

        cv::Point2f outCenter;
        cv::Point2f inCenter;
        float outRadius = 0.0F;
        float inRadius = 0.0F;
        cv::minEnclosingCircle(contours[cnt], outCenter, outRadius);
        cv::minEnclosingCircle(contours[inContour], inCenter, inRadius);
        if (outRadius <= 1e-6F)
        {
            continue;
        }

        const double dx = outCenter.x - inCenter.x;
        const double dy = outCenter.y - inCenter.y;
        const double offset = std::sqrt(dx * dx + dy * dy) / outRadius;
        const double radiusRatio = static_cast<double>(inRadius) / outRadius;
        if (offset > 0.4 || radiusRatio < 0.3 || radiusRatio > 0.9)
        {
            continue;
        }

        const cv::Rect box = cv::boundingRect(contours[cnt]);
        const double aspectRatio =
            static_cast<double>(box.width) / box.height;
        if (aspectRatio < 0.8 || aspectRatio > 1.2)
        {
            continue;
        }

        boxes.push_back(box);
        cv::rectangle(frame, box, kBoxColor, 2);
    }

    return boxes;
}

void RingDetector::CycleColorMode()
{
    switch (colorMode_)
    {
    case TargetColor::Red:
        colorMode_ = TargetColor::Blue;
        break;
    case TargetColor::Blue:
        colorMode_ = TargetColor::Any;
        break;
    case TargetColor::Any:
        colorMode_ = TargetColor::Red;
        break;
    }
}

const char* RingDetector::GetColorModeName() const
{
    switch (colorMode_)
    {
    case TargetColor::Red: return "red";
    case TargetColor::Blue: return "blue";
    case TargetColor::Any: return "any";
    }
    return "unknown";
}

void RingDetector::BuildColorMask(const cv::Mat& hsv, cv::Mat& mask) const
{
    mask = cv::Mat::zeros(hsv.size(), CV_8U);
    if (colorMode_ == TargetColor::Red || colorMode_ == TargetColor::Any)
    {
        cv::Mat red1;
        cv::Mat red2;
        cv::inRange(hsv, kRedRangesLow[0], kRedRangesLow[1], red1);
        cv::inRange(hsv, kRedRangesHigh[0], kRedRangesHigh[1], red2);
        cv::bitwise_or(red1, red2, mask);
    }
    if (colorMode_ == TargetColor::Blue || colorMode_ == TargetColor::Any)
    {
        cv::Mat blue;
        cv::inRange(hsv, kBlueLow, kBlueHigh, blue);
        cv::bitwise_or(mask, blue, mask);
    }
}
