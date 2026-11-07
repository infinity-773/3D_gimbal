#ifndef RING_DETECTOR_H_
#define RING_DETECTOR_H_

#include <opencv2/core.hpp>

#include <vector>

enum class TargetColor
{
    Red,
    Blue,
    Any
};

class RingDetector
{
public:
    RingDetector();

    std::vector<cv::Rect> Process(cv::Mat& frame, cv::Mat& maskOut);
    void CycleColorMode();
    const char* GetColorModeName() const;

private:
    cv::Mat kernel_;
    TargetColor colorMode_ = TargetColor::Any;

    void BuildColorMask(const cv::Mat& hsv, cv::Mat& mask) const;
};

#endif // RING_DETECTOR_H_
