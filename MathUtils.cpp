#include "MathUtils.h"

#include <cmath>
#include <stdexcept>

double clampValue(double value, double low, double high)
{
    if (!std::isfinite(value) || !std::isfinite(low) || !std::isfinite(high) || low > high)
    {
        throw std::invalid_argument("clampValue requires finite values and low <= high");
    }

    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

double pixelToAngle(double pixels, double focalLengthPx)
{
    if (!std::isfinite(pixels) || !std::isfinite(focalLengthPx) || focalLengthPx <= 0.0)
    {
        throw std::invalid_argument("pixelToAngle requires finite pixels and positive focal length");
    }

    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    return std::atan(pixels / focalLengthPx) * kRadiansToDegrees;
}
