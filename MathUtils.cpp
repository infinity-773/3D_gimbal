#include "MathUtils.h"
#include "Constants.h"

#include <cmath>
#include <stdexcept>

AxisData getShootAngles(const turret_kinematics::Translation3d& targetInBase)
{
    const units::meter_t x = targetInBase.X();
    const units::meter_t y = targetInBase.Y();
    const units::meter_t z = targetInBase.Z();

    const double yawDeg = std::atan2(y.value(), x.value()) * radToDeg;
    const double pitchDeg = -std::atan2(
        (z - kSHOOTER_IN_BASE.Z()).value(), std::hypot(x.value(), y.value())) * radToDeg;

    return AxisData{yawDeg, pitchDeg};
}

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
    return std::atan(pixels / focalLengthPx) * radToDeg;
}
