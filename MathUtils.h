#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include "Model.h"

#include <cmath>
#include <numbers>

double clampValue(double value, double low, double high);
double pixelToAngle(double pixels, double focalLengthPx);

struct AxisData{
    double yaw ;
    double pitch ;
};

inline AxisData getShootAngles(const frc::Translation3d& targetInBase)
{
    const double x = targetInBase.X().value();
    const double y = targetInBase.Y().value();
    const double z = targetInBase.Z().value();

        constexpr double radToDeg = 180.0 / std::numbers::pi;

    const double yawDeg = std::atan2(y, x) * radToDeg;
    const double pitchDeg = -std::atan2(z, std::hypot(x, y)) * radToDeg;

    return AxisData{yawDeg, pitchDeg};
}


inline AxisData withLead(const AxisData& data , double yawlead , double pitchlead){
    return AxisData{data.yaw - yawlead, data.pitch - pitchlead};
}

#endif // MATH_UTILS_H_
