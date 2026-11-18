#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <turret_kinematics/Rotation2d.h>
#include <turret_kinematics/Translation2d.h>
#include <turret_kinematics/Translation3d.h>

#include <cmath>
#include <numbers>

namespace frc {
using Pose2d = turret_kinematics::Pose2d;
using Translation2d = turret_kinematics::Translation2d;
using Translation3d = turret_kinematics::Translation3d;
using Transform2d = turret_kinematics::Transform2d;
using Rotation2d = turret_kinematics::Rotation2d;
using Rotation3d = turret_kinematics::Rotation3d;
using Transform3d = turret_kinematics::Transform3d;
}



const static double radToDeg = 180.0 / std::numbers::pi;

double clampValue(double value, double low, double high);
double pixelToAngle(double pixels, double focalLengthPx);

struct AxisData{
    double yaw ;
    double pitch ;
};

AxisData getShootAngles(const frc::Translation3d& targetInBase);


inline AxisData withLead(const AxisData& data , double yawlead , double pitchlead){
    return AxisData{data.yaw - yawlead, data.pitch - pitchlead};
}


inline frc::Translation2d ToYZ(
    const frc::Translation3d& position,
    const frc::Translation3d& center) {
        // 局部坐标系下x向右，y向上
        //X3d -> null, Y3d -> -x2d, Z3d -> y2d
        return {
            center.Y() - position.Y(),
            position.Z() - center.Z()
        };
}

inline frc::Translation3d ToXYZ(
    const frc::Translation2d& posInYZ,
    const frc::Translation3d& center) {
        return {
            center.X(),
            center.Y() - posInYZ.X(),
            center.Z() + posInYZ.Y()
        };
}
    // 顺时针旋转指定角度
inline frc::Rotation2d clockWiseRotate(units::degree_t degree){
    return frc::Rotation2d{-degree};
}

inline units::degree_t absDegree(units::degree_t degree){
    return units::degree_t{std::abs(degree.value())};
}

inline frc::Rotation2d absRotation(frc::Rotation2d rotation){
    return frc::Rotation2d{units::degree_t{std::abs(rotation.Degrees().value())}};
}

#endif // MATH_UTILS_H_
