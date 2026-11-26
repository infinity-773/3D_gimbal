#ifndef AIM_SOLVER_H
#define AIM_SOLVER_H

#include <optional>
#include <array>
#include <Constants.h>

namespace AimSolver {
    frc::Translation3d getPhaseAtTime(double dt , frc::Translation3d const& currTarget , frc::Translation3d const& center,double omega,bool Anti){
        const double dy = currTarget.Y().value() - center.Y().value();
        const double dz = currTarget.Z().value() - center.Z().value();
        for (std::size_t i = 0; i < kTARGET_OMEGAS.size(); ++i)
        {
            if (std::abs(omega - kTARGET_OMEGAS[i]) < 1e-9)
            {
                dt *= kLEAD_TIME_SCALES[i];
                break;
            }
        }
        const units::radian_t phi{
            (Anti ? -1.0 : 1.0) * omega * dt};

        const frc::Translation2d rotated =
            frc::Translation2d{dy, dz}.RotateBy(frc::Rotation2d{phi});

        return frc::Translation3d{
            currTarget.X(),
            center.Y() + rotated.X(),
            center.Z() + rotated.Y()
        };
    };

    frc::Translation3d buildPointByR(frc::Translation3d const& currTarget , frc::Translation3d const&  center3D,double r){
        //取出当前目标点在Y-Z平面上的坐标,角度
        const frc::Translation2d center = frc::Translation2d {center3D.Y(), center3D.Z()};
        const frc::Translation2d target = frc::Translation2d{currTarget.Y(), currTarget.Z()};
        frc::Translation2d targetInCenter = target - center;
        //限制norm，排除径向噪声
        targetInCenter = targetInCenter * (r / targetInCenter.Norm().value());

        return frc::Translation3d{
            currTarget.X(),
            center3D.Y() + targetInCenter.X(),
            center3D.Z() + targetInCenter.Y()
        };
    };


}
#endif // AIM_SOLVER_H