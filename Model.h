#ifndef MODEL_H
#define MODEL_H
#include "gcem.hpp"
#include "turret_kinematics/TurretKinematics.h"

#define PI 3.14159265358979323846

namespace frc {
using Translation3d = turret_kinematics::Translation3d;
using Rotation3d = turret_kinematics::Rotation3d;
using Transform3d = turret_kinematics::Transform3d;
}




class Model {
public:
    Model(double distance = 2.0, double radius = 0.275): 
        center{distance, 0.0, 0.0}, r{radius}, omega{PI/5} {}

    frc::Translation3d getPhaseAtTime(double dt , frc::Translation3d const& currTarget){
        double phi = omega * dt;
        double dz = r * gcem::sin(phi);
        double dy = r * gcem::cos(phi);
        return frc::Translation3d{
            center.X().value(),
            center.Y().value() + dy,
            center.Z().value() + dz
        };
    };

    void updateModel(frc::Translation3d const& center, double r, double omega);
    void updatetime(double t);
    void updateCenter(frc::Translation3d const& center);
    void updateOmega(double omega);
    void updateRadius(double r);



private:
    frc::Translation3d center;
    double omega;
    double r;
    double time = 0;


    const frc::Translation3d camInBase{0.00934 , 0 , 0.1102};
    const frc::Translation3d shooterInBase{0.05139 , 0 , 0.0622};
};

#endif // MODEL_H