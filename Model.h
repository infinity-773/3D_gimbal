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
        center{distance, 0.0, 0.0}, r{radius}, omega{PI/3} {}

    frc::Translation3d getPhaseAtTime(double dt , frc::Translation3d const& currTarget ,bool Anti){
        double phi = omega * dt;
        double dz = r * gcem::sin(phi);
        double dy = r * gcem::cos(phi);
        if(Anti){
            dz *= -1;
            dy *= -1;
        }
        return frc::Translation3d{
            currTarget.X().value(),
            currTarget.Y().value() + dy,
            currTarget.Z().value() + dz
        };
    };

    void updateModel(frc::Translation3d const& center, double r, double omega){
        this->omega = omega;
        this->r = r;
        this->center = center;
    };
    void updatetime(double t){
        this->time = t;
    };
    void updateCenter(frc::Translation3d const& center){
        this->center = center;
    };
    void updateOmega(double omega){
        this->omega = omega;
    };
    void updateRadius(double r){
        this->r = r;
    }   ;



private:
    frc::Translation3d center;
    double omega;
    double r;
    double time = 0;


    const frc::Translation3d camInBase{0.00934 , 0 , 0.1102};
    const frc::Translation3d shooterInBase{0.05139 , 0 , 0.0622};
};

#endif // MODEL_H