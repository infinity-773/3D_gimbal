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



struct ModelData {
    frc::Translation3d center;
    double r;
    double omega;
};

class Model {
public:
    Model(double distance = 2.0, double radius = 0.275): 
        data_{ {distance, 0.0, 0.0}, radius, PI/5} {}

    frc::Translation3d getPhaseAtTime(double time);

    void updateModel(ModelData const& data);

private:
    ModelData data_;
    double time;

};

#endif // MODEL_H