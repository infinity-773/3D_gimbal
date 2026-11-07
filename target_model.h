#pragma once

#include "turret_kinematics/Transform3d.h"

namespace frc {
using Translation3d = turret_kinematics::Translation3d;
using Rotation3d = turret_kinematics::Rotation3d;
using Transform3d = turret_kinematics::Transform3d;
}

struct Observation {
    double t = 0.0;
    double phi_obs = 0.0;
    bool detected = false;
};

enum class GeomSource { UNKNOWN, KNOWN, REFINED };

struct TargetGeometry {
    frc::Translation3d center;
    double radius = 0.0;
    frc::Translation3d normal;
    frc::Translation3d u, v;
    GeomSource source = GeomSource::UNKNOWN;
    bool valid = false;

    void BuildBasis();
    frc::Translation3d Predict(double phi) const;
};

struct PhaseModel {
    double phi_ref = 0.0;
    double t_ref = 0.0;
    double omega = 0.0;
    int direction = 1;

    double PhaseAt(double t) const;
    double OmegaAt(double t) const;
};

enum class LockState { IDLE, LOCKED, SWITCHING };

struct TargetState {
    double t_offset = 0.0;
    double phi_target_offset = 0.0;
    double phi_target_goal = 0.0;
    double e_filt = 0.0;
    double last_obs_time = 0.0;
    int switch_count = 0;
    bool fire_enabled = false;
    LockState lock = LockState::IDLE;
};

class TargetModel {
public:
    TargetGeometry geom;
    PhaseModel phase;
    TargetState state;

    static constexpr double kPllGain = 0.03;
    static constexpr double kPllDeadzoneDeg = 2.0;
    static constexpr double kSwitchDeg = 25.0;
    static constexpr int kSwitchFrames = 4;
    static constexpr double kMissTimeout = 1.0;
    static constexpr double kSmoothTau = 0.10;
    static constexpr double kDefaultTOffset = 0.05;

    double PhaseAt(double t) const;
    double OmegaAt(double t) const;
    double TargetPhaseAt(double t) const;
    frc::Translation3d Predict(double t) const;
    void LoadKnownGeometry(const frc::Translation3d& center, double radius,
                           const frc::Translation3d& normal);
    void ApplyGeomRefine(const frc::Translation3d& center, double radius);
    void OnObservation(const Observation& obs);
    void Tick(double t_now, double dt_ctrl);
    void ApplyFit(const TargetGeometry& g, const PhaseModel& p,
                  double phi_target_offset);
    bool FireEnabled() const;
    LockState Lock() const;
    static double WrapPi(double a);
};