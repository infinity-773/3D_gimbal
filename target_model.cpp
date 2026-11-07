#include "target_model.h"

#include <cmath>
#include <numbers>

void TargetGeometry::BuildBasis() {
    const double length = normal.Norm().value();
    if (!std::isfinite(length) || length < 1e-12) {
        valid = false;
        u = {};
        v = {};
        return;
    }
    normal = normal / length;
    const frc::Translation3d ref = std::abs(normal.X().value()) < 0.9
        ? frc::Translation3d{1.0, 0.0, 0.0}
        : frc::Translation3d{0.0, 1.0, 0.0};
    const auto cross = normal.Cross(ref);
    u = frc::Translation3d{cross[0].value(), cross[1].value(), cross[2].value()};
    u = u / u.Norm().value();
    const auto second = normal.Cross(u);
    v = frc::Translation3d{second[0].value(), second[1].value(), second[2].value()};
}

frc::Translation3d TargetGeometry::Predict(double phi) const {
    return center + (u * std::cos(phi) + v * std::sin(phi)) * radius;
}

double PhaseModel::PhaseAt(double t) const {
    return phi_ref + direction * omega * (t - t_ref);
}

double PhaseModel::OmegaAt(double) const {
    return direction * omega;
}

double TargetModel::PhaseAt(double t) const { return phase.PhaseAt(t + state.t_offset); }
double TargetModel::OmegaAt(double t) const { return phase.OmegaAt(t + state.t_offset); }
double TargetModel::TargetPhaseAt(double t) const { return PhaseAt(t) + state.phi_target_offset; }
frc::Translation3d TargetModel::Predict(double t) const { return geom.Predict(TargetPhaseAt(t)); }

void TargetModel::LoadKnownGeometry(const frc::Translation3d& center, double radius,
                                    const frc::Translation3d& normal) {
    geom.center = center;
    geom.radius = radius;
    geom.normal = normal;
    geom.source = GeomSource::KNOWN;
    geom.valid = center.IsFinite() && std::isfinite(radius) && radius > 0.0;
    geom.BuildBasis();
}

void TargetModel::ApplyGeomRefine(const frc::Translation3d& center, double radius) {
    if (!geom.valid || !center.IsFinite() || !std::isfinite(radius) || radius <= 0.0) return;
    geom.center = center;
    geom.radius = radius;
    geom.source = GeomSource::REFINED;
}

void TargetModel::OnObservation(const Observation& obs) {
    if (!obs.detected || !geom.valid) return;
    state.last_obs_time = obs.t;
    const double e = WrapPi(obs.phi_obs - TargetPhaseAt(obs.t));
    const double e_deg = e * 180.0 / std::numbers::pi;
    if (std::abs(e_deg) > kSwitchDeg) {
        ++state.switch_count;
        if (state.switch_count >= kSwitchFrames && state.lock == LockState::LOCKED) {
            if (std::abs(OmegaAt(obs.t)) > 0.05) {
                state.phi_target_goal = WrapPi(obs.phi_obs - PhaseAt(obs.t));
            }
            state.lock = LockState::SWITCHING;
            state.fire_enabled = false;
            state.switch_count = 0;
        }
        return;
    }
    state.switch_count = 0;
    if (state.lock == LockState::LOCKED && std::abs(e_deg) > kPllDeadzoneDeg) {
        state.e_filt += 0.15 * (e - state.e_filt);
        const double omega_now = OmegaAt(obs.t);
        if (std::abs(omega_now) > 0.1) {
            state.t_offset += kPllGain * state.e_filt / omega_now;
        }
    }
}

void TargetModel::Tick(double t_now, double dt_ctrl) {
    switch (state.lock) {
    case LockState::LOCKED:
        state.fire_enabled = true;
        break;
    case LockState::SWITCHING: {
        const double d = WrapPi(state.phi_target_goal - state.phi_target_offset);
        const double alpha = 1.0 - std::exp(-dt_ctrl / kSmoothTau);
        state.phi_target_offset = WrapPi(state.phi_target_offset + d * alpha);
        if (std::abs(d) < std::numbers::pi / 180.0) {
            state.phi_target_offset = state.phi_target_goal;
            state.lock = LockState::LOCKED;
            state.fire_enabled = true;
            state.switch_count = 0;
        }
        break;
    }
    case LockState::IDLE:
        state.fire_enabled = false;
        break;
    }
    if (state.lock != LockState::IDLE && t_now - state.last_obs_time > kMissTimeout) {
        state.lock = LockState::IDLE;
        state.fire_enabled = false;
    }
}

void TargetModel::ApplyFit(const TargetGeometry& g, const PhaseModel& p,
                           double phi_target_offset) {
    geom = g;
    geom.BuildBasis();
    phase = p;
    state.phi_target_offset = WrapPi(phi_target_offset);
    state.phi_target_goal = state.phi_target_offset;
    state.t_offset = kDefaultTOffset;
    state.e_filt = 0.0;
    state.switch_count = 0;
    state.last_obs_time = 0.0;
    state.lock = LockState::LOCKED;
    state.fire_enabled = true;
}

bool TargetModel::FireEnabled() const { return state.fire_enabled; }
LockState TargetModel::Lock() const { return state.lock; }
double TargetModel::WrapPi(double a) { return std::remainder(a, 2.0 * std::numbers::pi); }