#include "control_loop.h"
#include "target_fitter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
ServoCallback servo_callback;
}

FireController::FireController(double shot_interval, double pulse_duration)
    : shot_interval_{std::max(shot_interval, 0.0)},
      pulse_duration_{std::clamp(pulse_duration, 0.0, shot_interval_)},
      last_shot_{-std::numeric_limits<double>::infinity()},
      pulse_until_{-std::numeric_limits<double>::infinity()} {}

void FireController::SetEnabled(bool enabled) { enabled_ = enabled; }
bool FireController::IsEnabled() const { return enabled_; }

int FireController::Update(double t, const TargetModel& model, bool servo_ready) {
    if (!enabled_ || !servo_ready || model.Lock() != LockState::LOCKED || !model.FireEnabled()) {
        pulse_until_ = -std::numeric_limits<double>::infinity();
        return 1;
    }
    if (t >= last_shot_ + shot_interval_) {
        last_shot_ = t;
        pulse_until_ = t + pulse_duration_;
    }
    return t < pulse_until_ ? 0 : 1;
}

void SetServoCallback(ServoCallback callback) { servo_callback = std::move(callback); }
void SendToServo(double yaw, double pitch) {
    if (servo_callback) servo_callback(yaw, pitch);
}

void ControlTick(double t_now, double dt_ctrl, TargetModel& model, const frc::Transform3d&) {
    model.Tick(t_now, dt_ctrl);
    if (model.Lock() != LockState::IDLE) {
        const auto aim = model.Predict(t_now);
        const double x = aim.X().value(), y = aim.Y().value(), z = aim.Z().value();
        SendToServo(std::atan2(y, x), std::atan2(z, std::hypot(x, y)));
    }
}

void OnVisionFrame(const VisionFrame& vf, const CameraConfig& cam,
                   const frc::Transform3d& baseToCam, TargetModel& model,
                   std::vector<frc::Translation3d>& pts_base_accum,
                   std::vector<double>& times_accum, double& geom_residual_accum_time) {
    if (!vf.detected) { model.OnObservation({vf.t, 0.0, false}); return; }
    const auto points_cam = PnPBlade(vf.pts_px, cam, baseToCam);
    if (!points_cam) { model.OnObservation({vf.t, 0.0, false}); return; }
    const auto points = ToBase(*points_cam, baseToCam);
    const double previous_time = model.state.last_obs_time;
    const auto obs = MakeObservation(vf, points, model.geom);
    model.OnObservation(obs);
    frc::Translation3d representative;
    for (const auto& point : points) representative = representative + point;
    representative = representative / 4.0;
    if (model.Lock() == LockState::IDLE) {
        if (pts_base_accum.size() != times_accum.size() ||
            (!times_accum.empty() && (vf.t <= times_accum.back() || vf.t - times_accum.front() > 5.0))) {
            pts_base_accum.clear();
            times_accum.clear();
        }
        pts_base_accum.push_back(representative);
        times_accum.push_back(vf.t);
        if (model.geom.valid && pts_base_accum.size() >= 4) {
            std::vector<double> phases;
            for (const auto& point : pts_base_accum) {
                const auto delta = point - model.geom.center;
                phases.push_back(std::atan2(delta.Dot(model.geom.v).value(), delta.Dot(model.geom.u).value()));
            }
            const auto fit = FitPhase(phases, times_accum, model.phase.OmegaAt(vf.t));
            if (fit && fit->rms_deg < 1.5 && fit->omega > 0.05 && fit->omega < 10.0) {
                const PhaseModel phase{fit->phi_ref, fit->t_ref, fit->omega, fit->direction};
                model.ApplyFit(model.geom, phase, TargetModel::WrapPi(phases.front() - phase.PhaseAt(times_accum.front())));
            }
        } else if (!model.geom.valid && pts_base_accum.size() >= 20) {
            const auto fit = FitAll(pts_base_accum, times_accum, model.phase.OmegaAt(vf.t));
            if (fit && fit->rms_deg < 1.5) model.ApplyFit(fit->geom, fit->phase, fit->phi_target_offset);
        }
        if (model.Lock() != LockState::LOCKED) {
            if (model.geom.valid && pts_base_accum.size() >= 5) {
                pts_base_accum.erase(pts_base_accum.begin());
                times_accum.erase(times_accum.begin());
            }
            return;
        }
        model.state.last_obs_time = vf.t;
        pts_base_accum.clear();
        times_accum.clear();
        geom_residual_accum_time = 0.0;
        return;
    }
    if (model.Lock() != LockState::LOCKED) return;
    const auto delta = representative - model.geom.center;
    const double residual = std::abs(delta.Norm().value() - model.geom.radius);
    if (residual <= 0.02) { geom_residual_accum_time = 0.0; return; }
    const double elapsed = vf.t - previous_time;
    if (elapsed > 0.0 && elapsed <= 0.1) geom_residual_accum_time += elapsed;
    if (geom_residual_accum_time > 1.0) {
        const auto refined = RefineGeometryStep(model.geom, points, 0.05, 0.05, 0.002, 0.002);
        if (refined.valid) model.ApplyGeomRefine(refined.center, refined.radius);
        geom_residual_accum_time = 0.0;
    }
}