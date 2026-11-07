#pragma once

#include "vision_bridge.h"
#include <functional>

using ServoCallback = std::function<void(double, double)>;

class FireController {
public:
    FireController(double shot_interval, double pulse_duration);
    void SetEnabled(bool enabled);
    bool IsEnabled() const;
    int Update(double t, const TargetModel& model, bool servo_ready);

private:
    double shot_interval_;
    double pulse_duration_;
    double last_shot_;
    double pulse_until_;
    bool enabled_ = false;
};

void SetServoCallback(ServoCallback callback);
void SendToServo(double yaw, double pitch);
void ControlTick(double t_now, double dt_ctrl, TargetModel& model,
                 const frc::Transform3d& baseToCam);
void OnVisionFrame(const VisionFrame& vf, const CameraConfig& cam,
                   const frc::Transform3d& baseToCam,
                   TargetModel& model, std::vector<frc::Translation3d>& pts_base_accum,
                   std::vector<double>& times_accum, double& geom_residual_accum_time);