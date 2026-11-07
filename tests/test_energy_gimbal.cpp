#include "control_loop.h"
#include "target_fitter.h"

#include <opencv2/calib3d.hpp>
#include <cassert>
#include <cmath>
#include <numbers>
#include <vector>

int main() {
    TargetGeometry geometry;
    geometry.center = {2.0, 0.3, 0.8};
    geometry.radius = 0.7;
    geometry.normal = {1.0, 0.0, 0.0};
    geometry.valid = true;
    geometry.BuildBasis();
    assert(std::abs(geometry.u.Dot(geometry.v).value()) < 1e-12);
    assert(std::abs(geometry.u.Norm().value() - 1.0) < 1e-12);

    std::vector<frc::Translation3d> points;
    std::vector<double> times;
    for (int index = 0; index < 60; ++index) {
        const double time = 100.0 + index * 0.04;
        points.push_back(geometry.Predict(0.4 - 1.2 * (time - 100.0)));
        times.push_back(time);
    }
    const auto fit = FitAll(points, times, 0.0);
    assert(fit && fit->rms_deg < 1e-6);
    assert((fit->geom.center - geometry.center).Norm().value() < 1e-8);
    assert(std::abs(fit->geom.radius - geometry.radius) < 1e-8);
    assert(std::abs(fit->phase.omega - 1.2) < 1e-8);
    assert((fit->geom.Predict(fit->phase.PhaseAt(times.back())) - points.back()).Norm().value() < 1e-8);
    assert(!FitAll({}, {}, 0.0));

    const frc::Transform3d transform{{1.0, 2.0, 3.0}, frc::Rotation3d{0.0, 0.0, std::numbers::pi / 2.0}};
    const auto transformed = CamToBase({1.0, 0.0, 0.0}, transform);
    assert((transformed - frc::Translation3d{1.0, 3.0, 3.0}).Norm().value() < 1e-9);
    const CameraConfig projection_camera{cv::Matx33d{1000, 0, 640, 0, 1000, 360, 0, 0, 1}, {}, 0.12};
    const auto projected = ProjectToImage(CamToBase({0.2, -0.1, 2.0}, transform), transform, projection_camera);
    assert(projected && std::abs(projected->x - 740.0) < 1e-8 && std::abs(projected->y - 310.0) < 1e-8);
    assert(!ProjectToImage(CamToBase({0.0, 0.0, -1.0}, transform), transform, projection_camera));

    TargetModel model;
    model.ApplyFit(fit->geom, fit->phase, fit->phi_target_offset);
    model.state.last_obs_time = times.back();
    const double pll_offset = model.state.t_offset;
    const double time = times.back() + 0.01;
    model.OnObservation({time, model.TargetPhaseAt(time) + 0.2, true});
    assert(model.state.t_offset != pll_offset);
    assert(model.state.phi_target_offset == fit->phi_target_offset);
    const double before_switch = model.state.t_offset;
    for (int index = 0; index < 3; ++index)
        model.OnObservation({time, model.TargetPhaseAt(time) + 1.0, true});
    const double last_time = model.state.last_obs_time;
    model.OnObservation({time + 0.01, 0.0, false});
    assert(model.state.switch_count == 3 && model.state.last_obs_time == last_time);
    model.OnObservation({time + 0.02, model.TargetPhaseAt(time + 0.02) + 1.0, true});
    assert(model.Lock() == LockState::SWITCHING && !model.FireEnabled());
    const double initial_offset = model.state.phi_target_offset;
    model.Tick(time + 0.03, 0.01);
    assert(model.state.phi_target_offset != initial_offset && !model.FireEnabled());
    assert(model.state.t_offset == before_switch);
    for (int index = 0; index < 100; ++index) model.Tick(time + 0.03, 0.01);
    assert(model.Lock() == LockState::LOCKED && model.FireEnabled());
    FireController fire{0.8, 0.2};
    assert(!fire.IsEnabled() && fire.Update(0.0, model, true) == 1);
    fire.SetEnabled(true);
    assert(fire.Update(0.0, model, false) == 1);
    assert(fire.Update(0.0, model, true) == 0);
    assert(fire.Update(0.21, model, true) == 1);
    assert(fire.Update(0.79, model, true) == 1);
    assert(fire.Update(0.81, model, true) == 0);
    model.state.lock = LockState::SWITCHING;
    model.state.fire_enabled = false;
    assert(fire.Update(0.82, model, true) == 1);
    model.state.lock = LockState::LOCKED;
    model.state.fire_enabled = true;
    assert(fire.Update(0.83, model, true) == 1);
    fire.SetEnabled(false);
    assert(fire.Update(1.7, model, true) == 1);
    model.Tick(time + 1.03, 0.01);
    assert(model.Lock() == LockState::IDLE && !model.FireEnabled());
    fire.SetEnabled(true);
    assert(fire.Update(2.0, model, true) == 1);

    TargetModel integrated;
    integrated.LoadKnownGeometry({0.0, 0.0, 2.0}, 0.3, {0.0, 0.0, 1.0});
    std::vector<frc::Translation3d> accumulated;
    std::vector<double> accumulated_times;
    double residual_time = 0.0;
    const CameraConfig camera{cv::Matx33d{1000, 0, 640, 0, 1000, 360, 0, 0, 1}, {}, 0.12};
    const std::vector<cv::Point3f> object{{0, -0.06F, 0}, {0.06F, 0, 0}, {0, 0.06F, 0}, {-0.06F, 0, 0}};
    for (int index = 0; index < 20 && integrated.Lock() == LockState::IDLE; ++index) {
        const double time = 100.0 + index * 0.04;
        const auto point = integrated.geom.Predict(0.4 + 1.2 * (time - 100.0));
        std::vector<cv::Point2f> pixels;
        cv::projectPoints(object, cv::Vec3d{0.0, 0.0, 0.0},
                          cv::Vec3d{point.X().value(), point.Y().value(), point.Z().value()},
                          camera.K, camera.D, pixels);
        VisionFrame frame{time, pixels, {}, true};
        OnVisionFrame(frame, camera, {}, integrated, accumulated, accumulated_times, residual_time);
    }
    assert(integrated.Lock() == LockState::LOCKED);
    assert(integrated.state.last_obs_time > 100.0);
    assert(accumulated.empty() && accumulated_times.empty());
    integrated.Tick(integrated.state.last_obs_time + 0.01, 0.01);
    assert(integrated.Lock() == LockState::LOCKED);
}