#pragma once

#include "target_model.h"

#include <optional>
#include <vector>

struct PhaseFit {
    double phi_ref = 0.0, t_ref = 0.0, omega = 0.0;
    int direction = 1;
    double rms_deg = 0.0;
};

struct FitResult {
    TargetGeometry geom;
    PhaseModel phase;
    double phi_target_offset = 0.0;
    double rms_deg = 0.0;
};

struct GeomRefine {
    frc::Translation3d center;
    double radius = 0.0;
    bool valid = false;
};

std::optional<PhaseFit> FitPhase(const std::vector<double>& phi_obs,
                                 const std::vector<double>& times, double omega_hint);
std::optional<FitResult> FitAll(const std::vector<frc::Translation3d>& pts_base,
                                const std::vector<double>& times,
                                double omega_hint);
std::optional<FitResult> FitWithPlane(const std::vector<frc::Translation3d>& pts_base,
                                      const std::vector<double>& times,
                                      double omega_hint, const TargetGeometry& prior);
GeomRefine RefineGeometryStep(const TargetGeometry& cur,
                              const std::vector<frc::Translation3d>& pts_base,
                              double k_center, double k_radius,
                              double max_center_shift, double max_radius_shift);