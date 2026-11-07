#pragma once

#include "target_model.h"

#include <opencv2/core.hpp>
#include <optional>
#include <vector>

struct CameraConfig {
    cv::Matx33d K;
    cv::Vec<double, 5> D;
    double blade_size = 0.0;
};

struct VisionFrame {
    double t = 0.0;
    std::vector<cv::Point2f> pts_px;
    cv::Rect2f bbox;
    bool detected = false;
};

std::optional<std::vector<frc::Translation3d>> PnPBlade(
    const std::vector<cv::Point2f>& pts_px, const CameraConfig& cam,
    const frc::Transform3d& baseToCam);
frc::Translation3d CamToBase(const frc::Translation3d& p_cam,
                              const frc::Transform3d& baseToCam);
std::vector<frc::Translation3d> ToBase(const std::vector<frc::Translation3d>& pts_cam,
                                        const frc::Transform3d& baseToCam);
std::optional<cv::Point2d> ProjectToImage(const frc::Translation3d& point_base,
                                          const frc::Transform3d& baseToCam,
                                          const CameraConfig& cam);
Observation MakeObservation(const VisionFrame& vf,
                            const std::vector<frc::Translation3d>& pts_base,
                            const TargetGeometry& geom);