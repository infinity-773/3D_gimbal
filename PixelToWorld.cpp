#include "PixelToWorld.h"

#include <cmath>

namespace
{
double dot3(const cv::Vec3d& a, const cv::Vec3d& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm3(const cv::Vec3d& v)
{
    return std::sqrt(dot3(v, v));
}

cv::Point3d toPoint3d(const cv::Vec3d& v)
{
    return cv::Point3d(v[0], v[1], v[2]);
}
} // namespace

PixelToWorldConfig::PixelToWorldConfig()
    : fx(1300.0),
      fy(1300.0),
      cx(640.0),
      cy(360.0),
      T_camera_to_gimbal(
          1.0, 0.0, 0.0, 0.0,
          0.0, -1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0),
      gimbal_origin_in_world(0.0, 0.0, 0.0),
      target_plane_normal_world(0.0, 0.0, 1.0),
      target_plane_offset(2.0),
      use_plane_point(false),
      target_plane_point_world(0.0, 0.0, 2.0),
      yaw_offset_deg(0.0),
      pitch_offset_deg(0.0),
      yaw_sign(1.0),
      pitch_sign(1.0),
      angles_in_degrees(true),
      require_intersection_in_front(true)
{
}

BackProjectionResult::BackProjectionResult()
    : valid(false),
      world_point(0.0, 0.0, 0.0),
      camera_origin_world(0.0, 0.0, 0.0),
      ray_direction_world(0.0, 0.0, 0.0),
      plane_parameter(-1.0)
{
}

PixelToWorld::PixelToWorld(const PixelToWorldConfig& config)
    : config_(config)
{
}

void PixelToWorld::setConfig(const PixelToWorldConfig& config)
{
    config_ = config;
}

const PixelToWorldConfig& PixelToWorld::config() const
{
    return config_;
}

PixelToWorldConfig& PixelToWorld::config()
{
    return config_;
}

double PixelToWorld::angleToRad(double angle) const
{
    const double kPi = 3.14159265358979323846;
    return config_.angles_in_degrees ? angle * kPi / 180.0 : angle;
}

cv::Matx33d PixelToWorld::rotationX(double rad)
{
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return cv::Matx33d(1.0, 0.0, 0.0,
                       0.0,   c,  -s,
                       0.0,   s,   c);
}

cv::Matx33d PixelToWorld::rotationY(double rad)
{
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return cv::Matx33d( c, 0.0,   s,
                        0.0, 1.0, 0.0,
                       -s, 0.0,   c);
}

cv::Matx33d PixelToWorld::rotationZ(double rad)
{
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return cv::Matx33d( c,  -s, 0.0,
                        s,   c, 0.0,
                        0.0, 0.0, 1.0);
}

cv::Matx44d PixelToWorld::makeRigidTransform(const cv::Matx33d& R, const cv::Vec3d& t)
{
    return cv::Matx44d(
        R(0,0), R(0,1), R(0,2), t[0],
        R(1,0), R(1,1), R(1,2), t[1],
        R(2,0), R(2,1), R(2,2), t[2],
        0.0,    0.0,    0.0,    1.0);
}

cv::Matx44d PixelToWorld::invertRigidTransform(const cv::Matx44d& T)
{
    // T = [R t; 0 1]
    // T^{-1} = [R^T -R^T t; 0 1]
    const cv::Matx33d R(
        T(0,0), T(0,1), T(0,2),
        T(1,0), T(1,1), T(1,2),
        T(2,0), T(2,1), T(2,2));
    const cv::Vec3d t(T(0,3), T(1,3), T(2,3));

    const cv::Matx33d Rt = R.t();
    const cv::Vec3d tInv = -Rt * t;
    return makeRigidTransform(Rt, tInv);
}

cv::Vec3d PixelToWorld::transformPoint(const cv::Matx44d& T, const cv::Vec3d& p)
{
    const cv::Vec4d ph(p[0], p[1], p[2], 1.0);
    const cv::Vec4d r = T * ph;
    return cv::Vec3d(r[0], r[1], r[2]);
}

cv::Vec3d PixelToWorld::transformDirection(const cv::Matx44d& T, const cv::Vec3d& d)
{
    const cv::Vec4d dh(d[0], d[1], d[2], 0.0);
    const cv::Vec4d r = T * dh;
    return cv::Vec3d(r[0], r[1], r[2]);
}

cv::Matx33d PixelToWorld::gimbalToWorldRotation(double pitch, double yaw) const
{
    const double kPi = 3.14159265358979323846;

    const double yawRad =
        config_.yaw_sign *
        (angleToRad(yaw) + config_.yaw_offset_deg * kPi / 180.0);

    const double pitchRad =
        config_.pitch_sign *
        (angleToRad(pitch) + config_.pitch_offset_deg * kPi / 180.0);

    // Base axes are X forward, Y left, Z up: pitch about Y, then yaw about Z.
    return rotationZ(yawRad) * rotationY(pitchRad);
}

cv::Matx44d PixelToWorld::gimbalToWorldTransform(double pitch, double yaw) const
{
    const cv::Matx33d R_gw = gimbalToWorldRotation(pitch, yaw);
    return makeRigidTransform(R_gw, config_.gimbal_origin_in_world);
}

cv::Matx44d PixelToWorld::totalTransform(double pitch, double yaw) const
{
    // T_cw = T_gw * T_cg
    return gimbalToWorldTransform(pitch, yaw) * config_.T_camera_to_gimbal;
}

cv::Vec3d PixelToWorld::pixelRayCamera(double Px, double Py) const
{
    if (std::fabs(config_.fx) < 1e-12 || std::fabs(config_.fy) < 1e-12)
    {
        return cv::Vec3d(0.0, 0.0, 1.0);
    }

    return cv::Vec3d(
        (Px - config_.cx) / config_.fx,
        (Py - config_.cy) / config_.fy,
        1.0);
}

BackProjectionResult PixelToWorld::BackProjection(double Px,
                                                  double Py,
                                                  double pitch,
                                                  double yaw) const
{
    BackProjectionResult result;

    if (!std::isfinite(Px) || !std::isfinite(Py) ||
        !std::isfinite(pitch) || !std::isfinite(yaw))
    {
        return result;
    }

    // 1. Pixel -> Camera 射线
    const cv::Vec3d d_camera = pixelRayCamera(Px, Py);

    // 2. Camera -> World，使用齐次变换 T_cw
    const cv::Matx44d T_cw = totalTransform(pitch, yaw);

    // 方向向量：w = 0，只受旋转影响
    const cv::Vec3d d_world = transformDirection(T_cw, d_camera);

    // 相机光心：w = 1，受旋转和平移影响
    const cv::Vec3d camera_origin_world =
        transformPoint(T_cw, cv::Vec3d(0.0, 0.0, 0.0));

    // 3. 目标平面 n · X = d
    cv::Vec3d n = config_.target_plane_normal_world;
    const double n_norm = norm3(n);
    if (n_norm < 1e-12)
    {
        return result;
    }
    n /= n_norm;

    double plane_d = 0.0;
    if (config_.use_plane_point)
    {
        const cv::Vec3d p0(
            config_.target_plane_point_world.x,
            config_.target_plane_point_world.y,
            config_.target_plane_point_world.z);
        plane_d = dot3(n, p0);
    }
    else
    {
        // 原始平面 n_orig · X = d_orig，归一化后 d = d_orig / ||n_orig||
        plane_d = config_.target_plane_offset / n_norm;
    }

    const double denom = dot3(n, d_world);
    if (std::fabs(denom) < 1e-12)
    {
        return result;
    }

    const double t = (plane_d - dot3(n, camera_origin_world)) / denom;
    if (config_.require_intersection_in_front && t <= 0.0)
    {
        return result;
    }

    const cv::Vec3d world = camera_origin_world + t * d_world;
    if (!std::isfinite(world[0]) ||
        !std::isfinite(world[1]) ||
        !std::isfinite(world[2]))
    {
        return result;
    }

    result.valid = true;
    result.world_point = toPoint3d(world);
    result.camera_origin_world = toPoint3d(camera_origin_world);
    result.ray_direction_world = toPoint3d(d_world);
    result.plane_parameter = t;
    return result;
}

bool PixelToWorld::BackProjection(double Px,
                                  double Py,
                                  double pitch,
                                  double yaw,
                                  cv::Point3d& worldPoint) const
{
    const BackProjectionResult result = BackProjection(Px, Py, pitch, yaw);
    if (!result.valid)
    {
        return false;
    }
    worldPoint = result.world_point;
    return true;
}