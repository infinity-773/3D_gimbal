#ifndef PIXEL_TO_WORLD_H_
#define PIXEL_TO_WORLD_H_

#include <opencv2/core.hpp>

struct PixelToWorldConfig
{
    // 相机内参
    double fx;
    double fy;
    double cx;
    double cy;

    // 相机 C -> 云台 G 的齐次变换：P_G = T_camera_to_gimbal * P_C
    // T = [R t; 0 1]
    cv::Matx44d T_camera_to_gimbal;

    // G 原点在 World 中的固定位置
    cv::Vec3d gimbal_origin_in_world;

    // 目标平面：n · X = d
    cv::Vec3d target_plane_normal_world;
    double target_plane_offset;

    // 若 use_plane_point 为 true，则用 target_plane_point_world 定义平面：
    // n · (X - P0) = 0，即 n · X = n · P0
    bool use_plane_point;
    cv::Vec3d target_plane_point_world;

    // 角度零偏，单位 deg，会先加到输入角度上
    double yaw_offset_deg;
    double pitch_offset_deg;

    // 角度正方向修正
    double yaw_sign;
    double pitch_sign;

    // 输入 pitch/yaw 是否为角度制
    bool angles_in_degrees;

    // 是否要求射线与平面交点在相机前方 t > 0
    bool require_intersection_in_front;

    PixelToWorldConfig();
};

struct BackProjectionResult
{
    bool valid;
    cv::Point3d world_point;          // 与目标平面的交点
    cv::Point3d camera_origin_world;  // 相机光心在世界系的位置
    cv::Point3d ray_direction_world;  // 世界射线方向，未强制归一化
    double plane_parameter;           // P = O + t * D 中的 t

    BackProjectionResult();
};

class PixelToWorld
{
public:
    explicit PixelToWorld(const PixelToWorldConfig& config = PixelToWorldConfig());

    void setConfig(const PixelToWorldConfig& config);
    const PixelToWorldConfig& config() const;
    PixelToWorldConfig& config();

    // 主函数：像素 + pitch + yaw -> 世界坐标
    BackProjectionResult BackProjection(double Px,
                                        double Py,
                                        double pitch,
                                        double yaw) const;

    // 只要坐标、不关心射线信息时可用这个重载
    bool BackProjection(double Px,
                        double Py,
                        double pitch,
                        double yaw,
                        cv::Point3d& worldPoint) const;

    // 子函数，方便调试和复用
    cv::Vec3d pixelRayCamera(double Px, double Py) const;
    cv::Matx44d gimbalToWorldTransform(double pitch, double yaw) const;
    cv::Matx44d totalTransform(double pitch, double yaw) const; // C -> W

    // 齐次变换工具
    static cv::Matx44d makeRigidTransform(const cv::Matx33d& R, const cv::Vec3d& t);
    static cv::Matx44d invertRigidTransform(const cv::Matx44d& T);
    static cv::Vec3d transformPoint(const cv::Matx44d& T, const cv::Vec3d& p);
    static cv::Vec3d transformDirection(const cv::Matx44d& T, const cv::Vec3d& d);

    static cv::Matx33d rotationX(double rad);
    static cv::Matx33d rotationY(double rad);
    static cv::Matx33d rotationZ(double rad);

private:
    double angleToRad(double angle) const;
    cv::Matx33d gimbalToWorldRotation(double pitch, double yaw) const;

    PixelToWorldConfig config_;
};

#endif // PIXEL_TO_WORLD_H_