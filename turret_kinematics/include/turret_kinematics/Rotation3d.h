#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <units/angle.h>

#include "turret_kinematics/Quaternion.h"
#include "turret_kinematics/Translation3d.h"

namespace turret_kinematics {

class Rotation3d {
 public:
  constexpr Rotation3d() = default;
  explicit Rotation3d(const Quaternion& quaternion)
      : m_quaternion{quaternion.Normalize()} {}
    Rotation3d(units::radian_t roll, units::radian_t pitch,
         units::radian_t yaw)
      : m_quaternion{FromRollPitchYaw(roll.value(), pitch.value(),
                      yaw.value())} {}
  Rotation3d(double rollRadians, double pitchRadians, double yawRadians)
      : m_quaternion{FromRollPitchYaw(rollRadians, pitchRadians, yawRadians)} {}
    Rotation3d(const std::array<double, 3>& axis, units::radian_t angle)
      : m_quaternion{FromAxisAngle(axis, angle.value())} {}
  Rotation3d(const Translation3d& axis, double angleRadians)
      : m_quaternion{FromAxisAngle(
        {axis.X().value(), axis.Y().value(), axis.Z().value()},
        angleRadians)} {}

  static Rotation3d FromDegrees(double rollDegrees, double pitchDegrees,
                                double yawDegrees) {
    constexpr double radiansPerDegree = 3.14159265358979323846 / 180.0;
    return {units::radian_t{rollDegrees * radiansPerDegree},
            units::radian_t{pitchDegrees * radiansPerDegree},
            units::radian_t{yawDegrees * radiansPerDegree}};
  }
  static Rotation3d FromDegrees(units::degree_t roll, units::degree_t pitch,
                                units::degree_t yaw) {
    return {units::radian_t{roll}, units::radian_t{pitch}, units::radian_t{yaw}};
  }

  Rotation3d operator+(const Rotation3d& other) const { return RotateBy(other); }
  Rotation3d operator-(const Rotation3d& other) const { return *this + -other; }
  Rotation3d operator-() const { return Rotation3d{m_quaternion.Inverse()}; }
  Rotation3d operator*(double scalar) const {
    return Rotation3d{Axis(),
                      units::radian_t{scalar * Angle().value()}};
  }
  Rotation3d operator/(double scalar) const { return *this * (1.0 / scalar); }
  bool operator==(const Rotation3d& other) const {
    return m_quaternion == other.m_quaternion;
  }

  Rotation3d RotateBy(const Rotation3d& other) const {
    return Rotation3d{other.m_quaternion * m_quaternion};
  }
  Rotation3d RelativeTo(const Rotation3d& other) const {
    return Rotation3d{other.m_quaternion.Inverse() * m_quaternion};
  }
  const Quaternion& GetQuaternion() const { return m_quaternion; }
  units::radian_t X() const {
    const auto& q = m_quaternion;
    return units::radian_t{
        std::atan2(2.0 * (q.W() * q.X() + q.Y() * q.Z()),
                   1.0 - 2.0 * (q.X() * q.X() + q.Y() * q.Y()))};
  }
  units::radian_t Y() const {
    const auto& q = m_quaternion;
    return units::radian_t{
        std::asin(std::clamp(2.0 * (q.W() * q.Y() - q.Z() * q.X()), -1.0,
                             1.0))};
  }
  units::radian_t Z() const {
    const auto& q = m_quaternion;
    return units::radian_t{
        std::atan2(2.0 * (q.W() * q.Z() + q.X() * q.Y()),
                   1.0 - 2.0 * (q.Y() * q.Y() + q.Z() * q.Z()))};
  }
  std::array<double, 3> Axis() const {
    const double norm = std::sqrt(m_quaternion.X() * m_quaternion.X() +
                                  m_quaternion.Y() * m_quaternion.Y() +
                                  m_quaternion.Z() * m_quaternion.Z());
    if (norm <= 1e-12) {
      return {0.0, 0.0, 0.0};
    }
    return {m_quaternion.X() / norm, m_quaternion.Y() / norm,
            m_quaternion.Z() / norm};
  }
  units::radian_t Angle() const {
    const double norm = std::sqrt(m_quaternion.X() * m_quaternion.X() +
                                  m_quaternion.Y() * m_quaternion.Y() +
                                  m_quaternion.Z() * m_quaternion.Z());
    return units::radian_t{2.0 * std::atan2(norm, m_quaternion.W())};
  }
  std::array<std::array<double, 3>, 3> ToMatrix() const {
    const double w = m_quaternion.W();
    const double x = m_quaternion.X();
    const double y = m_quaternion.Y();
    const double z = m_quaternion.Z();
    return {{{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),
              2.0 * (x * z + w * y)},
             {2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z),
              2.0 * (y * z - w * x)},
             {2.0 * (x * z - w * y), 2.0 * (y * z + w * x),
              1.0 - 2.0 * (x * x + y * y)}}};
  }

 private:
  static Quaternion FromAxisAngle(const std::array<double, 3>& axis,
                                  double radians) {
    const double norm = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] +
                                  axis[2] * axis[2]);
    if (norm <= 1e-12) {
      return {};
    }
    const double scale = std::sin(radians * 0.5) / norm;
        return {std::cos(radians * 0.5), axis[0] * scale, axis[1] * scale,
          axis[2] * scale};
  }
  static Quaternion FromRollPitchYaw(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
        return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
  }

  Quaternion m_quaternion;
};

inline Translation3d Translation3d::RotateBy(
    const Rotation3d& rotation) const {
  const Quaternion unit = rotation.GetQuaternion().Normalize();
   const Quaternion result = unit * Quaternion{0.0, X().value(), Y().value(), Z().value()} * unit.Inverse();
  return {result.X(), result.Y(), result.Z()};
}

inline Translation3d Translation3d::RotateAround(
    const Translation3d& point, const Rotation3d& rotation) const {
  return (*this - point).RotateBy(rotation) + point;
}

}  // namespace turret_kinematics