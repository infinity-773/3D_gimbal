#pragma once

#include <cmath>

#include <gcem.hpp>
#include <units/angle.h>

namespace turret_kinematics {

class Rotation2d {
 public:
  constexpr Rotation2d() = default;
  constexpr Rotation2d(units::radian_t angle)
      : m_cos{gcem::cos(angle.value())}, m_sin{gcem::sin(angle.value())} {}
  constexpr Rotation2d(units::degree_t angle)
      : Rotation2d{units::radian_t{angle}} {}
  constexpr Rotation2d(double x, double y) {
    const double magnitude = gcem::hypot(x, y);
    if (magnitude > 1e-6) {
      m_cos = x / magnitude;
      m_sin = y / magnitude;
    }
  }

  constexpr Rotation2d operator+(const Rotation2d& other) const {
    return RotateBy(other);
  }
  constexpr Rotation2d operator-(const Rotation2d& other) const {
    return *this + -other;
  }
  constexpr Rotation2d operator-() const { return {m_cos, -m_sin}; }
  constexpr Rotation2d operator*(double scalar) const {
    return Rotation2d{units::radian_t{Radians().value() * scalar}};
  }
  constexpr Rotation2d operator/(double scalar) const {
    return *this * (1.0 / scalar);
  }
  constexpr bool operator==(const Rotation2d& other) const {
    return gcem::hypot(Cos() - other.Cos(), Sin() - other.Sin()) < 1e-9;
  }

  constexpr Rotation2d RotateBy(const Rotation2d& other) const {
    return {Cos() * other.Cos() - Sin() * other.Sin(),
            Cos() * other.Sin() + Sin() * other.Cos()};
  }
  constexpr Rotation2d RelativeTo(const Rotation2d& other) const {
    return RotateBy(-other);
  }
  constexpr units::radian_t Radians() const {
    return units::radian_t{gcem::atan2(m_sin, m_cos)};
  }
  constexpr units::degree_t Degrees() const { return Radians(); }
  constexpr double Cos() const { return m_cos; }
  constexpr double Sin() const { return m_sin; }
  constexpr double Tan() const { return Sin() / Cos(); }

 private:
  double m_cos = 1.0;
  double m_sin = 0.0;
};

}  // namespace turret_kinematics
