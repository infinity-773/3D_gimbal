#pragma once

#include <array>
#include <cmath>
#include <optional>

#include <gcem.hpp>
#include <units/area.h>
#include <units/length.h>

#include "turret_kinematics/Rotation2d.h"

namespace turret_kinematics {

class Translation2d {
 public:
  constexpr Translation2d() = default;
  constexpr Translation2d(double x, double y)
      : m_x{units::meter_t{x}}, m_y{units::meter_t{y}} {}
  constexpr Translation2d(units::meter_t x, units::meter_t y)
      : m_x{x}, m_y{y} {}
  constexpr Translation2d(units::meter_t distance, const Rotation2d& angle)
      : m_x{distance * angle.Cos()}, m_y{distance * angle.Sin()} {}

  constexpr units::meter_t Distance(const Translation2d& other) const {
    return units::meter_t{gcem::hypot((other.m_x - m_x).value(),
                                      (other.m_y - m_y).value())};
  }
  constexpr units::square_meter_t SquaredDistance(
      const Translation2d& other) const {
    return (other - *this).SquaredNorm();
  }
  constexpr units::meter_t X() const { return m_x; }
  constexpr units::meter_t Y() const { return m_y; }
  constexpr std::array<units::meter_t, 2> ToArray() const {
    return {m_x, m_y};
  }
  constexpr units::meter_t Norm() const {
    return units::meter_t{gcem::hypot(m_x.value(), m_y.value())};
  }
  constexpr units::square_meter_t SquaredNorm() const {
    return units::square_meter_t{m_x.value() * m_x.value() +
                                 m_y.value() * m_y.value()};
  }
  constexpr std::optional<Rotation2d> Angle() const {
    if (Norm().value() > 1e-6) {
      return Rotation2d{m_x.value(), m_y.value()};
    }
    return std::nullopt;
  }
  bool IsFinite() const {
    return std::isfinite(m_x.value()) && std::isfinite(m_y.value());
  }

  constexpr Translation2d RotateBy(const Rotation2d& rotation) const {
    return {m_x * rotation.Cos() - m_y * rotation.Sin(),
            m_x * rotation.Sin() + m_y * rotation.Cos()};
  }
  constexpr Translation2d RotateAround(const Translation2d& point,
                                       const Rotation2d& rotation) const {
    return {((m_x - point.m_x) * rotation.Cos() -
             (m_y - point.m_y) * rotation.Sin()) +
                point.m_x,
            ((m_x - point.m_x) * rotation.Sin() +
             (m_y - point.m_y) * rotation.Cos()) +
                point.m_y};
  }
  constexpr units::square_meter_t Dot(const Translation2d& other) const {
    return units::square_meter_t{m_x.value() * other.m_x.value() +
                                 m_y.value() * other.m_y.value()};
  }
  constexpr units::square_meter_t Cross(const Translation2d& other) const {
    return units::square_meter_t{m_x.value() * other.m_y.value() -
                                 m_y.value() * other.m_x.value()};
  }

  constexpr Translation2d operator+(const Translation2d& other) const {
    return {m_x + other.m_x, m_y + other.m_y};
  }
  constexpr Translation2d operator-(const Translation2d& other) const {
    return {m_x - other.m_x, m_y - other.m_y};
  }
  constexpr Translation2d operator-() const { return {-m_x, -m_y}; }
  constexpr Translation2d operator*(double scalar) const {
    return {m_x * scalar, m_y * scalar};
  }
  constexpr Translation2d operator/(double scalar) const {
    return *this * (1.0 / scalar);
  }
  constexpr bool operator==(const Translation2d& other) const {
    return gcem::abs((m_x - other.m_x).value()) < 1e-9 &&
           gcem::abs((m_y - other.m_y).value()) < 1e-9;
  }

 private:
  units::meter_t m_x{0.0};
  units::meter_t m_y{0.0};
};

constexpr Translation2d operator*(double scalar,
                                  const Translation2d& translation) {
  return translation * scalar;
}

}  // namespace turret_kinematics
