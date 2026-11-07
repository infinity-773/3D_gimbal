#pragma once

#include <array>
#include <cmath>

#include <units/area.h>
#include <units/length.h>

namespace turret_kinematics {

class Rotation3d;

class Translation3d {
 public:
  constexpr Translation3d() = default;
  constexpr Translation3d(double x, double y, double z)
      : m_x{units::meter_t{x}}, m_y{units::meter_t{y}}, m_z{units::meter_t{z}} {}
  constexpr Translation3d(units::meter_t x, units::meter_t y, units::meter_t z)
      : m_x{x}, m_y{y}, m_z{z} {}

  constexpr units::meter_t X() const { return m_x; }
  constexpr units::meter_t Y() const { return m_y; }
  constexpr units::meter_t Z() const { return m_z; }
  constexpr std::array<units::meter_t, 3> ToArray() const {
    return {m_x, m_y, m_z};
  }
  units::meter_t Distance(const Translation3d& other) const {
    return units::meter_t{std::sqrt(SquaredDistance(other).value())};
  }
  constexpr units::square_meter_t SquaredDistance(
      const Translation3d& other) const {
    return (other - *this).SquaredNorm();
  }
  constexpr units::square_meter_t Dot(const Translation3d& other) const {
    return units::square_meter_t{m_x.value() * other.m_x.value() +
                                 m_y.value() * other.m_y.value() +
                                 m_z.value() * other.m_z.value()};
  }
  constexpr std::array<units::square_meter_t, 3> Cross(
      const Translation3d& other) const {
    return {units::square_meter_t{m_y.value() * other.m_z.value() -
                                  m_z.value() * other.m_y.value()},
            units::square_meter_t{m_z.value() * other.m_x.value() -
                                  m_x.value() * other.m_z.value()},
            units::square_meter_t{m_x.value() * other.m_y.value() -
                                  m_y.value() * other.m_x.value()}};
  }
  constexpr units::square_meter_t SquaredNorm() const { return Dot(*this); }
  units::meter_t Norm() const {
    return units::meter_t{std::sqrt(SquaredNorm().value())};
  }
  bool IsFinite() const {
    return std::isfinite(m_x.value()) && std::isfinite(m_y.value()) &&
           std::isfinite(m_z.value());
  }

  Translation3d RotateBy(const Rotation3d& rotation) const;
  Translation3d RotateAround(const Translation3d& point,
                             const Rotation3d& rotation) const;

  constexpr Translation3d operator+(const Translation3d& other) const {
    return {m_x + other.m_x, m_y + other.m_y, m_z + other.m_z};
  }
  constexpr Translation3d operator-(const Translation3d& other) const {
    return {m_x - other.m_x, m_y - other.m_y, m_z - other.m_z};
  }
  constexpr Translation3d operator-() const { return {-m_x, -m_y, -m_z}; }
  constexpr Translation3d operator*(double scalar) const {
    return {m_x * scalar, m_y * scalar, m_z * scalar};
  }
  constexpr Translation3d operator/(double scalar) const {
    return *this * (1.0 / scalar);
  }
  bool operator==(const Translation3d& other) const {
      return std::abs((m_x - other.m_x).value()) < 1e-9 &&
        std::abs((m_y - other.m_y).value()) < 1e-9 &&
        std::abs((m_z - other.m_z).value()) < 1e-9;
  }

 private:
  units::meter_t m_x{0.0};
  units::meter_t m_y{0.0};
  units::meter_t m_z{0.0};
};

constexpr Translation3d operator*(double scalar, const Translation3d& value) {
  return value * scalar;
}

}  // namespace turret_kinematics