#pragma once

#include <cmath>

namespace turret_kinematics {

class Quaternion {
 public:
  constexpr Quaternion() = default;
  constexpr Quaternion(double w, double x, double y, double z)
      : m_w{w}, m_x{x}, m_y{y}, m_z{z} {}

  constexpr double W() const { return m_w; }
  constexpr double X() const { return m_x; }
  constexpr double Y() const { return m_y; }
  constexpr double Z() const { return m_z; }
  constexpr double Dot(const Quaternion& other) const {
    return m_w * other.m_w + m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;
  }
  double Norm() const { return std::sqrt(Dot(*this)); }
  constexpr Quaternion Conjugate() const { return {m_w, -m_x, -m_y, -m_z}; }
  Quaternion Inverse() const {
    const double normSquared = Dot(*this);
    if (normSquared <= 1e-24) {
      return {};
    }
    return {m_w / normSquared, -m_x / normSquared, -m_y / normSquared,
            -m_z / normSquared};
  }
  Quaternion Normalize() const {
    const double norm = Norm();
    if (norm <= 1e-12) {
      return {};
    }
    return {m_w / norm, m_x / norm, m_y / norm, m_z / norm};
  }

  constexpr Quaternion operator+(const Quaternion& other) const {
    return {m_w + other.m_w, m_x + other.m_x, m_y + other.m_y,
            m_z + other.m_z};
  }
  constexpr Quaternion operator-(const Quaternion& other) const {
    return {m_w - other.m_w, m_x - other.m_x, m_y - other.m_y,
            m_z - other.m_z};
  }
  constexpr Quaternion operator*(double scalar) const {
    return {m_w * scalar, m_x * scalar, m_y * scalar, m_z * scalar};
  }
  constexpr Quaternion operator/(double scalar) const {
    return *this * (1.0 / scalar);
  }
  constexpr Quaternion operator*(const Quaternion& other) const {
    return {m_w * other.m_w - m_x * other.m_x - m_y * other.m_y - m_z * other.m_z,
            m_w * other.m_x + m_x * other.m_w + m_y * other.m_z - m_z * other.m_y,
            m_w * other.m_y - m_x * other.m_z + m_y * other.m_w + m_z * other.m_x,
            m_w * other.m_z + m_x * other.m_y - m_y * other.m_x + m_z * other.m_w};
  }
  bool operator==(const Quaternion& other) const {
    return std::abs(Dot(other) - Norm() * other.Norm()) < 1e-9 &&
           std::abs(Norm() - other.Norm()) < 1e-9;
  }

 private:
  double m_w = 1.0;
  double m_x = 0.0;
  double m_y = 0.0;
  double m_z = 0.0;
};

}  // namespace turret_kinematics