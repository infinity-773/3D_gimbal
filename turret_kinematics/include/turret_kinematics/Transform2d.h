#pragma once

#include <units/length.h>

#include "turret_kinematics/Rotation2d.h"
#include "turret_kinematics/Translation2d.h"

namespace turret_kinematics {

class Pose2d;

class Transform2d {
 public:
  constexpr Transform2d() = default;
  constexpr Transform2d(Translation2d translation, Rotation2d rotation)
      : m_translation{translation}, m_rotation{rotation} {}
  constexpr Transform2d(units::meter_t x, units::meter_t y,
                        Rotation2d rotation)
      : m_translation{x, y}, m_rotation{rotation} {}
  Transform2d(const Pose2d& initial, const Pose2d& final);

  constexpr const Translation2d& Translation() const { return m_translation; }
  constexpr const Rotation2d& Rotation() const { return m_rotation; }
  constexpr units::meter_t X() const { return m_translation.X(); }
  constexpr units::meter_t Y() const { return m_translation.Y(); }

  constexpr Transform2d Inverse() const {
    const Rotation2d inverseRotation = -m_rotation;
    return {(-m_translation).RotateBy(inverseRotation), inverseRotation};
  }
  constexpr Transform2d operator+(const Transform2d& other) const {
    return {m_translation + other.m_translation.RotateBy(m_rotation),
            m_rotation + other.m_rotation};
  }
  constexpr Transform2d operator*(double scalar) const {
    return {m_translation * scalar, m_rotation * scalar};
  }
  constexpr bool operator==(const Transform2d& other) const {
    return m_translation == other.m_translation && m_rotation == other.m_rotation;
  }

 private:
  Translation2d m_translation;
  Rotation2d m_rotation;
};

constexpr Transform2d operator*(double scalar, const Transform2d& transform) {
  return transform * scalar;
}

}  // namespace turret_kinematics