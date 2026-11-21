#pragma once

#include <array>

#include <units/length.h>

#include "turret_kinematics/Transform2d.h"

namespace turret_kinematics {

class Pose2d {
 public:
  constexpr Pose2d() = default;
  constexpr Pose2d(Translation2d translation, Rotation2d rotation)
      : m_translation{translation}, m_rotation{rotation} {}
  constexpr Pose2d(double x, double y, Rotation2d rotation)
      : m_translation{x, y}, m_rotation{rotation} {}
  constexpr Pose2d(units::meter_t x, units::meter_t y, Rotation2d rotation)
      : m_translation{x, y}, m_rotation{rotation} {}

  constexpr const Translation2d& Translation() const { return m_translation; }
  constexpr const Rotation2d& Rotation() const { return m_rotation; }
  constexpr units::meter_t X() const { return m_translation.X(); }
  constexpr units::meter_t Y() const { return m_translation.Y(); }

  constexpr Pose2d operator+(const Transform2d& transform) const {
    return TransformBy(transform);
  }
  Transform2d operator-(const Pose2d& other) const;
  constexpr Pose2d operator*(double scalar) const {
    return {m_translation * scalar, m_rotation * scalar};
  }
  constexpr bool operator==(const Pose2d& other) const {
    return m_translation == other.m_translation && m_rotation == other.m_rotation;
  }

  constexpr Pose2d TransformBy(const Transform2d& transform) const {
    return {m_translation + transform.Translation().RotateBy(m_rotation),
            m_rotation + transform.Rotation()};
  }
  constexpr Pose2d RelativeTo(const Pose2d& other) const {
    const Rotation2d inverseRotation = -other.m_rotation;
    return {(m_translation - other.m_translation).RotateBy(inverseRotation),
            m_rotation.RelativeTo(other.m_rotation)};
  }
  constexpr Pose2d RotateBy(const Rotation2d& rotation) const {
    return {m_translation.RotateBy(rotation), m_rotation + rotation};
  }
  constexpr Pose2d RotateAround(const Translation2d& point,
                               const Rotation2d& rotation) const {
    return {m_translation.RotateAround(point, rotation), m_rotation + rotation};
  }
  constexpr std::array<std::array<double, 3>, 3> ToMatrix() const {
    return {{{m_rotation.Cos(), -m_rotation.Sin(), X().value()},
             {m_rotation.Sin(), m_rotation.Cos(), Y().value()},
             {0.0, 0.0, 1.0}}};
  }

 private:
  Translation2d m_translation;
  Rotation2d m_rotation;
};

inline Transform2d::Transform2d(const Pose2d& initial, const Pose2d& final)
    : m_translation{(final.Translation() - initial.Translation())
                        .RotateBy(-initial.Rotation())},
      m_rotation{final.Rotation().RelativeTo(initial.Rotation())} {}

inline Transform2d Pose2d::operator-(const Pose2d& other) const {
  return {other, *this};
}

}  // namespace turret_kinematics