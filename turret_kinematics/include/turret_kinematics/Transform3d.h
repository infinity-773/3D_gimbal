#pragma once

#include "turret_kinematics/Rotation3d.h"

namespace turret_kinematics {

class Pose3d;

class Transform3d {
 public:
  constexpr Transform3d() = default;
  constexpr Transform3d(Translation3d translation, Rotation3d rotation)
      : m_translation{translation}, m_rotation{rotation} {}
  constexpr Transform3d(units::meter_t x, units::meter_t y, units::meter_t z,
                        Rotation3d rotation)
      : m_translation{x, y, z}, m_rotation{rotation} {}
  Transform3d(const Pose3d& initial, const Pose3d& final);

  constexpr const Translation3d& Translation() const { return m_translation; }
  constexpr const Rotation3d& Rotation() const { return m_rotation; }
  constexpr units::meter_t X() const { return m_translation.X(); }
  constexpr units::meter_t Y() const { return m_translation.Y(); }
  constexpr units::meter_t Z() const { return m_translation.Z(); }
  Transform3d Inverse() const {
    const Rotation3d inverseRotation = -m_rotation;
    return {(-m_translation).RotateBy(inverseRotation), inverseRotation};
  }
  Transform3d operator+(const Transform3d& other) const;
  Transform3d operator*(double scalar) const {
    return {m_translation * scalar, m_rotation * scalar};
  }

 private:
  Translation3d m_translation;
  Rotation3d m_rotation;
};

}  // namespace turret_kinematics