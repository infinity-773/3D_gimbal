#pragma once

#include <array>

#include <units/length.h>

#include "turret_kinematics/Transform3d.h"

namespace turret_kinematics {

class Pose3d {
 public:
  constexpr Pose3d() = default;
  constexpr Pose3d(Translation3d translation, Rotation3d rotation)
      : m_translation{translation}, m_rotation{rotation} {}
  constexpr Pose3d(double x, double y, double z, Rotation3d rotation)
      : m_translation{x, y, z}, m_rotation{rotation} {}
  constexpr Pose3d(units::meter_t x, units::meter_t y, units::meter_t z,
                   Rotation3d rotation)
      : m_translation{x, y, z}, m_rotation{rotation} {}

  constexpr const Translation3d& Translation() const { return m_translation; }
  constexpr const Rotation3d& Rotation() const { return m_rotation; }
  constexpr units::meter_t X() const { return m_translation.X(); }
  constexpr units::meter_t Y() const { return m_translation.Y(); }
  constexpr units::meter_t Z() const { return m_translation.Z(); }

  Pose3d operator+(const Transform3d& transform) const {
    return TransformBy(transform);
  }
  Transform3d operator-(const Pose3d& other) const;
  Pose3d RotateBy(const Rotation3d& rotation) const {
    return {m_translation.RotateBy(rotation), m_rotation.RotateBy(rotation)};
  }
  Pose3d TransformBy(const Transform3d& transform) const {
    return {m_translation + transform.Translation().RotateBy(m_rotation),
            transform.Rotation().RotateBy(m_rotation)};
  }
  Pose3d RelativeTo(const Pose3d& other) const {
    const Transform3d transform{other, *this};
    return {transform.Translation(), transform.Rotation()};
  }
  Pose3d RotateAround(const Translation3d& point,
                      const Rotation3d& rotation) const {
    return {m_translation.RotateAround(point, rotation),
            m_rotation.RotateBy(rotation)};
  }
  std::array<std::array<double, 4>, 4> ToMatrix() const {
    const auto rotationMatrix = m_rotation.ToMatrix();
    return {{{rotationMatrix[0][0], rotationMatrix[0][1], rotationMatrix[0][2], X().value()},
         {rotationMatrix[1][0], rotationMatrix[1][1], rotationMatrix[1][2], Y().value()},
         {rotationMatrix[2][0], rotationMatrix[2][1], rotationMatrix[2][2], Z().value()},
             {0.0, 0.0, 0.0, 1.0}}};
  }

 private:
  Translation3d m_translation;
  Rotation3d m_rotation;
};

inline Transform3d::Transform3d(const Pose3d& initial, const Pose3d& final)
    : m_translation{(final.Translation() - initial.Translation())
                         .RotateBy(-initial.Rotation())},
      m_rotation{final.Rotation().RelativeTo(initial.Rotation())} {}

inline Transform3d Transform3d::operator+(const Transform3d& other) const {
  return Pose3d{}.TransformBy(*this).TransformBy(other) - Pose3d{};
}

inline Transform3d Pose3d::operator-(const Pose3d& other) const {
  const Pose3d pose = RelativeTo(other);
  return {pose.Translation(), pose.Rotation()};
}

}  // namespace turret_kinematics