#include <cassert>
#include <cmath>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include "turret_kinematics/TurretKinematics.h"
#include <units/time.h>
#include <units/velocity.h>
#include "frc/estimator/KalmanFilter.h"

using turret_kinematics::Pose3d;
using turret_kinematics::Rotation3d;
using turret_kinematics::Translation3d;
using turret_kinematics::Transform3d;

int main() {
  using namespace units::literals;

  const auto cross =
    Translation3d{1.0, 0.0, 0.0}.Cross({0.0, 1.0, 0.0});
  assert(std::abs(cross[2].value() - 1.0) < 1e-9);

  const Translation3d inchPosition{12_in, 0_m, 0_m};
  assert(std::abs(inchPosition.X().value() - 0.3048) < 1e-9);
  const Rotation3d unitRotation{units::degree_t{0}, units::degree_t{0},
                units::degree_t{90}};
  assert(std::abs(unitRotation.Z().value() - 1.5707963267948966) < 1e-9);

  const Rotation3d quarterTurn{0.0, 0.0, 1.5707963267948966};
  const Pose3d frame{{1.0, 0.0, 0.0}, quarterTurn};
  const Pose3d transformed =
    frame.TransformBy({{1.0, 0.0, 0.0}, Rotation3d{}});
  assert(std::abs(transformed.X().value() - 1.0) < 1e-9);
  assert(std::abs(transformed.Y().value() - 1.0) < 1e-9);

    const turret_kinematics::Velocity3d targetVelocity{
    units::meters_per_second_t{0.0}, units::meters_per_second_t{1.0},
    units::meters_per_second_t{0.0}};
    turret_kinematics::Aimer::Config leadConfig;
    leadConfig.projectileSpeed = units::meters_per_second_t{10.0};
    const turret_kinematics::Aimer leadAimer{leadConfig};
    const auto lead = leadAimer.CalculateLead({10.0, 0.0, 0.0}, targetVelocity);
  assert(lead.has_value());
  assert(lead->angles.yawRadians.value() > 0.0);
  assert(std::abs(std::hypot(lead->launchVelocity[0].value(),
               lead->launchVelocity[1].value(),
               lead->launchVelocity[2].value()) -
          10.0) < 1e-8);

    const turret_kinematics::Velocity3d stationaryTarget{
    units::meters_per_second_t{0.0}, units::meters_per_second_t{0.0},
    units::meters_per_second_t{0.0}};
    turret_kinematics::Aimer::Config ballisticConfig;
    ballisticConfig.projectileSpeed = units::meters_per_second_t{20.0};
    ballisticConfig.gravity = {
      units::meters_per_second_squared_t{0.0},
      units::meters_per_second_squared_t{0.0},
      units::meters_per_second_squared_t{-9.80665}};
    const turret_kinematics::Aimer ballisticAimer{ballisticConfig};
    const auto ballistic =
      ballisticAimer.Calculate({10.0, 0.0, 0.0}, stationaryTarget);
  assert(ballistic.has_value());
  assert(ballistic->angles.pitchRadians.value() > 0.0);
  assert(std::abs(std::hypot(ballistic->launchVelocity[0].value(),
               ballistic->launchVelocity[1].value(),
               ballistic->launchVelocity[2].value()) -
          20.0) < 1e-6);
  if (!ballistic.has_value() ||
      std::abs(std::hypot(ballistic->launchVelocity[0].value(),
                          ballistic->launchVelocity[1].value(),
                          ballistic->launchVelocity[2].value()) -
               20.0) > 1e-6) {
    return 3;
  }
  if (ballistic->flightTime.value() >= 1.0) {
    return 6;
  }

  turret_kinematics::Aimer::Config tangentConfig;
  tangentConfig.projectileSpeed = units::meters_per_second_t{10.0};
  tangentConfig.gravity = {
      units::meters_per_second_squared_t{0.0},
      units::meters_per_second_squared_t{0.0},
      units::meters_per_second_squared_t{-10.0}};
  tangentConfig.maxFlightTime = units::second_t{2.0};
  tangentConfig.scanSteps = 1;
  const turret_kinematics::Aimer tangentAimer{tangentConfig};
  const auto tangent =
      tangentAimer.Calculate({10.0, 0.0, 0.0}, stationaryTarget);
  if (!tangent.has_value()) {
    return 1;
  }
  if (std::abs(tangent->flightTime.value() - std::sqrt(2.0)) >= 1e-8) {
    return 2;
  }
  if (std::abs(std::hypot(tangent->launchVelocity[0].value(),
                          tangent->launchVelocity[1].value(),
                          tangent->launchVelocity[2].value()) -
               10.0) > 1e-8) {
    return 4;
  }
  if (tangentAimer.Calculate({10.1, 0.0, 0.0}, stationaryTarget).has_value()) {
    return 5;
  }

  frc::Matrixd<1, 1> a{0.0};
  frc::Matrixd<1, 1> b{1.0};
  frc::Matrixd<1, 1> c{1.0};
  frc::Matrixd<1, 1> d{0.0};
  frc::LinearSystem<1, 1, 1> plant{a, b, c, d};
  frc::KalmanFilter<1, 1, 1> filter{
    plant, {0.1}, {0.5}, units::second_t{0.02}};

  frc::Vectord<1> input;
  input(0) = 0.0;
  frc::Vectord<1> measurement;
  measurement(0) = 1.0;
  filter.Predict(input, units::second_t{0.02});
  filter.Correct(input, measurement);
  assert(filter.Xhat(0) > 0.0);
  assert(filter.Xhat(0) < 1.0);
}