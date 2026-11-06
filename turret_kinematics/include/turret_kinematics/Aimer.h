// 此类用于接受目标相对于炮口的位置和速度，计算出炮台的目标角度//
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/velocity.h>

#include "turret_kinematics/Translation3d.h"

namespace turret_kinematics {

struct AimAngles {
  units::radian_t yawRadians;
  units::radian_t pitchRadians;
};

using Velocity3d = std::array<units::meters_per_second_t, 3>;
using Acceleration3d =
    std::array<units::meters_per_second_squared_t, 3>;

struct AimSetpoint {
  AimAngles angles;
  units::second_t flightTime;
  Velocity3d launchVelocity;
};

namespace detail {

inline AimAngles AnglesTo(const Translation3d& target) {
  if (!target.IsFinite() || target.Norm().value() <= 1e-12) {
    throw std::invalid_argument("Target vector must be finite and non-zero");
  }
  return {units::radian_t{std::atan2(target.Y().value(), target.X().value())},
          units::radian_t{std::atan2(
              target.Z().value(),
              std::hypot(target.X().value(), target.Y().value()))}};
}

inline AimSetpoint MakeAimSetpoint(const std::array<double, 3>& displacement,
                                   double flightTimeSeconds) {
  if (flightTimeSeconds <= 0.0) {
    throw std::invalid_argument("Flight time must be positive");
  }
  const Translation3d launchDisplacement{
      units::meter_t{displacement[0]}, units::meter_t{displacement[1]},
      units::meter_t{displacement[2]}};
  const Velocity3d launchVelocity{
      units::meters_per_second_t{displacement[0] / flightTimeSeconds},
      units::meters_per_second_t{displacement[1] / flightTimeSeconds},
      units::meters_per_second_t{displacement[2] / flightTimeSeconds}};
  return {AnglesTo(launchDisplacement), units::second_t{flightTimeSeconds},
          launchVelocity};
}

inline double EvaluatePolynomial(const std::vector<double>& coefficients,
                                 double value) {
  double result = coefficients.back();
  for (std::size_t index = coefficients.size() - 1; index > 0; --index) {
    result = result * value + coefficients[index - 1];
  }
  return result;
}

inline double PolynomialScale(const std::vector<double>& coefficients,
                              double value) {
  double scale = 0.0;
  const double absoluteValue = std::abs(value);
  for (std::size_t index = coefficients.size(); index > 0; --index) {
    scale = scale * absoluteValue + std::abs(coefficients[index - 1]);
  }
  return scale;
}

inline void AddUniqueRoot(std::vector<double>& roots, double root) {
  constexpr double kRootTolerance = 1e-10;
  for (double existing : roots) {
    if (std::abs(existing - root) <=
        kRootTolerance * std::max({1.0, std::abs(existing), std::abs(root)})) {
      return;
    }
  }
  roots.push_back(root);
}

// Derivative roots split the polynomial into monotonic intervals.
inline std::vector<double> FindPolynomialRoots(
    std::vector<double> coefficients, double lowerBound, double upperBound,
    std::size_t bisectionSteps) {
  while (coefficients.size() > 1 && coefficients.back() == 0.0) {
    coefficients.pop_back();
  }
  if (coefficients.size() <= 1) {
    return {};
  }

  const double coefficientScale =
      *std::max_element(coefficients.begin(), coefficients.end(),
                        [](double lhs, double rhs) {
                          return std::abs(lhs) < std::abs(rhs);
                        });
  const double normalization = std::abs(coefficientScale);
  if (normalization == 0.0 || !std::isfinite(normalization)) {
    return {};
  }
  for (double& coefficient : coefficients) {
    coefficient /= normalization;
  }

  if (coefficients.size() == 2) {
    const double root = -coefficients[0] / coefficients[1];
    if (root >= lowerBound && root <= upperBound) {
      return {root};
    }
    return {};
  }

  std::vector<double> derivative(coefficients.size() - 1);
  for (std::size_t index = 1; index < coefficients.size(); ++index) {
    derivative[index - 1] = coefficients[index] * index;
  }
  std::vector<double> criticalPoints = FindPolynomialRoots(
      std::move(derivative), lowerBound, upperBound, bisectionSteps);
  std::sort(criticalPoints.begin(), criticalPoints.end());

  std::vector<double> boundaries{lowerBound};
  for (double point : criticalPoints) {
    if (point > lowerBound && point < upperBound) {
      boundaries.push_back(point);
    }
  }
  boundaries.push_back(upperBound);

  std::vector<double> roots;
  constexpr double kValueTolerance =
      64.0 * std::numeric_limits<double>::epsilon();
  for (double boundary : boundaries) {
    const double value = EvaluatePolynomial(coefficients, boundary);
    const double scale = PolynomialScale(coefficients, boundary);
    if (std::abs(value) <= kValueTolerance * std::max(1.0, scale)) {
      AddUniqueRoot(roots, boundary);
    }
  }

  for (std::size_t interval = 1; interval < boundaries.size(); ++interval) {
    double lower = boundaries[interval - 1];
    double upper = boundaries[interval];
    double lowerValue = EvaluatePolynomial(coefficients, lower);
    double upperValue = EvaluatePolynomial(coefficients, upper);
    if ((lowerValue < 0.0) == (upperValue < 0.0) || lowerValue == 0.0 ||
        upperValue == 0.0) {
      continue;
    }

    for (std::size_t iteration = 0; iteration < bisectionSteps; ++iteration) {
      const double middle = lower + (upper - lower) * 0.5;
      if (middle == lower || middle == upper) {
        break;
      }
      const double middleValue = EvaluatePolynomial(coefficients, middle);
      if (middleValue == 0.0) {
        lower = middle;
        upper = middle;
        break;
      }
      if ((middleValue < 0.0) == (lowerValue < 0.0)) {
        lower = middle;
        lowerValue = middleValue;
      } else {
        upper = middle;
        upperValue = middleValue;
      }
    }
    AddUniqueRoot(roots, lower + (upper - lower) * 0.5);
  }

  std::sort(roots.begin(), roots.end());
  return roots;
}

inline std::optional<AimSetpoint> SolveLead(
    const Translation3d& targetRelativePosition,
    const Velocity3d& targetVelocity,
    units::meters_per_second_t projectileSpeed) {
  if (!targetRelativePosition.IsFinite() ||
      !std::isfinite(targetVelocity[0].value()) ||
      !std::isfinite(targetVelocity[1].value()) ||
      !std::isfinite(targetVelocity[2].value()) ||
      !std::isfinite(projectileSpeed.value()) || projectileSpeed.value() <= 0.0) {
    throw std::invalid_argument("Aim inputs must be finite and speed positive");
  }

  const std::array<double, 3> position{
      targetRelativePosition.X().value(), targetRelativePosition.Y().value(),
      targetRelativePosition.Z().value()};
  const std::array<double, 3> velocity{
      targetVelocity[0].value(), targetVelocity[1].value(),
      targetVelocity[2].value()};
  const double speed = projectileSpeed.value();
  const double a = velocity[0] * velocity[0] + velocity[1] * velocity[1] +
                   velocity[2] * velocity[2] - speed * speed;
  const double b = 2.0 * (position[0] * velocity[0] +
                          position[1] * velocity[1] +
                          position[2] * velocity[2]);
  const double c = position[0] * position[0] + position[1] * position[1] +
                   position[2] * position[2];

  double flightTime = 0.0;
  if (std::abs(a) <= 1e-12) {
    if (std::abs(b) <= 1e-12) {
      return std::nullopt;
    }
    flightTime = -c / b;
    if (flightTime <= 0.0) {
      return std::nullopt;
    }
  } else {
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      return std::nullopt;
    }
    const double root = std::sqrt(discriminant);
    const double first = (-b - root) / (2.0 * a);
    const double second = (-b + root) / (2.0 * a);
    if (first > 0.0 && second > 0.0) {
      flightTime = std::min(first, second);
    } else if (first > 0.0) {
      flightTime = first;
    } else if (second > 0.0) {
      flightTime = second;
    } else {
      return std::nullopt;
    }
  }

  const std::array<double, 3> intercept{
      position[0] + velocity[0] * flightTime,
      position[1] + velocity[1] * flightTime,
      position[2] + velocity[2] * flightTime};
  return MakeAimSetpoint(intercept, flightTime);
}

inline std::optional<AimSetpoint> SolveBallistic(
    const Translation3d& targetRelativePosition,
    const Velocity3d& targetVelocity,
    units::meters_per_second_t projectileSpeed,
    const Acceleration3d& gravity, units::second_t maxFlightTime,
    std::size_t scanSteps, std::size_t bisectionSteps) {
  if (!targetRelativePosition.IsFinite() ||
      !std::isfinite(targetVelocity[0].value()) ||
      !std::isfinite(targetVelocity[1].value()) ||
      !std::isfinite(targetVelocity[2].value()) ||
      !std::isfinite(gravity[0].value()) ||
      !std::isfinite(gravity[1].value()) ||
      !std::isfinite(gravity[2].value()) ||
      !std::isfinite(projectileSpeed.value()) || projectileSpeed.value() <= 0.0 ||
      !std::isfinite(maxFlightTime.value()) || maxFlightTime.value() <= 0.0 ||
      scanSteps == 0 || bisectionSteps == 0) {
    throw std::invalid_argument("Invalid ballistic aim inputs");
  }
  if (targetRelativePosition.Norm().value() <= 1e-12) {
    throw std::invalid_argument("Target vector must be non-zero");
  }

  const std::array<double, 3> position{
      targetRelativePosition.X().value(), targetRelativePosition.Y().value(),
      targetRelativePosition.Z().value()};
  const std::array<double, 3> velocity{
      targetVelocity[0].value(), targetVelocity[1].value(),
      targetVelocity[2].value()};
  const std::array<double, 3> gravityValues{
      gravity[0].value(), gravity[1].value(), gravity[2].value()};
  const double speed = projectileSpeed.value();
  const auto displacementAt = [&](double time) {
    return std::array<double, 3>{
        position[0] + velocity[0] * time - 0.5 * gravityValues[0] * time * time,
        position[1] + velocity[1] * time - 0.5 * gravityValues[1] * time * time,
        position[2] + velocity[2] * time - 0.5 * gravityValues[2] * time * time};
  };

  const double positionDotVelocity =
      position[0] * velocity[0] + position[1] * velocity[1] +
      position[2] * velocity[2];
  const double velocitySquared = velocity[0] * velocity[0] +
                                 velocity[1] * velocity[1] +
                                 velocity[2] * velocity[2];
  const double positionDotGravity =
      position[0] * gravityValues[0] + position[1] * gravityValues[1] +
      position[2] * gravityValues[2];
  const double velocityDotGravity =
      velocity[0] * gravityValues[0] + velocity[1] * gravityValues[1] +
      velocity[2] * gravityValues[2];
  const double gravitySquared = gravityValues[0] * gravityValues[0] +
                                gravityValues[1] * gravityValues[1] +
                                gravityValues[2] * gravityValues[2];
  // Squaring |p + vt - 0.5gt^2| = speed * t yields this quartic.
  const std::vector<double> distancePolynomial{
      position[0] * position[0] + position[1] * position[1] +
          position[2] * position[2],
      2.0 * positionDotVelocity,
      velocitySquared - positionDotGravity - speed * speed,
      -velocityDotGravity, 0.25 * gravitySquared};

  const auto roots = FindPolynomialRoots(
      distancePolynomial, 0.0, maxFlightTime.value(), bisectionSteps);
  for (double flightTime : roots) {
    if (flightTime > 0.0) {
      return MakeAimSetpoint(displacementAt(flightTime), flightTime);
    }
  }
  return std::nullopt;
}

}  // namespace detail

class Aimer {
 public:
  struct Config {
    units::meters_per_second_t projectileSpeed{18.0};
    Acceleration3d gravity{
        units::meters_per_second_squared_t{0.0},
        units::meters_per_second_squared_t{0.0},
        units::meters_per_second_squared_t{-9.80665}};
    units::second_t maxFlightTime{5.0};
    // Kept for API compatibility; no longer used by the root solver.
    std::size_t scanSteps = 2048;
    std::size_t bisectionSteps = 64;
  };

  Aimer() = default;
  explicit Aimer(Config config) : m_config{config} {}

  std::optional<AimSetpoint> Calculate(
      const Translation3d& targetRelativePosition,
      const Velocity3d& targetVelocity) const {
    return detail::SolveBallistic(
        targetRelativePosition, targetVelocity, m_config.projectileSpeed,
        m_config.gravity, m_config.maxFlightTime, m_config.scanSteps,
        m_config.bisectionSteps);
  }

  std::optional<AimSetpoint> CalculateLead(
      const Translation3d& targetRelativePosition,
      const Velocity3d& targetVelocity) const {
    return detail::SolveLead(targetRelativePosition, targetVelocity,
                             m_config.projectileSpeed);
  }

  const Config& GetConfig() const { return m_config; }
  void SetConfig(Config config) { m_config = config; }

 private:
  Config m_config;
};

}  // namespace turret_kinematics