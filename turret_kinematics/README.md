# Turret Kinematics

A small, standalone C++20 header-only package for 3D coordinates, frame transforms, and turret aiming. It has no WPILib runtime or Eigen dependency. The geometry API follows WPILib C++ naming and operation conventions, in the `turret_kinematics` namespace.

## Contents

- `include/turret_kinematics/Translation3d.h`: 3D position/vector operations.
- `include/turret_kinematics/Translation2d.h` and `Rotation2d.h`: 2D translation and angle operations, following WPILib's geometry API.
- `include/turret_kinematics/Quaternion.h`: quaternion operations.
- `include/turret_kinematics/Rotation3d.h`: 3D orientation and rotation operations.
- `include/turret_kinematics/Transform3d.h`: relative coordinate transforms.
- `include/turret_kinematics/Pose3d.h`: 3D position and orientation.
- `include/turret_kinematics/Aimer.h`: configurable `Aimer` class for moving-target lead and constant-gravity ballistic aim.
- `include/turret_kinematics/TurretKinematics.h`: umbrella include for all package types.
- `include/frc/estimator/KalmanFilter.h`: WPILib-style `frc::KalmanFilter` implementation.
- `include/frc/system/`: self-contained fixed-size matrix, discretization, and linear-system support used by the filter.
- `include/units/`: header-only WPILib units library; it has no roboRIO dependency.
- `include/gcem.hpp`, `include/gcem_incl/`: units math dependency.
- `tests/`: small executable checks.

## Coordinate and unit convention

Use a right-handed coordinate frame: +X forward, +Y left, +Z up. Geometry coordinates use `units::meter_t`; rotation inputs/results use `units::radian_t`; velocity, acceleration, and time use their corresponding `units` types. Compatible units convert automatically, so inches can be passed where meters are expected. The units headers are bundled under `include/units` and do not depend on roboRIO. The ballistic solver defaults to gravity `{0, 0, -9.80665} m/s^2`.

For aiming, `targetFromMuzzle` is the target position relative to the muzzle, with axes aligned to the fixed world/base frame. `targetVelocityInWorld` is the target velocity in that same fixed frame; for a moving muzzle use velocity relative to the muzzle. Transform camera measurements and the muzzle position into the same frame before subtracting them. `Rotation3d{roll, pitch, yaw}` takes radians; `Rotation3d::FromDegrees(...)` is also available.

Aim angles are in radians: positive yaw points left and positive pitch points down, matching `Rotation3d{0, pitch, yaw}`. The launch velocity remains a world-frame vector, so an upward launch has positive Z and negative pitch.

## Example

```cpp
#include <iostream>

#include "turret_kinematics/TurretKinematics.h"

int main() {
  using namespace turret_kinematics;

  using namespace units::literals;
  const Translation3d targetFromMuzzle{8_m, 1_m, 2_m};
  const Velocity3d targetVelocity{-0.5_mps, 0_mps, 0_mps};

  Aimer::Config config;
  config.projectileSpeed = 18_mps;
  Aimer aimer{config};

  const auto solution = aimer.Calculate(targetFromMuzzle, targetVelocity);
  if (!solution) {
    std::cerr << "No ballistic solution in the configured flight-time range\n";
    return 1;
  }

  std::cout << "yaw rad: " << solution->angles.yawRadians.value() << '\n'
            << "pitch rad: " << solution->angles.pitchRadians.value() << '\n'
            << "flight seconds: " << solution->flightTime.value() << '\n';
}
```

## Kalman filter

The filter keeps WPILib's template parameters and main methods:

```cpp
#include "frc/estimator/KalmanFilter.h"

frc::Matrixd<1, 1> A{0.0};
frc::Matrixd<1, 1> B{1.0};
frc::Matrixd<1, 1> C{1.0};
frc::Matrixd<1, 1> D{0.0};
frc::LinearSystem<1, 1, 1> plant{A, B, C, D};
frc::KalmanFilter<1, 1, 1> filter{
    plant, {0.1}, {0.5}, units::second_t{0.02}};
frc::Vectord<1> input;
frc::Vectord<1> measurement;
filter.Predict(input, units::second_t{0.02});
filter.Correct(input, measurement);
```

Unlike the full WPILib implementation, this independent version uses its own `frc::Matrixd` instead of Eigen and uses `std::array` for standard-deviation arrays. `units::second_t` and the other copied units are included locally, so this code does not require WPILib or roboRIO. The filter uses matrix-exponential system discretization, continuous process/measurement noise covariance scaling, and Joseph-form covariance correction. Initial covariance is found by iterating the discrete Riccati covariance update; unstable or undetectable systems may fail to converge.

`Aimer::CalculateLead` solves the constant-speed intercept without gravity. `Aimer::Calculate` numerically searches for the earliest reachable intercept under constant gravity, up to the configured `maxFlightTime`; both return `std::nullopt` if no solution is found. The ballistic result assumes a fixed launch speed and a point target; it does not model drag, spin, launcher dynamics, or target acceleration.

## Build and test

With CMake and a C++20 compiler installed:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

To use it in another CMake project, add this directory with `add_subdirectory(...)` and link the `turret_kinematics` interface target. Without CMake, add `include/` to the compiler include paths.

The solution angles follow the frame convention above, not necessarily calibrated servo commands. Apply your turret's zero offsets, direction signs, travel limits, and coordinate-frame calibration before commanding hardware. Geometry methods use WPILib-style names such as `X()`, `Norm()`, `RotateBy()`, `TransformBy()`, and `RelativeTo()`. Include `units::literals` to use suffixes such as `_m`, `_deg`, `_mps`, and `_ms`; alternatively construct a value directly, such as `units::meter_t{1.5}`.

`Translation2d::Angle()` returns an optional `Rotation2d`; it is empty for a near-zero vector because its direction is undefined. The API follows WPILib's C++ [`Translation2d`](https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/native/include/wpi/math/geometry/Translation2d.hpp) conventions. Eigen-specific vector conversion is omitted to keep this library dependency-free.