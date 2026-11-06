#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

#include <units/time.h>

#include "frc/system/Discretization.h"
#include "frc/system/LinearSystem.h"

namespace frc {

template <int States, int Inputs, int Outputs>
class KalmanFilter {
 public:
  using StateVector = Vectord<States>;
  using InputVector = Vectord<Inputs>;
  using OutputVector = Vectord<Outputs>;
  using StateArray = std::array<double, States>;
  using OutputArray = std::array<double, Outputs>;
  using StateMatrix = Matrixd<States, States>;

  KalmanFilter(LinearSystem<States, Inputs, Outputs>& plant,
               const StateArray& stateStdDevs,
               const OutputArray& measurementStdDevs, units::second_t dt)
      : m_plant{&plant}, m_dt{dt} {
    if (dt.value() <= 0.0) {
      throw std::invalid_argument("Kalman timestep must be positive");
    }
    for (double standardDeviation : stateStdDevs) {
      if (!std::isfinite(standardDeviation) || standardDeviation < 0.0) {
        throw std::invalid_argument("State standard deviations must be finite and non-negative");
      }
    }
    for (double standardDeviation : measurementStdDevs) {
      if (!std::isfinite(standardDeviation) || standardDeviation <= 0.0) {
        throw std::invalid_argument("Measurement standard deviations must be finite and positive");
      }
    }

    for (int index = 0; index < States; ++index) {
      m_contQ(index, index) = stateStdDevs[static_cast<std::size_t>(index)] *
                              stateStdDevs[static_cast<std::size_t>(index)];
    }
    for (int index = 0; index < Outputs; ++index) {
      m_contR(index, index) =
          measurementStdDevs[static_cast<std::size_t>(index)] *
          measurementStdDevs[static_cast<std::size_t>(index)];
    }

    m_initP = SolveSteadyStateCovariance();
    Reset();
  }

  const StateMatrix& P() const { return m_p; }
  double P(int row, int col) const { return m_p(row, col); }
  void SetP(const StateMatrix& covariance) { m_p = covariance; }

  const StateVector& Xhat() const { return m_xHat; }
  double Xhat(int index) const { return m_xHat(index); }
  void SetXhat(const StateVector& estimate) { m_xHat = estimate; }
  void SetXhat(int index, double value) { m_xHat(index) = value; }

  void Reset() {
    m_xHat.setZero();
    m_p = m_initP;
  }

  void Predict(const InputVector& input, units::second_t dt) {
    Matrixd<States, States> discreteA;
    Matrixd<States, States> discreteQ;
    DiscretizeAQ(m_plant->A(), m_contQ, dt, &discreteA, &discreteQ);
    m_xHat = m_plant->CalculateX(m_xHat, input, dt);
    m_p = discreteA * m_p * discreteA.transpose() + discreteQ;
    m_dt = dt;
  }

  void Correct(const InputVector& input, const OutputVector& measurement) {
    Correct(input, measurement, m_contR);
  }

  void Correct(const InputVector& input, const OutputVector& measurement,
               const Matrixd<Outputs, Outputs>& measurementCovariance) {
    const auto& c = m_plant->C();
    const auto& d = m_plant->D();
    const auto discreteR = DiscretizeR(measurementCovariance, m_dt);
    const auto innovationCovariance = c * m_p * c.transpose() + discreteR;
    const auto gain =
        m_p * c.transpose() * innovationCovariance.inverse();
    const OutputVector residual =
        measurement - (c * m_xHat + d * input);
    m_xHat += gain * residual;

    const StateMatrix identity = StateMatrix::Identity();
    const StateMatrix correction = identity - gain * c;
    m_p = correction * m_p * correction.transpose() +
          gain * discreteR * gain.transpose();
  }

 private:
  StateMatrix SolveSteadyStateCovariance() const {
    Matrixd<States, States> discreteA;
    Matrixd<States, States> discreteQ;
    DiscretizeAQ(m_plant->A(), m_contQ, m_dt, &discreteA, &discreteQ);
    const auto discreteR = DiscretizeR(m_contR, m_dt);
    const auto& c = m_plant->C();

    StateMatrix covariance = StateMatrix::Identity();
    for (int iteration = 0; iteration < 10000; ++iteration) {
      const StateMatrix predicted =
          discreteA * covariance * discreteA.transpose() + discreteQ;
      const auto innovation = c * predicted * c.transpose() + discreteR;
      const auto gain = predicted * c.transpose() * innovation.inverse();
      const StateMatrix identity = StateMatrix::Identity();
      const StateMatrix correction = identity - gain * c;
      const StateMatrix corrected =
          correction * predicted * correction.transpose() +
          gain * discreteR * gain.transpose();
      const StateMatrix next =
          discreteA * corrected * discreteA.transpose() + discreteQ;
      if (!next.allFinite()) {
        throw std::runtime_error("Kalman covariance iteration diverged");
      }
      if ((next - covariance).norm() <= 1e-10 * std::max(1.0, next.norm())) {
        return next;
      }
      covariance = next;
    }
    throw std::invalid_argument(
        "Kalman covariance did not converge; system may not be detectable");
  }

  LinearSystem<States, Inputs, Outputs>* m_plant;
  StateVector m_xHat;
  StateMatrix m_p;
  StateMatrix m_contQ;
  Matrixd<Outputs, Outputs> m_contR;
  units::second_t m_dt;
  StateMatrix m_initP;
};

}  // namespace frc