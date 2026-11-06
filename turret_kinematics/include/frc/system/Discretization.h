#pragma once

#include <stdexcept>

#include <units/time.h>

#include "frc/system/Matrix.h"

namespace frc {

template <int States, int Inputs>
void DiscretizeAB(const Matrixd<States, States>& continuousA,
                  const Matrixd<States, Inputs>& continuousB,
                  units::second_t dt, Matrixd<States, States>* discreteA,
                  Matrixd<States, Inputs>* discreteB) {
  if (dt.value() <= 0.0) {
    throw std::invalid_argument("Discretization timestep must be positive");
  }
  Matrixd<States + Inputs, States + Inputs> augmented;
  for (int row = 0; row < States; ++row) {
    for (int col = 0; col < States; ++col) {
      augmented(row, col) = continuousA(row, col);
    }
    for (int col = 0; col < Inputs; ++col) {
      augmented(row, States + col) = continuousB(row, col);
    }
  }

  const auto exponential = (augmented * dt.value()).exp();
  for (int row = 0; row < States; ++row) {
    for (int col = 0; col < States; ++col) {
      (*discreteA)(row, col) = exponential(row, col);
    }
    for (int col = 0; col < Inputs; ++col) {
      (*discreteB)(row, col) = exponential(row, States + col);
    }
  }
}

template <int States>
void DiscretizeAQ(const Matrixd<States, States>& continuousA,
                  const Matrixd<States, States>& continuousQ,
                  units::second_t dt, Matrixd<States, States>* discreteA,
                  Matrixd<States, States>* discreteQ) {
  if (dt.value() <= 0.0) {
    throw std::invalid_argument("Discretization timestep must be positive");
  }
  const auto symmetricQ =
      (continuousQ + continuousQ.transpose()) * 0.5;
  Matrixd<2 * States, 2 * States> vanLoan;
  for (int row = 0; row < States; ++row) {
    for (int col = 0; col < States; ++col) {
      vanLoan(row, col) = -continuousA(row, col);
      vanLoan(row, States + col) = symmetricQ(row, col);
      vanLoan(States + row, States + col) = continuousA(col, row);
    }
  }

  const auto exponential = (vanLoan * dt.value()).exp();
  Matrixd<States, States> phi12;
  for (int row = 0; row < States; ++row) {
    for (int col = 0; col < States; ++col) {
      phi12(row, col) = exponential(row, States + col);
      (*discreteA)(row, col) = exponential(States + col, States + row);
    }
  }
  const auto q = *discreteA * phi12;
  *discreteQ = (q + q.transpose()) * 0.5;
}

template <int Outputs>
Matrixd<Outputs, Outputs> DiscretizeR(
    const Matrixd<Outputs, Outputs>& continuousR, units::second_t dt) {
  if (dt.value() <= 0.0) {
    throw std::invalid_argument("Discretization timestep must be positive");
  }
  return continuousR / dt.value();
}

}  // namespace frc