#pragma once

#include <stdexcept>

#include <units/time.h>

#include "frc/system/Discretization.h"

namespace frc {

template <int States, int Inputs, int Outputs>
class LinearSystem {
 public:
  using StateVector = Vectord<States>;
  using InputVector = Vectord<Inputs>;
  using OutputVector = Vectord<Outputs>;

  constexpr LinearSystem(const Matrixd<States, States>& a,
                         const Matrixd<States, Inputs>& b,
                         const Matrixd<Outputs, States>& c,
                         const Matrixd<Outputs, Inputs>& d)
      : m_a{a}, m_b{b}, m_c{c}, m_d{d} {
    if (!a.allFinite() || !b.allFinite() || !c.allFinite() ||
        !d.allFinite()) {
      throw std::domain_error("Linear system matrices must be finite");
    }
  }

  constexpr const Matrixd<States, States>& A() const { return m_a; }
  constexpr const Matrixd<States, Inputs>& B() const { return m_b; }
  constexpr const Matrixd<Outputs, States>& C() const { return m_c; }
  constexpr const Matrixd<Outputs, Inputs>& D() const { return m_d; }

  StateVector CalculateX(const StateVector& x, const InputVector& u,
                         units::second_t dt) const {
    Matrixd<States, States> discreteA;
    Matrixd<States, Inputs> discreteB;
    DiscretizeAB(m_a, m_b, dt, &discreteA, &discreteB);
    return discreteA * x + discreteB * u;
  }

 private:
  Matrixd<States, States> m_a;
  Matrixd<States, Inputs> m_b;
  Matrixd<Outputs, States> m_c;
  Matrixd<Outputs, Inputs> m_d;
};

}  // namespace frc