#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace frc {

template <int Rows, int Cols>
class Matrixd {
 public:
  Matrixd() = default;

  Matrixd(std::initializer_list<double> values) {
    if (values.size() != m_values.size()) {
      throw std::invalid_argument("Matrix initializer has the wrong size");
    }
    std::copy(values.begin(), values.end(), m_values.begin());
  }

  static constexpr int rows() { return Rows; }
  static constexpr int cols() { return Cols; }

  double& operator()(int row, int col) {
    return m_values.at(static_cast<std::size_t>(row * Cols + col));
  }
  const double& operator()(int row, int col) const {
    return m_values.at(static_cast<std::size_t>(row * Cols + col));
  }
  double& operator()(int index) {
    static_assert(Cols == 1, "Single-index access is only for column vectors");
    return m_values.at(static_cast<std::size_t>(index));
  }
  const double& operator()(int index) const {
    static_assert(Cols == 1, "Single-index access is only for column vectors");
    return m_values.at(static_cast<std::size_t>(index));
  }

  void setZero() { m_values.fill(0.0); }

  static Matrixd Identity() {
    static_assert(Rows == Cols, "Identity requires a square matrix");
    Matrixd result;
    for (int index = 0; index < Rows; ++index) {
      result(index, index) = 1.0;
    }
    return result;
  }

  Matrixd<Cols, Rows> transpose() const {
    Matrixd<Cols, Rows> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(col, row) = (*this)(row, col);
      }
    }
    return result;
  }

  bool allFinite() const {
    return std::all_of(m_values.begin(), m_values.end(),
                       [](double value) { return std::isfinite(value); });
  }

  double norm() const {
    double sum = 0.0;
    for (double value : m_values) {
      sum += value * value;
    }
    return std::sqrt(sum);
  }

  Matrixd operator+(const Matrixd& other) const {
    Matrixd result;
    for (std::size_t index = 0; index < m_values.size(); ++index) {
      result.m_values[index] = m_values[index] + other.m_values[index];
    }
    return result;
  }

  Matrixd operator-(const Matrixd& other) const {
    Matrixd result;
    for (std::size_t index = 0; index < m_values.size(); ++index) {
      result.m_values[index] = m_values[index] - other.m_values[index];
    }
    return result;
  }

  Matrixd& operator+=(const Matrixd& other) {
    for (std::size_t index = 0; index < m_values.size(); ++index) {
      m_values[index] += other.m_values[index];
    }
    return *this;
  }

  Matrixd& operator-=(const Matrixd& other) {
    for (std::size_t index = 0; index < m_values.size(); ++index) {
      m_values[index] -= other.m_values[index];
    }
    return *this;
  }

  Matrixd operator*(double scalar) const {
    Matrixd result;
    for (std::size_t index = 0; index < m_values.size(); ++index) {
      result.m_values[index] = m_values[index] * scalar;
    }
    return result;
  }

  Matrixd operator/(double scalar) const {
    if (std::abs(scalar) <= 1e-15) {
      throw std::invalid_argument("Matrix division by zero");
    }
    return *this * (1.0 / scalar);
  }

  template <int OtherCols>
  Matrixd<Rows, OtherCols> operator*(
      const Matrixd<Cols, OtherCols>& other) const {
    Matrixd<Rows, OtherCols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int inner = 0; inner < Cols; ++inner) {
        for (int col = 0; col < OtherCols; ++col) {
          result(row, col) += (*this)(row, inner) * other(inner, col);
        }
      }
    }
    return result;
  }

  Matrixd inverse() const {
    static_assert(Rows == Cols, "Inverse requires a square matrix");
    Matrixd left = *this;
    Matrixd right = Identity();

    for (int pivot = 0; pivot < Rows; ++pivot) {
      int bestRow = pivot;
      for (int row = pivot + 1; row < Rows; ++row) {
        if (std::abs(left(row, pivot)) > std::abs(left(bestRow, pivot))) {
          bestRow = row;
        }
      }
      if (std::abs(left(bestRow, pivot)) <= 1e-14) {
        throw std::domain_error("Matrix is singular");
      }
      if (bestRow != pivot) {
        for (int col = 0; col < Rows; ++col) {
          std::swap(left(pivot, col), left(bestRow, col));
          std::swap(right(pivot, col), right(bestRow, col));
        }
      }

      const double divisor = left(pivot, pivot);
      for (int col = 0; col < Rows; ++col) {
        left(pivot, col) /= divisor;
        right(pivot, col) /= divisor;
      }
      for (int row = 0; row < Rows; ++row) {
        if (row == pivot) {
          continue;
        }
        const double factor = left(row, pivot);
        for (int col = 0; col < Rows; ++col) {
          left(row, col) -= factor * left(pivot, col);
          right(row, col) -= factor * right(pivot, col);
        }
      }
    }
    return right;
  }

  Matrixd exp() const {
    static_assert(Rows == Cols, "Matrix exponential requires a square matrix");
    double normOne = 0.0;
    for (int col = 0; col < Cols; ++col) {
      double columnSum = 0.0;
      for (int row = 0; row < Rows; ++row) {
        columnSum += std::abs((*this)(row, col));
      }
      normOne = std::max(normOne, columnSum);
    }

    int squarings = 0;
    if (normOne > 0.5) {
      squarings = static_cast<int>(std::ceil(std::log2(normOne / 0.5)));
    }
    const Matrixd scaled = *this / std::ldexp(1.0, squarings);
    Matrixd result = Identity();
    Matrixd term = Identity();
    for (int order = 1; order <= 48; ++order) {
      term = (term * scaled) / static_cast<double>(order);
      result += term;
      if (term.norm() < 1e-16) {
        break;
      }
    }
    for (int index = 0; index < squarings; ++index) {
      result = result * result;
    }
    return result;
  }

 private:
  std::array<double, static_cast<std::size_t>(Rows * Cols)> m_values{};
};

template <int Rows, int Cols>
Matrixd<Rows, Cols> operator*(double scalar,
                              const Matrixd<Rows, Cols>& matrix) {
  return matrix * scalar;
}

template <int Size>
using Vectord = Matrixd<Size, 1>;

}  // namespace frc