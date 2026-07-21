#pragma once

#ifndef VPRIM_UTILITY_HH
#define VPRIM_UTILITY_HH


#include <vector>
#include <initializer_list>
#include <cstddef>
#include <optional>

namespace vprim {

  struct RootSolverOptions {
      int max_iter = 50;
      double alpha = 1.0;
      double step_tolerance = 1e-6;
      double func_tolerance = 1e-6;
  };
  //
  // And update the function signature to:
  // [[nodiscard]] std::optional<double> closest_root(
  //   double initial_guess,
  //   const RootSolverOptions& options = {}
  // ) const;

  class Polynomial {
  private:
    // Coefficients stored as: c[0] + c[1]*t + c[2]*t^2 + ... + c[n]*t^n
    std::vector<double> c_;

  public:
    Polynomial() = default;

    // Construct with a specific degree (initializes all to 0.0)
    explicit Polynomial(int degree) : c_(degree + 1, 0.0) {}

    // Clean C++11 initializer: Polynomial p = {c0, c1, c2, c3};
    Polynomial(std::initializer_list<double> coeffs) : c_(coeffs) {}

    // Accessors
    double& operator[](size_t i) { return c_[i]; }
    const double& operator[](size_t i) const { return c_[i]; }

    [[nodiscard]] size_t degree() const;

    // Horner's Method: O(N) complexity, extremely stable, no pow() calls
    [[nodiscard]] double eval(double x) const;

    // Returns a completely new Polynomial representing the mathematical derivative
    [[nodiscard]] Polynomial derivative() const;

    // Generates the n-th derivative recursively
    [[nodiscard]] Polynomial derivative(int n) const;

    [[nodiscard]] std::optional<double> closest_root(
      double initial_guess,
      const RootSolverOptions& options = {}
    ) const;
  };




} // namespace vprim

#endif // VPRIM_UTILITY_HH
