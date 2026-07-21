#include "include/CubicSpiral.hh"

#include <cmath>

namespace vprim {

  double CubicSpiral::eval_yaw(double s) const {
    return start_state_.theta + 
            p_[0] * s + 
            (p_[1] / 2.0) * std::pow(s, 2) + 
            (p_[2] / 3.0) * std::pow(s, 3) + 
            (p_[3] / 4.0) * std::pow(s, 4);
  };

  double CubicSpiral::integrate_simpson(std::function<double(double)> func, double a, double b, int n_intervals) const {
    if (n_intervals % 2 != 0) ++n_intervals; // Must be even
    double h = (b - a) / n_intervals;
    double sum = func(a) + func(b);

    for (int i = 1; i < n_intervals; i += 2) sum += 4.0 * func(a + i * h);
    for (int i = 2; i < n_intervals - 1; i += 2) sum += 2.0 * func(a + i * h);

    return (h / 3.0) * sum;
  };


  std::array<double, 4> CubicSpiral::solve_4x4(std::array<std::array<double, 4>, 4> A, std::array<double, 4> b) const {
    std::array<double, 4> x = {0};
    int n = 4;

    for (int i = 0; i < n; i++) {
      // 1. Partial Pivoting for numerical stability
      double max_el = std::abs(A[i][i]);
      int max_row = i;
      for (int k = i + 1; k < n; k++) {
          if (std::abs(A[k][i]) > max_el) {
              max_el = std::abs(A[k][i]);
              max_row = k;
          }
      }

      // Swap rows in A and b
      for (int k = i; k < n; k++) std::swap(A[max_row][k], A[i][k]);
      std::swap(b[max_row], b[i]);

      // Prevent division by zero if the matrix is singular
      if (std::abs(A[i][i]) < 1e-9) return x; 

      // 2. Eliminate entries below the pivot
      for (int k = i + 1; k < n; k++) {
          double factor = -A[k][i] / A[i][i];
          for (int j = i; j < n; j++) {
              if (i == j) A[k][j] = 0;
              else A[k][j] += factor * A[i][j];
          }
          b[k] += factor * b[i];
      }
    }

    // 3. Back substitution
    for (int i = n - 1; i >= 0; i--) {
      x[i] = b[i];
      for (int j = i + 1; j < n; j++) x[i] -= A[i][j] * x[j];
      x[i] = x[i] / A[i][i];
    }
    return x;
  }


  bool CubicSpiral::build(const SpatialState& start, const SpatialState& goal) {
    start_state_ = start;
    p_[0] = start.kappa; 

    int max_iter = 50;
    double tolerance = 1e-4;
    double delta = 1e-5; // Perturbation for finite differences
    
    // Initial guess: S = Euclidean distance, straight line parameters
    double S = std::hypot(goal.x - start.x, goal.y - start.y);
    p_[1] = 0.0; p_[2] = 0.0; p_[3] = 0.0;

    // Lambda to compute the error residual vector for a given state guess
    auto compute_residuals = [&](double dp1, double dp2, double dp3, double dS) -> std::array<double, 4> {
        // Backup current state
        double orig_p1 = p_[1], orig_p2 = p_[2], orig_p3 = p_[3];
        
        // Apply test state
        p_[1] = dp1; p_[2] = dp2; p_[3] = dp3;
        SpatialState current = eval(dS);
        
        // Restore actual state
        p_[1] = orig_p1; p_[2] = orig_p2; p_[3] = orig_p3;

        return {
            current.x - goal.x,
            current.y - goal.y,
            current.theta - goal.theta,
            current.kappa - goal.kappa
        };
    };

    for (int iter = 0; iter < max_iter; ++iter) {
        // Step 1 & 2: Evaluate current residuals
        std::array<double, 4> F = compute_residuals(p_[1], p_[2], p_[3], S);

        // Check for convergence
        if (std::abs(F[0]) < tolerance && std::abs(F[1]) < tolerance && 
            std::abs(F[2]) < tolerance && std::abs(F[3]) < tolerance) {
            length_ = S;
            return true;
        }

        // Step 3: Compute 4x4 Jacobian via finite differences
        std::array<std::array<double, 4>, 4> J;
        
        // Perturb p1 (Column 0)
        auto F_p1 = compute_residuals(p_[1] + delta, p_[2], p_[3], S);
        for(int i = 0; i < 4; ++i) J[i][0] = (F_p1[i] - F[i]) / delta;

        // Perturb p2 (Column 1)
        auto F_p2 = compute_residuals(p_[1], p_[2] + delta, p_[3], S);
        for(int i = 0; i < 4; ++i) J[i][1] = (F_p2[i] - F[i]) / delta;

        // Perturb p3 (Column 2)
        auto F_p3 = compute_residuals(p_[1], p_[2], p_[3] + delta, S);
        for(int i = 0; i < 4; ++i) J[i][2] = (F_p3[i] - F[i]) / delta;

        // Perturb S (Column 3)
        auto F_S = compute_residuals(p_[1], p_[2], p_[3], S + delta);
        for(int i = 0; i < 4; ++i) J[i][3] = (F_S[i] - F[i]) / delta;

        // Step 4: Solve J * delta_x = -F
        std::array<double, 4> minus_F = {-F[0], -F[1], -F[2], -F[3]};
        std::array<double, 4> step = solve_4x4(J, minus_F);

        // If the solver returned exact zeros, the Jacobian is singular. BVP failed.
        if (step[0] == 0.0 && step[1] == 0.0 && step[2] == 0.0 && step[3] == 0.0) {
            return false; 
        }

        // Step 5: Update parameters
        p_[1] += step[0];
        p_[2] += step[1];
        p_[3] += step[2];
        S += step[3];
        
        // Arc length cannot be negative
        if (S < 0) S = std::abs(S);
    }

    return false; // Failed to converge within max_iter
  }

  SpatialState CubicSpiral::eval(double s) const {
    SpatialState state;
    state.kappa = p_[0] + p_[1] * s + p_[2] * std::pow(s, 2) + p_[3] * std::pow(s, 3);
    state.theta = eval_yaw(s);

    // Numerical integration for x and y
    auto cos_func = [this](double tau) { return std::cos(this->eval_yaw(tau)); };
    auto sin_func = [this](double tau) { return std::sin(this->eval_yaw(tau)); };

    state.x = start_state_.x + integrate_simpson(cos_func, 0, s);
    state.y = start_state_.y + integrate_simpson(sin_func, 0, s);

    return state;
  }

  double CubicSpiral::get_cost() const {
      return length_;
  }

} // namespace vprim