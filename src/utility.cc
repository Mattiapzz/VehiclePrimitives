
#include "include/utility.hh"

namespace vprim {

  size_t Polynomial::degree() const { 
      return c_.empty() ? 0 : c_.size() - 1; 
  }

  // Horner's Method: O(N) complexity, extremely stable, no pow() calls
  double Polynomial::eval(double x) const {
    if (c_.empty()) return 0.0;
    //
    double result = c_.back();
    for (int i = static_cast<int>(c_.size()) - 2; i >= 0; --i) {
      result = result * x + c_[i];
    }
    return result;
  }

  // Returns a completely new Polynomial representing the mathematical derivative
  Polynomial Polynomial::derivative() const {
    if (c_.size() <= 1) return Polynomial{0.0}; // Derivative of a constant is 0
    //
    Polynomial deriv(degree() - 1);
    for (size_t i = 1; i < c_.size(); ++i) {
      deriv.c_[i - 1] = c_[i] * i;
    }
    return deriv;
  }

  // Generates the n-th derivative recursively
  Polynomial Polynomial::derivative(int n) const {
    Polynomial result = *this;
    for (int i = 0; i < n; ++i) {
      result = result.derivative();
    }
    return result;
  }

} // namespace vprim