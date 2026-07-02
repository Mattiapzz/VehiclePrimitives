#pragma once

#ifndef VPRIM_UTILITY_HH
#define VPRIM_UTILITY_HH


#include <vector>
#include <initializer_list>
#include <cstddef>

namespace vprim {

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

        size_t degree() const;

        // Horner's Method: O(N) complexity, extremely stable, no pow() calls
        double eval(double x) const;

        // Returns a completely new Polynomial representing the mathematical derivative
        Polynomial derivative() const;

        // Generates the n-th derivative recursively
        Polynomial derivative(int n) const;
    };

} // namespace vprim

#endif // VPRIM_UTILITY_HH
