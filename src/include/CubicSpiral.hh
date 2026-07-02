/*
This file serves as (more than) dummy example code for primitives. It is not intended to be used in production, but rather to illustrate how to implement a new primitive type.
*/

#pragma once    

#ifndef VPRIM_CUBIC_SPIRAL_HH
#define VPRIM_CUBIC_SPIRAL_HH


#include <array>
#include <functional>
#include "MotionPrimitive.hh"


namespace vprim {

    // A lightweight struct for spatial paths (no dependency on Eigen)
    struct SpatialState {
        double x{0.0};      // x position
        double y{0.0};      // y position
        double theta{0.0};  // heading angle
        double kappa{0.0};  // curvature

        SpatialState() = default;

        // Constructor to initialize with specific values
        SpatialState(double _x, double _y, double _theta, double _kappa) : x(_x), y(_y), theta(_theta), kappa(_kappa) {}
    };

    // A spatial primitive evaluated over arc length 's'
    class CubicSpiral : public MotionPrimitive<SpatialState, double> {
    private:

        double length_ = 0.0;
        std::array<double, 4> p_ = {0}; // Internal lightweight array
        SpatialState start_state_;
        SpatialState goal_state_;

        // Lightweight numerical integrator using Simpson's 1/3 Rule
        double integrate_simpson(std::function<double(double)> func, double a, double b, int n_intervals = 16) const;

        // Analytical heading evaluation
        [[nodiscard]] double eval_yaw(double s) const;

        std::array<double, 4> solve_4x4(std::array<std::array<double, 4>, 4> A, std::array<double, 4> b) const;

    public:
        CubicSpiral() = default;
        ~CubicSpiral() override = default;


        // copy and move constructors/assignment operators 
        CubicSpiral(const CubicSpiral&) = default;
        CubicSpiral& operator=(const CubicSpiral&) = default;
        CubicSpiral(CubicSpiral&&) = default;
        CubicSpiral& operator=(CubicSpiral&&) = default;

        bool build(const SpatialState& start, const SpatialState& goal) override;

        [[nodiscard]] SpatialState eval(double s) const override;

        [[nodiscard]] double get_cost() const override;
    };

} // namespace vprim

#endif // VPRIM_CUBIC_SPIRAL_HH 
