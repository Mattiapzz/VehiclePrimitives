#pragma once    

#ifndef VPRIM_SPEED_MATCHING_HH
#define VPRIM_SPEED_MATCHING_HH



#include "MotionPrimitive.hh"

#include <cmath>
#include <algorithm>
#include "include/utility.hh"


namespace vprim {

    // Extended state to support jerk, matching your original trajectory polynomials
    struct LongitudinalState {
        double s{0.0};
        double v{0.0};
        double a{0.0};
        double j{0.0}; // Jerk

        LongitudinalState() = default;
        LongitudinalState(double _s, double _v, double _a, double _j) : s(_s), v(_v), a(_a), j(_j) {}
    };



    class SpeedMatching : public MotionPrimitive<LongitudinalState, double> {
    private:
        LongitudinalState start_state_;
        double wT_{1.0}; // Weight on time optimization
        double T_{0.0};  // Optimal travel time
        double cost_{0.0};
        bool is_valid_{false};

        // Standalone polynomial objects for each kinematic derivative
        Polynomial poly_s_;
        Polynomial poly_v_;
        Polynomial poly_a_;
        Polynomial poly_j_;

        // Validates that the velocity never drops below zero during the maneuver
        [[nodiscard]] bool check_velocity() const ;

    public:
        SpeedMatching() = default;
        explicit SpeedMatching(double weight_time) : wT_(weight_time) {}

        void set_time_weight(double wT) { wT_ = wT; }
        [[nodiscard]] double get_T() const { return T_; }
        [[nodiscard]] bool is_ok() const { return is_valid_; }

        bool build(const LongitudinalState& start, const LongitudinalState& goal) override;

        [[nodiscard]] LongitudinalState eval(double t) const override;

        [[nodiscard]] double get_cost() const override { return cost_; }

        [[nodiscard]] inline double T() const { return T_; }

        [[nodiscard]] inline double S(double const t)  { return ( this->poly_s_.eval(std::clamp(t, 0.0, this->T_)) ); }
        [[nodiscard]] inline double V(double const t)  { return ( this->poly_v_.eval(std::clamp(t, 0.0, this->T_)) ); }
        [[nodiscard]] inline double A(double const t)  { return ( this->poly_a_.eval(std::clamp(t, 0.0, this->T_)) ); }
        [[nodiscard]] inline double J(double const t)  { return ( this->poly_j_.eval(std::clamp(t, 0.0, this->T_)) ); }

        

    };





 

} // namespace vprim

#endif // VPRIM_SPEED_MATCHING_HH 
