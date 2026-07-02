#pragma once    

#ifndef VPRIM_SPEED_MATCHING_HH
#define VPRIM_SPEED_MATCHING_HH


#include <array>
#include <functional>
#include "MotionPrimitive.hh"


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

    

 

} // namespace vprim

#endif // VPRIM_SPEED_MATCHING_HH 
