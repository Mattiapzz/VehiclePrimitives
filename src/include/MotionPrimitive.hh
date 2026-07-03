#pragma once

#ifndef VPRIM_MOTION_PRIMITIVE_HH
#define VPRIM_MOTION_PRIMITIVE_HH

namespace vprim {

    // T_State is the physical state (e.g., Pose, VelocityProfile)
    // T_Param is the evaluation variable (defaults to double for time 't' or arc length 's')
    template <typename T_State, typename T_Param = double>
    class MotionPrimitive {
    public:
        MotionPrimitive() = default;
        virtual ~MotionPrimitive() = default;
        // copy and move constructors/assignment operators
        MotionPrimitive(const MotionPrimitive&) = default;
        MotionPrimitive& operator=(const MotionPrimitive&) = default;
        MotionPrimitive(MotionPrimitive&&) = default;
        MotionPrimitive& operator=(MotionPrimitive&&) = default;

        // Force derived classes to implement a solver for the transient
        // Returns true if a valid primitive was found, false if infeasible
        virtual bool build(const T_State& initial_state, const T_State& goal_state) = 0;

        // Force derived classes to return a state at a given parameter
        virtual T_State eval(T_Param p) const = 0;
        
        // Optional but highly recommended: get the total cost or length of the primitive
        virtual double get_cost() const = 0;
    };

} // namespace vprim

#endif // VPRIM_MOTION_PRIMITIVE_HH
