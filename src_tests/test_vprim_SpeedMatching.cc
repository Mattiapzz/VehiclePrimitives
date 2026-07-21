#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "include/SpeedMatching.hh" // Adjust if your include path differs

using namespace vprim;

class SpeedMatchingTest : public ::testing::Test {
protected:
    SpeedMatching sm;
    
    // Boundary conditions specified:
    // Start: 10 m/s, 1.0 m/s^2 at s = 0m
    LongitudinalState start{0.0, 10.0, 1.0, 0.0}; 
    
    // Goal: 20 m/s, -1.0 m/s^2 at s = 50m
    LongitudinalState goal{50.0, 20.0, -1.0, 0.0}; 
};

// 1. Test Optimizer Convergence and Feasibility
TEST_F(SpeedMatchingTest, OptimizerConverges) {
    bool success = sm.build(start, goal);
    
    ASSERT_TRUE(success) << "The time-optimization solver failed to converge.";
    
    double T = sm.get_T();
    EXPECT_GT(T, 0.0) << "Calculated time must be strictly positive.";
    EXPECT_LT(T, 10.0) << "Calculated time should be reasonable for a 50m maneuver.";
}

// 2. Test Exact Boundary Condition Matching
TEST_F(SpeedMatchingTest, BoundaryConditionsMatch) {
    ASSERT_TRUE(sm.build(start, goal));

    double T = sm.get_T();
    
    // Evaluate at t = 0
    LongitudinalState eval_start = sm.eval(0.0);
    
    // Evaluate at t = T
    LongitudinalState eval_goal = sm.eval(T);

    double tol = 1e-3;

    // Check Initial State
    EXPECT_NEAR(eval_start.s, start.s, tol);
    EXPECT_NEAR(eval_start.v, start.v, tol);
    EXPECT_NEAR(eval_start.a, start.a, tol);

    // Check Final State
    EXPECT_NEAR(eval_goal.s, goal.s, tol);
    EXPECT_NEAR(eval_goal.v, goal.v, tol);
    EXPECT_NEAR(eval_goal.a, goal.a, tol);
}

// 3. Edge Case: Straight Constant Velocity
TEST_F(SpeedMatchingTest, ConstantVelocityEdgeCase) {
    LongitudinalState cv_start{0.0, 15.0, 0.0, 0.0};
    LongitudinalState cv_goal{45.0, 15.0, 0.0, 0.0};

    sm.set_time_weight(0.0);
    
    ASSERT_TRUE(sm.build(cv_start, cv_goal));
    
    // Time should be exactly distance / velocity = 45 / 15 = 3.0 seconds
    EXPECT_NEAR(sm.get_T(), 3.0, 1e-3);
    
    // Acceleration and Jerk should be effectively zero throughout
    LongitudinalState mid_state = sm.eval(1.5);
    EXPECT_NEAR(mid_state.a, 0.0, 1e-3);
    EXPECT_NEAR(mid_state.j, 0.0, 1e-3);
}

// 4. Utility: Print the trajectory profile for the 10->20 m/s maneuver
TEST_F(SpeedMatchingTest, PrintTrajectoryProfile) {
    sm.set_time_weight(1000.0);
    ASSERT_TRUE(sm.build(start, goal));


    double T = sm.get_T();
    int steps = 10;
    double dt = T / steps;

    std::cout << "\n--- SpeedMatching Profile: 10m/s -> 20m/s in 50m ---\n";
    std::cout << "Optimal Time (T): " << T << " s\n";
    std::cout << std::string(60, '-') << "\n";
    std::cout << std::setw(10) << "t (s)" 
              << std::setw(10) << "s (m)" 
              << std::setw(10) << "v (m/s)" 
              << std::setw(12) << "a (m/s^2)" 
              << std::setw(12) << "j (m/s^3)\n";
    std::cout << std::string(60, '-') << "\n";

    for (int i = 0; i <= steps; ++i) {
        double current_t = i * dt;
        LongitudinalState state = sm.eval(current_t);
        
        std::cout << std::fixed << std::setprecision(4)
                  << std::setw(10) << current_t
                  << std::setw(10) << state.s
                  << std::setw(10) << state.v
                  << std::setw(12) << state.a
                  << std::setw(12) << state.j << "\n";
    }
    std::cout << std::string(60, '-') << "\n\n";
}


// 5. build_fixed_time reproduces build()'s own solved T exactly (same boundary
//    conditions, T supplied instead of searched for -> identical trajectory).
TEST_F(SpeedMatchingTest, FixedTimeMatchesOptimizerSolution) {
    SpeedMatching sm_free;
    ASSERT_TRUE(sm_free.build(start, goal));
    double T_opt = sm_free.get_T();

    SpeedMatching sm_fixed;
    ASSERT_TRUE(sm_fixed.build_fixed_time(start, goal, T_opt));

    EXPECT_NEAR(sm_fixed.get_T(), T_opt, 1e-9);

    double tol = 1e-6;
    for (double t = 0.0; t <= T_opt; t += T_opt / 10.0) {
        LongitudinalState a = sm_free.eval(t);
        LongitudinalState b = sm_fixed.eval(t);
        EXPECT_NEAR(a.s, b.s, tol);
        EXPECT_NEAR(a.v, b.v, tol);
        EXPECT_NEAR(a.a, b.a, tol);
        EXPECT_NEAR(a.j, b.j, tol);
    }
}

// 6. build_fixed_time with an arbitrary, non-optimal T still satisfies the
//    boundary conditions (this is the actual point of the feature: caller
//    picks T, e.g. to match another maneuver's duration).
TEST_F(SpeedMatchingTest, FixedTimeArbitraryDurationMatchesBoundaryConditions) {
    double T_arbitrary = 6.0; // deliberately not the time-optimal value
    ASSERT_TRUE(sm.build_fixed_time(start, goal, T_arbitrary));

    EXPECT_NEAR(sm.get_T(), T_arbitrary, 1e-9);

    double tol = 1e-3;
    LongitudinalState eval_start = sm.eval(0.0);
    LongitudinalState eval_goal = sm.eval(T_arbitrary);

    EXPECT_NEAR(eval_start.s, start.s, tol);
    EXPECT_NEAR(eval_start.v, start.v, tol);
    EXPECT_NEAR(eval_start.a, start.a, tol);
    EXPECT_NEAR(eval_goal.s, goal.s, tol);
    EXPECT_NEAR(eval_goal.v, goal.v, tol);
    EXPECT_NEAR(eval_goal.a, goal.a, tol);
}

// 7. build_fixed_time rejects a non-positive duration instead of solving
//    garbage.
TEST_F(SpeedMatchingTest, FixedTimeRejectsNonPositiveDuration) {
    EXPECT_FALSE(sm.build_fixed_time(start, goal, 0.0));
    EXPECT_FALSE(sm.build_fixed_time(start, goal, -1.0));
}

// 8. SN (snap) and CR (crackle) match finite-difference derivatives of J (jerk)
//    at an interior point, since they are just poly_j_ differentiated once
//    (SN) or twice (CR).
TEST_F(SpeedMatchingTest, SnapAndCrackleAreDerivativesOfJerk) {
    ASSERT_TRUE(sm.build(start, goal));
    double T = sm.get_T();
    double t_mid = T / 2.0;
    double h = 1e-5;

    double j_plus = sm.J(t_mid + h);
    double j_minus = sm.J(t_mid - h);
    double sn_finite_diff = (j_plus - j_minus) / (2.0 * h);

    EXPECT_NEAR(sm.SN(t_mid), sn_finite_diff, 1e-3);

    double sn_plus = sm.SN(t_mid + h);
    double sn_minus = sm.SN(t_mid - h);
    double cr_finite_diff = (sn_plus - sn_minus) / (2.0 * h);

    EXPECT_NEAR(sm.CR(t_mid), cr_finite_diff, 1e-3);
}

// 9. Both SN and CR are identically zero for the constant-velocity edge case
//    (a straight-line quintic with only linear s(t) collapses every
//    derivative above velocity to zero).
TEST_F(SpeedMatchingTest, SnapAndCrackleAreZeroForConstantVelocity) {
    LongitudinalState cv_start{0.0, 15.0, 0.0, 0.0};
    LongitudinalState cv_goal{45.0, 15.0, 0.0, 0.0};

    sm.set_time_weight(0.0);
    ASSERT_TRUE(sm.build(cv_start, cv_goal));

    EXPECT_NEAR(sm.SN(1.5), 0.0, 1e-3);
    EXPECT_NEAR(sm.CR(1.5), 0.0, 1e-3);
}

TEST_F(SpeedMatchingTest, PrintTrajectoryProfileNoMinT) {
    sm.set_time_weight(0.0);  
    ASSERT_TRUE(sm.build(start, goal));

    double T = sm.get_T();
    int steps = 10;
    double dt = T / steps;

    std::cout << "\n--- SpeedMatching Profile (No Min Time): 10m/s -> 20m/s in 50m ---\n";
    std::cout << "Optimal Time (T): " << T << " s\n";
    std::cout << std::string(60, '-') << "\n";
    std::cout << std::setw(10) << "t (s)" 
              << std::setw(10) << "s (m)" 
              << std::setw(10) << "v (m/s)" 
              << std::setw(12) << "a (m/s^2)" 
              << std::setw(12) << "j (m/s^3)\n";
    std::cout << std::string(60, '-') << "\n";  

    for (int i = 0; i <= steps; ++i) {
        double current_t = i * dt;
        LongitudinalState state = sm.eval(current_t);
        
        std::cout << std::fixed << std::setprecision(4)
                  << std::setw(10) << current_t
                  << std::setw(10) << state.s
                  << std::setw(10) << state.v
                  << std::setw(12) << state.a
                  << std::setw(12) << state.j << "\n";
    }
    std::cout << std::string(60, '-') << "\n\n";
}
