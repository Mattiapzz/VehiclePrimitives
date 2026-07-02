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
