#include <iostream>
#include <iomanip>
#include <cmath>

// Include the Google Test framework
#include <gtest/gtest.h>

// Include the header we just built
#include "include/CubicSpiral.hh"

// Test case for a standard lane-change maneuver.
// It checks that the optimizer converges and that the final state matches the goal.
TEST(CubicSpiralTest, ConvergenceAndBoundaryConditions) {
    vprim::CubicSpiral spiral;

    vprim::SpatialState start(0.0, 0.0, 0.0, 0.0);
    vprim::SpatialState goal(10.0, 3.0, 0.0, 0.0); 

    // 1. Test Build Process
    bool success = spiral.build(start, goal);
    ASSERT_TRUE(success) << "Optimizer failed to converge for the lane change maneuver.";

    // 2. Validate Trajectory Length
    double length = spiral.get_cost();
    ASSERT_GT(length, 10.0) << "Arc length should be greater than the straight-line distance.";

    // 3. Test Forward Kinematics (Evaluation)
    vprim::SpatialState final_state = spiral.eval(length);
    
    // 4. Assert boundary matching using EXPECT_NEAR for robust floating-point comparison.
    double tolerance = 1e-3;
    EXPECT_NEAR(final_state.x, goal.x, tolerance);
    EXPECT_NEAR(final_state.y, goal.y, tolerance);
    EXPECT_NEAR(final_state.theta, goal.theta, tolerance);
    EXPECT_NEAR(final_state.kappa, goal.kappa, tolerance);
}

// This test builds a more complex trajectory and prints its profile for visual inspection.
// The main assertion is that the build process succeeds.
TEST(CubicSpiralTest, ProfileGeneration) {
    vprim::CubicSpiral spiral;
    vprim::SpatialState start(0.0, 0.0, 0.0, 0.0);
    vprim::SpatialState goal(10.0, 5.0, M_PI/4.0, 0.0); // 45-degree right turn

    bool success = spiral.build(start, goal);
    ASSERT_TRUE(success) << "Failed to build trajectory for profiling.";

    double S = spiral.get_cost();
    int steps = 10;
    double ds = S / steps;

    // Printing to cout is fine within a test for debugging or visualization.
    // GTest will capture the output and display it on test failure.
    std::cout << "--- Trajectory Profile for 45-degree turn ---\n";
    std::cout << std::setw(10) << "s (m)" 
              << std::setw(10) << "x (m)" 
              << std::setw(10) << "y (m)" 
              << std::setw(10) << "theta (rad)" 
              << std::setw(10) << "kappa\n";
    std::cout << std::string(50, '-') << "\n";

    for (int i = 0; i <= steps; ++i) {
        double current_s = i * ds;
        vprim::SpatialState state = spiral.eval(current_s);
        
        std::cout << std::fixed << std::setprecision(4)
                  << std::setw(10) << current_s
                  << std::setw(10) << state.x
                  << std::setw(10) << state.y
                  << std::setw(10) << state.theta
                  << std::setw(10) << state.kappa << "\n";
    }
    std::cout << std::endl; // Add a final newline for clean test output.
}

// The main() function is provided by the GTest_main library,
// so we no longer need to define it ourselves.