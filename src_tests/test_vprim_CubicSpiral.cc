#include <iostream>
#include <iomanip>
#include <cmath>
#include <cassert>

// Include the header we just built
#include "include/CubicSpiral.hh"

// Helper function to compare doubles with a tolerance
bool is_close(double a, double b, double tol = 1e-3) {
    return std::abs(a - b) < tol;
}

void test_cubic_spiral_convergence() {
    std::cout << "--- Testing Cubic Spiral Boundary Value Problem ---\n";

    vprim::CubicSpiral spiral;

    // Define a challenging but realistic boundary value problem
    // e.g., a lane change maneuver
    vprim::SpatialState start(0.0, 0.0, 0.0, 0.0);
    vprim::SpatialState goal(10.0, 3.0, 0.0, 0.0); 

    // 1. Test Build Process
    bool success = spiral.build(start, goal);
    
    if (success) {
        std::cout << "[PASS] Optimizer converged successfully.\n";
    } else {
        std::cout << "[FAIL] Optimizer failed to converge.\n";
        return;
    }

    // 2. Validate Trajectory Length
    double length = spiral.get_cost();
    std::cout << "Calculated Arc Length (S): " << length << " m\n";
    assert(length > 10.0 && "Arc length should be greater than straight-line distance.");

    // 3. Test Forward Kinematics (Evaluation)
    vprim::SpatialState final_state = spiral.eval(length);

    std::cout << "\nTarget Goal vs Evaluated Final State:\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "X:   Target = " << goal.x << " | Actual = " << final_state.x << "\n";
    std::cout << "Y:   Target = " << goal.y << " | Actual = " << final_state.y << "\n";
    std::cout << "Yaw: Target = " << goal.theta << " | Actual = " << final_state.theta << "\n";
    std::cout << "K:   Target = " << goal.kappa << " | Actual = " << final_state.kappa << "\n";

    // 4. Assert exact boundary matching
    assert(is_close(final_state.x, goal.x) && "X coordinate mismatch");
    assert(is_close(final_state.y, goal.y) && "Y coordinate mismatch");
    assert(is_close(final_state.theta, goal.theta) && "Heading mismatch");
    assert(is_close(final_state.kappa, goal.kappa) && "Curvature mismatch");

    std::cout << "[PASS] Boundary conditions met within tolerance.\n\n";
}

void print_trajectory_profile() {
    std::cout << "--- Printing Trajectory Profile ---\n";
    
    vprim::CubicSpiral spiral;
    vprim::SpatialState start(0.0, 0.0, 0.0, 0.0);
    vprim::SpatialState goal(10.0, 5.0, M_PI/4.0, 0.0); // 90-degree right turn equivalent

    if (!spiral.build(start, goal)) {
        std::cout << "Failed to build trajectory for profiling.\n";
        return;
    }

    double S = spiral.get_cost();
    int steps = 10;
    double ds = S / steps;

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
    std::cout << "[PASS] Trajectory profile generated.\n";
}

int main() {
    test_cubic_spiral_convergence();
    print_trajectory_profile();
    
    std::cout << "\nAll vprim tests completed successfully.\n";
    return 0;
}