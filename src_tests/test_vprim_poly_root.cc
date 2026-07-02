#include <gtest/gtest.h>
#include <iostream>
#include "include/utility.hh" // Adjust path if necessary based on your includes

using namespace vprim;

// 1. Test Simple Quadratic: f(x) = x^2
TEST(ClosestRootTest, SimpleQuadratic) {
    // The polynomial is x^2, which has a double root at x=0.
    Polynomial p = {0.0, 0.0, 1.0}; // 0 + 0x + 1x^2
    
    // For multiple roots, Newton's method converges slowly (linearly), and standard
    // termination criteria (e.g., a small step size) can trigger prematurely.
    // We must force the solver to use a much smaller internal tolerance to get
    // an accurate result. The key is to use a very small *step tolerance*, as
    // the function value tolerance can be misleading near a flat minimum.
    RootSolverOptions opts;
    opts.max_iter = 100;
    opts.step_tolerance = 1e-9; // Use a tight tolerance on the step size
    opts.func_tolerance = 0;    // Effectively disable the function value check

    auto root = p.closest_root(0.5, opts);
    
    ASSERT_TRUE(root.has_value()) << "Solver failed to converge for x^2";
    EXPECT_NEAR(root.value(), 0.0, 1e-5);
}

// 2. Test Quadratic with two roots: f(x) = x^2 - 1
TEST(ClosestRootTest, QuadraticTwoRoots) {
    Polynomial p = {-1.0, 0.0, 1.0}; // -1 + 0x + 1x^2

    // Positive guess should find the positive root (+1.0)
    auto root_pos = p.closest_root(0.5); // Use default options
    ASSERT_TRUE(root_pos.has_value());
    EXPECT_NEAR(root_pos.value(), 1.0, 1e-5);

    // Negative guess should find the negative root (-1.0)
    auto root_neg = p.closest_root(-0.5); // Use default options
    ASSERT_TRUE(root_neg.has_value());
    EXPECT_NEAR(root_neg.value(), -1.0, 1e-5);
}

// 3. Test 6th Degree Polynomial with 6 distinct roots
// f(x) = (x-1)*x*(x+2)*(x-3)*(x+4)*(x-5)
// f(x) = x^6 - 3x^5 - 23x^4 + 51x^3 + 94x^2 - 120x
// Roots expected: -4, -2, 0, 1, 3, 5
TEST(ClosestRootTest, SixthDegreePolynomial) {
    // Coefficients ordered from x^0 up to x^6
    Polynomial p = {0.0, -120.0, 94.0, 51.0, -23.0, -3.0, 1.0};

    struct TestCase {
        double guess;
        double expected_root;
    };

    // Test starting close to each root (0.1 offset)
    std::vector<TestCase> cases = {
        {-4.1, -4.0},
        {-1.9, -2.0},
        { 0.1,  0.0},
        { 1.1,  1.0},
        { 2.9,  3.0},
        { 5.1,  5.0}
    };

    for (const auto& test_case : cases) {
        auto root = p.closest_root(test_case.guess); // Use default options
        
        ASSERT_TRUE(root.has_value()) 
            << "Solver failed to converge from guess: " << test_case.guess;
        
        EXPECT_NEAR(root.value(), test_case.expected_root, 1e-5) 
            << "Solver converged to wrong root from guess: " << test_case.guess;
    }
}