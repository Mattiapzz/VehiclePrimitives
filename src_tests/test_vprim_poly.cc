#include <gtest/gtest.h>
#include "include/utility.hh" // Adjust path if necessary based on your includes

using namespace vprim;

// 1. Test Constructors and Basic Properties
TEST(PolynomialTest, Initialization) {
    // Empty polynomial
    Polynomial p_empty;
    EXPECT_EQ(p_empty.degree(), 0);
    EXPECT_DOUBLE_EQ(p_empty.eval(5.0), 0.0);

    // Initialized by degree (creates a polynomial of degree 3, initialized to 0.0)
    Polynomial p_deg(3);
    EXPECT_EQ(p_deg.degree(), 3);
    EXPECT_DOUBLE_EQ(p_deg[3], 0.0);

    // Initializer list constructor: f(x) = 1 + 2x + 3x^2
    Polynomial p_list = {1.0, 2.0, 3.0};
    EXPECT_EQ(p_list.degree(), 2);
    EXPECT_DOUBLE_EQ(p_list[0], 1.0);
    EXPECT_DOUBLE_EQ(p_list[1], 2.0);
    EXPECT_DOUBLE_EQ(p_list[2], 3.0);
}

// 2. Test Horner's Method Evaluation
TEST(PolynomialTest, Evaluation) {
    // f(x) = 5 - 3x + 2x^2
    Polynomial p = {5.0, -3.0, 2.0};

    // f(0) = 5
    EXPECT_DOUBLE_EQ(p.eval(0.0), 5.0);

    // f(1) = 5 - 3(1) + 2(1)^2 = 4
    EXPECT_DOUBLE_EQ(p.eval(1.0), 4.0);

    // f(2.5) = 5 - 3(2.5) + 2(2.5)^2 = 5 - 7.5 + 12.5 = 10
    EXPECT_DOUBLE_EQ(p.eval(2.5), 10.0);

    // f(-2) = 5 - 3(-2) + 2(-2)^2 = 5 + 6 + 8 = 19
    EXPECT_DOUBLE_EQ(p.eval(-2.0), 19.0);
}

// 3. Test First Derivative Generation
TEST(PolynomialTest, FirstDerivative) {
    // f(x) = 10 + 4x - 5x^2 + x^3
    Polynomial p = {10.0, 4.0, -5.0, 1.0};
    
    // f'(x) = 4 - 10x + 3x^2
    Polynomial p_dot = p.derivative();

    EXPECT_EQ(p_dot.degree(), 2);
    EXPECT_DOUBLE_EQ(p_dot[0], 4.0);
    EXPECT_DOUBLE_EQ(p_dot[1], -10.0);
    EXPECT_DOUBLE_EQ(p_dot[2], 3.0);

    // Evaluate the derivative at x = 2
    // f'(2) = 4 - 10(2) + 3(2)^2 = 4 - 20 + 12 = -4
    EXPECT_DOUBLE_EQ(p_dot.eval(2.0), -4.0);
}

// 4. Test Recursive/N-th Derivative
TEST(PolynomialTest, NthDerivative) {
    // Kinematic application: spatial position s(t)
    // s(t) = 0 + 10t + 0.5(2)t^2 + (1/6)(3)t^3
    // s(t) = 0 + 10t + 1t^2 + 0.5t^3
    Polynomial s = {0.0, 10.0, 1.0, 0.5};

    // v(t) = s'(t) = 10 + 2t + 1.5t^2
    Polynomial v = s.derivative(1);
    EXPECT_EQ(v.degree(), 2);
    EXPECT_DOUBLE_EQ(v[0], 10.0);
    EXPECT_DOUBLE_EQ(v[1], 2.0);
    EXPECT_DOUBLE_EQ(v[2], 1.5);

    // a(t) = s''(t) = 2 + 3t
    Polynomial a = s.derivative(2);
    EXPECT_EQ(a.degree(), 1);
    EXPECT_DOUBLE_EQ(a[0], 2.0);
    EXPECT_DOUBLE_EQ(a[1], 3.0);

    // j(t) = s'''(t) = 3
    Polynomial j = s.derivative(3);
    EXPECT_EQ(j.degree(), 0);
    EXPECT_DOUBLE_EQ(j[0], 3.0);
    EXPECT_DOUBLE_EQ(j.eval(100.0), 3.0); // Constant jerk
}

// 5. Edge Cases
TEST(PolynomialTest, EdgeCases) {
    // Constant polynomial f(x) = 42
    Polynomial p_const = {42.0};
    
    // Derivative should be f'(x) = 0
    Polynomial p_zero = p_const.derivative();
    EXPECT_EQ(p_zero.degree(), 0);
    EXPECT_DOUBLE_EQ(p_zero.eval(10.0), 0.0);

    // Over-deriving (4th derivative of a cubic)
    Polynomial cubic = {1.0, 1.0, 1.0, 1.0};
    Polynomial flatline = cubic.derivative(5);
    EXPECT_EQ(flatline.degree(), 0);
    EXPECT_DOUBLE_EQ(flatline.eval(0.0), 0.0);
}