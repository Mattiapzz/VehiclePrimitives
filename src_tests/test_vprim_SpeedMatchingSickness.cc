#include <gtest/gtest.h>
#include <cmath>

#include "include/SpeedMatchingSickness.hh"
#include "include/SpeedMatching.hh"

using namespace vprim;

// Regression values established in motion_sickness_filter_minT_notes.md (verified
// against maple2022 CLI execution of filtered_cost.mpl / test_filter_vs_nofilter.mpl,
// and cross-checked in motionsick/sympy_port.py).

TEST(SpeedMatchingSicknessTest, RestToRestMatchesVerifiedTStar) {
    SpeedMatchingSickness sms(1.0);  // w_T = 1
    LongitudinalState start{0.0, 0.0, 0.0, 0.0};
    LongitudinalState goal{100.0, 0.0, 0.0, 0.0};

    ASSERT_TRUE(sms.build(start, goal));
    EXPECT_NEAR(sms.get_T(), 17.8447756690874, 1e-3);
}

TEST(SpeedMatchingSicknessTest, CruiseMatchesVerifiedTStar) {
    SpeedMatchingSickness sms(1.0);
    LongitudinalState start{0.0, 10.0, 0.0, 0.0};
    LongitudinalState goal{100.0, 10.0, 0.0, 0.0};

    ASSERT_TRUE(sms.build(start, goal));
    EXPECT_NEAR(sms.get_T(), 9.371, 1e-2);
}

TEST(SpeedMatchingSicknessTest, BoundaryConditionsMatch) {
    SpeedMatchingSickness sms(1.0);
    LongitudinalState start{0.0, 10.0, 0.0, 0.0};
    LongitudinalState goal{100.0, 10.0, 0.0, 0.0};

    ASSERT_TRUE(sms.build(start, goal));
    const double T = sms.get_T();

    LongitudinalState eval_start = sms.eval(0.0);
    LongitudinalState eval_goal = sms.eval(T);

    const double tol = 1e-6;
    EXPECT_NEAR(eval_start.s, start.s, tol);
    EXPECT_NEAR(eval_start.v, start.v, tol);
    EXPECT_NEAR(eval_start.a, start.a, tol);
    EXPECT_NEAR(eval_goal.s, goal.s, tol);
    EXPECT_NEAR(eval_goal.v, goal.v, tol);
    EXPECT_NEAR(eval_goal.a, goal.a, tol);
}

// Same shape family as SpeedMatching: for a fixed T, the quintic position
// coefficients should match SpeedMatching::build_fixed_time's exactly.
TEST(SpeedMatchingSicknessTest, ShapeMatchesSpeedMatchingForFixedT) {
    LongitudinalState start{0.0, 10.0, 0.0, 0.0};
    LongitudinalState goal{100.0, 10.0, 0.0, 0.0};
    const double T_fixed = 9.371;

    SpeedMatchingSickness sms(1.0);
    ASSERT_TRUE(sms.build(start, goal));

    SpeedMatching sm;
    ASSERT_TRUE(sm.build_fixed_time(start, goal, sms.get_T()));

    for (double t = 0.0; t <= sms.get_T(); t += sms.get_T() / 10.0) {
        LongitudinalState a = sms.eval(t);
        LongitudinalState b = sm.eval(t);
        EXPECT_NEAR(a.s, b.s, 1e-9);
        EXPECT_NEAR(a.v, b.v, 1e-9);
        EXPECT_NEAR(a.a, b.a, 1e-9);
        EXPECT_NEAR(a.j, b.j, 1e-9);
    }
    (void)T_fixed;
}

// a_w(0) should equal q0 filtered through C when q0 != 0, and more generally
// AW(t) should be a smooth, bounded signal (sanity, not a tight numeric check).
TEST(SpeedMatchingSicknessTest, FilteredAccelerationIsBoundedAndSmooth) {
    SpeedMatchingSickness sms(1.0);
    LongitudinalState start{0.0, 0.0, 0.0, 0.0};
    LongitudinalState goal{100.0, 0.0, 0.0, 0.0};
    ASSERT_TRUE(sms.build(start, goal));

    const double T = sms.get_T();
    double max_abs_aw = 0.0;
    double prev = sms.AW(0.0);
    for (int i = 1; i <= 50; ++i) {
        const double t = T * i / 50.0;
        const double aw = sms.AW(t);
        EXPECT_TRUE(std::isfinite(aw));
        max_abs_aw = std::max(max_abs_aw, std::abs(aw));
        EXPECT_LT(std::abs(aw - prev), 5.0) << "a_w should not jump discontinuously";
        prev = aw;
    }
    EXPECT_GT(max_abs_aw, 0.0);
}
