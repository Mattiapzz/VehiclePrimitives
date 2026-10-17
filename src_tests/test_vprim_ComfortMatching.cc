#include <gtest/gtest.h>
#include <cmath>

#include "include/ComfortMatching.hh"
#include "include/SpeedMatching.hh"

using namespace vprim;

// Regression values and design rationale: see src/include/ComfortMatching.hh
// and motion_sickness_filter_minT_notes.md. The w_a=0 case is the key
// cross-check -- with no comfort term, ComfortMatching's joint OCP reduces
// exactly to SpeedMatching's free-time min-jerk problem (same cost, same
// BCs), so BOTH T* and the full trajectory shape should match SpeedMatching
// almost exactly (not just the shape at a fixed T, as SpeedMatchingSickness's
// analogous test checks -- here duration selection is unified too).

TEST(ComfortMatchingTest, RestToRestMatchesSpeedMatchingWhenAccelWeightIsZero) {
    ComfortMatching cm(1.0); // w_T = 1
    cm.set_jerk_weight(1.0);
    cm.set_jerk_limit(1.0);
    cm.set_accel_weight(0.0); // no comfort term -> reduces to plain SpeedMatching
    cm.set_search_bounds(1.0, 40.0, 400);

    LongitudinalState start{0.0, 0.0, 0.0, 0.0};
    LongitudinalState goal{100.0, 0.0, 0.0, 0.0};

    ASSERT_TRUE(cm.build(start, goal));
    EXPECT_NEAR(cm.get_T(), 18.171205928321406, 1e-3);

    SpeedMatching sm;
    sm.set_jerk_limit(1.0);
    ASSERT_TRUE(sm.build_fixed_time(start, goal, cm.get_T()));

    for (double t = 0.0; t <= cm.get_T(); t += cm.get_T() / 10.0) {
        LongitudinalState a = cm.eval(t);
        LongitudinalState b = sm.eval(t);
        EXPECT_NEAR(a.s, b.s, 1e-4);
        EXPECT_NEAR(a.v, b.v, 1e-4);
        EXPECT_NEAR(a.a, b.a, 1e-4);
        EXPECT_NEAR(a.j, b.j, 1e-3);
    }
}

TEST(ComfortMatchingTest, BoundaryConditionsMatch) {
    ComfortMatching cm(1.0);
    cm.set_jerk_limit(10.0);
    cm.set_accel_limit(10.0);
    cm.set_accel_weight(1.0);
    cm.set_search_bounds(0.5, 25.0, 800);

    LongitudinalState start{0.0, 0.0, 0.0, 0.0};
    LongitudinalState goal{100.0, 0.0, 0.0, 0.0};

    ASSERT_TRUE(cm.build(start, goal));
    const double T = cm.get_T();

    LongitudinalState eval_start = cm.eval(0.0);
    LongitudinalState eval_goal = cm.eval(T);

    const double tol = 1e-4;
    EXPECT_NEAR(eval_start.s, start.s, tol);
    EXPECT_NEAR(eval_start.v, start.v, tol);
    EXPECT_NEAR(eval_start.a, start.a, tol);
    EXPECT_NEAR(eval_goal.s, goal.s, tol);
    EXPECT_NEAR(eval_goal.v, goal.v, tol);
    EXPECT_NEAR(eval_goal.a, goal.a, tol);
}

TEST(ComfortMatchingTest, FilteredAccelerationIsBoundedAndSmooth) {
    ComfortMatching cm(1.0);
    cm.set_jerk_limit(10.0);
    cm.set_accel_limit(10.0);
    cm.set_accel_weight(1.0);
    cm.set_search_bounds(0.5, 25.0, 800);

    LongitudinalState start{0.0, 0.0, 0.0, 0.0};
    LongitudinalState goal{100.0, 0.0, 0.0, 0.0};
    ASSERT_TRUE(cm.build(start, goal));

    const double T = cm.get_T();
    double max_abs_aw = 0.0;
    double prev = cm.AW(0.0);
    for (int i = 1; i <= 50; ++i) {
        const double t = T * i / 50.0;
        const double aw = cm.AW(t);
        EXPECT_TRUE(std::isfinite(aw));
        max_abs_aw = std::max(max_abs_aw, std::abs(aw));
        EXPECT_LT(std::abs(aw - prev), 5.0) << "a_w should not jump discontinuously";
        prev = aw;
    }
    EXPECT_GT(max_abs_aw, 0.0);
}

// Reproduces the directional finding from this primitive's design validation
// (motionsick/scratchpad prototypes, see git history): raising the comfort
// weight relative to jerk should reduce integrated a_w^2, since the optimizer
// is explicitly trading jerk budget for comfort.
TEST(ComfortMatchingTest, ComfortImprovesAsAccelWeightIncreases) {
    LongitudinalState start{0.0, 0.0, 0.0, 0.0};
    LongitudinalState goal{100.0, 0.0, 0.0, 0.0};

    auto integrated_aw_sq = [&](double w_a) {
        ComfortMatching cm(1.0);
        cm.set_jerk_limit(10.0);
        cm.set_accel_limit(10.0);
        cm.set_accel_weight(w_a);
        cm.set_search_bounds(0.5, 25.0, 1200);
        EXPECT_TRUE(cm.build(start, goal));
        const double T = cm.get_T();
        const int n = 400;
        double sum = 0.0;
        double prev = cm.AW(0.0) * cm.AW(0.0);
        const double h = T / n;
        for (int i = 1; i <= n; ++i) {
            const double aw = cm.AW(i * h);
            const double cur = aw * aw;
            sum += 0.5 * (prev + cur) * h;
            prev = cur;
        }
        return sum;
    };

    const double low = integrated_aw_sq(0.1);
    const double high = integrated_aw_sq(5.0);
    EXPECT_LT(high, low) << "higher comfort weight should reduce integrated a_w^2";
}
