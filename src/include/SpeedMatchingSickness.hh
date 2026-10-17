#pragma once

#ifndef VPRIM_SPEED_MATCHING_SICKNESS_HH
#define VPRIM_SPEED_MATCHING_SICKNESS_HH

#include <array>
#include <utility>
#include <vector>

#include "MotionPrimitive.hh"
#include "include/utility.hh"
#include "include/linalg.hh"
#include "include/SpeedMatching.hh"  // reuses LongitudinalState

namespace vprim {

    // A strictly-proper n-state LTI weighting filter: q' = A q + B a, a_w = C q.
    // D is implicitly 0 (see motion_sickness_filter_minT_notes.md section 2: a
    // first-order filter cannot reproduce a band-pass, so this is always n >= 2).
    struct LTIFilter {
        int n{0};
        std::vector<double> A;  // n*n, row-major
        std::vector<double> B;  // n
        std::vector<double> C;  // n
    };

    // Default 2nd-order approximation of the ISO 2631-1 x-axis Wf weighting
    // filter used throughout motion_sickness_filter_minT_notes.md ("2 (used in
    // all tests)" row): fn ~= 0.2288 Hz, Q ~= 0.5715, ~20% magnitude error vs
    // the band-pass target over 0.1-0.5 Hz. Swap via set_filter() for a
    // different order/fit (see the roadmap doc, section 2 and section 6).
    [[nodiscard]] LTIFilter default_motion_sickness_filter();

    // Motion-sickness-minimizing longitudinal primitive. Reuses SpeedMatching's
    // quintic-position / cubic-acceleration shape (same free-time min-jerk BVP),
    // but instead of picking T by minimizing the jerk-squared cost again, it
    // minimizes J(T) = int_0^T (a_w^2 + w_T) dt, where a_w is the acceleration
    // filtered through an LTIFilter (a proxy for motion sickness, not raw
    // acceleration/jerk). See motionsick/motion_sickness_primitive_roadmap.md.
    class SpeedMatchingSickness : public MotionPrimitive<LongitudinalState, double> {
    private:
        LongitudinalState start_state_;
        LongitudinalState goal_state_;
        Matrix z0_;  // cached z0(T_), reused by AW()
        double wT_{1.0};
        double T_{0.0};
        double cost_{0.0};
        bool is_valid_{false};

        LTIFilter filter_{default_motion_sickness_filter()};
        std::vector<double> q0_{0.0, 0.0};

        // Divides the comfort term (z0'P(T)z0, i.e. the integrated a_w^2) by
        // this factor before adding w_T*T: J(T) = (1/accel_scale_)*z0'P(T)z0 +
        // w_T*T. Default 1.0 is the literal cost from notes.md/filtered_cost.mpl.
        // Mirrors MDV_2.mw's J_SM_bis := w__T + w__a*a(t)^2/50 pattern (a fixed
        // normalization divisor on the comfort term). Raising it makes comfort
        // cheaper relative to w_T*T, which is needed for w_T to meaningfully
        // affect T* on fast/short maneuvers where a_w stays small throughout
        // (see python_bindings/examples/plot_motion_sickness_primitive.py).
        double accel_scale_{1.0};

        // Search interval for the (transcendental) stationarity scan, matching
        // the Maple/Python reference in motion_sickness_filter_minT_notes.md
        // section 6: M(T) becomes ill-conditioned for T >~ 25 s.
        double T_search_min_{0.5};
        double T_search_max_{25.0};
        int T_search_points_{400};

        Polynomial poly_s_, poly_v_, poly_a_, poly_j_;

        // Cubic acceleration coefficients c = [c0,c1,c2,c3] (a(t) = c0 + c1 t +
        // c2 t^2 + c3 t^3) and their T-derivatives, from the same BC system as
        // SpeedMatching (notes.md section 1's M(T) c = b).
        [[nodiscard]] std::array<double, 4> accel_coeffs(double T, const LongitudinalState& start, const LongitudinalState& goal) const;
        [[nodiscard]] std::array<double, 4> accel_coeffs_derivative(double T, const LongitudinalState& start, const LongitudinalState& goal) const;

        // Augmented autonomous system F (N=n+4 states: filter + [a,a',a'',a''']),
        // H (1xN, picks a_w), built once per build() call from filter_.
        [[nodiscard]] Matrix build_F() const;
        [[nodiscard]] Matrix build_H() const;

        [[nodiscard]] std::pair<Matrix, Matrix> z0_dz0(double T, const LongitudinalState& start, const LongitudinalState& goal) const;
        // Returns {P(T), expm(F*T)}.
        [[nodiscard]] std::pair<Matrix, Matrix> P_and_EFT(const Matrix& F, const Matrix& H, double T) const;

        [[nodiscard]] double J(double T, const LongitudinalState& start, const LongitudinalState& goal) const;
        [[nodiscard]] double dJ(double T, const LongitudinalState& start, const LongitudinalState& goal) const;

        // Scans [T_search_min_, T_search_max_] for the first -> + sign change of
        // dJ/dT, then bisects. Returns false if none is found (no interior
        // minimum in the search interval).
        bool find_T(const LongitudinalState& start, const LongitudinalState& goal);

        bool build_with_time(const LongitudinalState& start, const LongitudinalState& goal, double T);

    public:
        SpeedMatchingSickness() = default;
        explicit SpeedMatchingSickness(double weight_time) : wT_(weight_time) {}

        void set_time_weight(double wT) { wT_ = wT; }
        void set_accel_scale(double scale) { accel_scale_ = scale; }
        void set_filter(const LTIFilter& filter) { filter_ = filter; q0_.assign(filter.n, 0.0); }
        void set_filter_initial_state(std::vector<double> q0) { q0_ = std::move(q0); }
        void set_search_bounds(double Tmin, double Tmax, int n_points = 400) {
            T_search_min_ = Tmin;
            T_search_max_ = Tmax;
            T_search_points_ = n_points;
        }

        [[nodiscard]] double get_T() const { return T_; }
        [[nodiscard]] inline double T() const { return T_; }
        [[nodiscard]] bool is_ok() const { return is_valid_; }

        bool build(const LongitudinalState& start, const LongitudinalState& goal) override;

        [[nodiscard]] LongitudinalState eval(double t) const override;
        [[nodiscard]] double get_cost() const override { return cost_; }

        // Filtered ("motion sickness") acceleration a_w(t), t in [0, T].
        [[nodiscard]] double AW(double t) const;
    };

} // namespace vprim

#endif // VPRIM_SPEED_MATCHING_SICKNESS_HH
