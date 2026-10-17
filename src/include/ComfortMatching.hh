#pragma once

#ifndef VPRIM_COMFORT_MATCHING_HH
#define VPRIM_COMFORT_MATCHING_HH

#include <vector>

#include "MotionPrimitive.hh"
#include "include/linalg.hh"
#include "include/SpeedMatching.hh"        // reuses LongitudinalState
#include "include/SpeedMatchingSickness.hh" // reuses LTIFilter, default_motion_sickness_filter

namespace vprim {

    // Motion-sickness-minimizing longitudinal primitive, JOINTLY optimizing
    // shape and duration (unlike SpeedMatchingSickness, which reuses
    // SpeedMatching's fixed min-jerk shape and only changes how T is picked).
    // Bakes the frequency-weighted-acceleration cost directly into a
    // free-time LQ optimal control problem:
    //
    //   minimize  w_T*T + integral_0^T [ w_j*(jx/jmax)^2 + w_a*(a_w/a_max)^2 ] dt
    //   subject to  s'=v, v'=a, a'=jx, q'=Aq+Ba, a_w=Cq
    //
    // with full-state boundary conditions on s,v,a and q(0)=0 (q(T) free).
    // A pure a_w^2 cost (no jerk term, i.e. w_j=0) is a SINGULAR optimal
    // control problem -- the Hamiltonian is affine in jx, so there is no
    // smooth minimizing control (confirmed analytically and numerically
    // during this primitive's design: see git history / design notes --
    // exact a_w=0 from q(0)=0 forces a(t)=0 identically, which cannot
    // satisfy a non-trivial boundary condition). w_j and jmax must therefore
    // both be kept strictly positive.
    //
    // By Pontryagin's Maximum Principle, with costate lambda=[ls,lv,la,lq]:
    //   optimal jerk:  jx*(t) = -la(t) / (2*W_J),  W_J = w_j/jmax^2
    //   costate eqs:   ls'=0, lv'=-ls, la'=-lv-B^T*lq, lq'=-A^T*lq-2*W_A*C^T*C*q
    //   W_A = w_a/a_max^2
    // Substituting jx* into the state equations gives a single LINEAR
    // autonomous system z'=Mz, z=[s,v,a,q,ls,lv,la,lq] -- the whole
    // trajectory (state AND costate, hence AND the optimal jerk) is given by
    // a single matrix exponential, exactly like SpeedMatchingSickness's
    // filter-only Van Loan construction, just applied to a bigger system
    // that also determines the shape (not just the cost of a fixed shape).
    //
    // Free T is resolved via the standard autonomous free-final-time
    // transversality condition H(T)=0, scanned+bisected the same way
    // SpeedMatchingSickness scans+bisects dJ/dT=0 (this is also
    // transcendental, for the same reason: matrix exponentials in T).
    class ComfortMatching : public MotionPrimitive<LongitudinalState, double> {
    private:
        double wT_{1.0};
        double w_j_{1.0};
        double jmax_{10.0};
        double w_a_{1.0};
        double a_max_{10.0};

        LTIFilter filter_{default_motion_sickness_filter()};
        std::vector<double> q0_{0.0, 0.0};

        double T_{0.0};
        double cost_{0.0};
        bool is_valid_{false};

        double T_search_min_{0.5};
        double T_search_max_{25.0};
        int T_search_points_{400};

        Matrix M_;    // cached Hamiltonian system matrix (depends on filter_/weights only)
        Matrix z0_;   // cached full [state;costate] initial vector at T_
        int nx_{0};   // 3 + filter_.n (state block size; full z is 2*nx_)

        [[nodiscard]] double W_J() const { return w_j_ / (jmax_ * jmax_); }
        [[nodiscard]] double W_A() const { return w_a_ / (a_max_ * a_max_); }

        // Builds M (2*nx x 2*nx) from filter_ and the current weights.
        [[nodiscard]] Matrix build_M() const;

        // For a fixed T, solves the linear two-point BVP for the unknown
        // initial costates (ls0,lv0,la0,lq0) given x(0)=[start.s,start.v,
        // start.a,0,...,0] and the terminal conditions [goal.s,goal.v,
        // goal.a, lq(T)=0]. Returns the full initial vector z0.
        [[nodiscard]] Matrix solve_for_T(const Matrix& M, const LongitudinalState& start,
                                          const LongitudinalState& goal, double T) const;

        // Free-time transversality residual H(T); its first -> + sign
        // change (scanned over [T_search_min_, T_search_max_]) locates T*.
        [[nodiscard]] double H_at_T(const Matrix& M, const Matrix& z0, double T) const;

        [[nodiscard]] bool find_T(const LongitudinalState& start, const LongitudinalState& goal);

        // Composite Simpson's rule over [0,T] for w_T*T + W_J*int(jx^2) +
        // W_A*int(a_w^2), evaluated once in build() to report get_cost().
        // Not needed during the T* search itself -- H(T) only needs the
        // boundary state/costate, not the integral cost.
        [[nodiscard]] double compute_cost(double T) const;

    public:
        ComfortMatching() = default;
        explicit ComfortMatching(double weight_time) : wT_(weight_time) {}

        void set_time_weight(double wT) { wT_ = wT; }
        void set_jerk_weight(double w_j) { w_j_ = w_j; }
        void set_jerk_limit(double jmax) { jmax_ = jmax; }
        void set_accel_weight(double w_a) { w_a_ = w_a; }
        void set_accel_limit(double a_max) { a_max_ = a_max; }
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

#endif // VPRIM_COMFORT_MATCHING_HH
