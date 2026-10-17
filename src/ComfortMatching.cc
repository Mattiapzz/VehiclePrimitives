#include "include/ComfortMatching.hh"

#include <algorithm>
#include <cmath>

using namespace vprim;

Matrix ComfortMatching::build_M() const {
    const int n = filter_.n;
    const int nx = 3 + n;
    const int N = 2 * nx;
    Matrix M(N, N, 0.0);

    const int IS = 0, IV = 1, IA = 2;
    const int IQ0 = 3;
    const int ILS = nx + 0, ILV = nx + 1, ILA = nx + 2;
    const int ILQ0 = nx + 3;

    const double W_J = this->W_J();
    const double W_A = this->W_A();

    // x' = v, a, -la/(2*W_J), Aq+Ba   (jx* = -la/(2*W_J) from dH/djx=0)
    M(IS, IV) = 1.0;
    M(IV, IA) = 1.0;
    M(IA, ILA) = -1.0 / (2.0 * W_J);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            M(IQ0 + i, IQ0 + j) = filter_.A[i * n + j];
        }
        M(IQ0 + i, IA) = filter_.B[i];
    }

    // lam' = 0, -ls, -lv - B^T.lq, -A^T.lq - 2*W_A*C^T*C*q
    M(ILV, ILS) = -1.0;
    M(ILA, ILV) = -1.0;
    for (int i = 0; i < n; ++i) {
        M(ILA, ILQ0 + i) = -filter_.B[i];
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            // -A^T: row i, col j of A^T is A[j*n+i]
            M(ILQ0 + i, ILQ0 + j) = -filter_.A[j * n + i];
        }
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            M(ILQ0 + i, IQ0 + j) += -2.0 * W_A * filter_.C[i] * filter_.C[j];
        }
    }

    return M;
}

Matrix ComfortMatching::solve_for_T(
    const Matrix& M, const LongitudinalState& start, const LongitudinalState& goal, double T) const
{
    const int n = filter_.n;
    const int nx = 3 + n;
    const Matrix G = expm(M * T);

    Matrix z0_fixed(nx, 1, 0.0);
    z0_fixed(0, 0) = start.s;
    z0_fixed(1, 0) = start.v;
    z0_fixed(2, 0) = start.a;
    // q(0) = q0_ (default zero)
    for (int i = 0; i < n; ++i) z0_fixed(3 + i, 0) = q0_[i];

    // Known terminal rows: s(T),v(T),a(T) and lq(T)=0 -- (3+n) equations for
    // the (3+n) unknown initial costates.
    const int n_rows = 3 + n;
    Matrix G_unknown(n_rows, nx, 0.0);
    Matrix rhs(n_rows, 1, 0.0);

    // rows 0..2: s(T),v(T),a(T)
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < nx; ++c) G_unknown(r, c) = G(r, nx + c);
        double known_contrib = 0.0;
        for (int c = 0; c < nx; ++c) known_contrib += G(r, c) * z0_fixed(c, 0);
        const double goal_val = (r == 0) ? goal.s : (r == 1) ? goal.v : goal.a;
        rhs(r, 0) = goal_val - known_contrib;
    }
    // rows 3..3+n-1: lq(T) = 0
    for (int i = 0; i < n; ++i) {
        const int r = 3 + i;
        const int row_idx = nx + 3 + i;
        for (int c = 0; c < nx; ++c) G_unknown(r, c) = G(row_idx, nx + c);
        double known_contrib = 0.0;
        for (int c = 0; c < nx; ++c) known_contrib += G(row_idx, c) * z0_fixed(c, 0);
        rhs(r, 0) = 0.0 - known_contrib;
    }

    const Matrix p = Matrix::solve(G_unknown, rhs);

    Matrix z0(2 * nx, 1, 0.0);
    for (int i = 0; i < nx; ++i) z0(i, 0) = z0_fixed(i, 0);
    for (int i = 0; i < nx; ++i) z0(nx + i, 0) = p(i, 0);
    return z0;
}

double ComfortMatching::H_at_T(const Matrix& M, const Matrix& z0, double T) const {
    const int n = filter_.n;
    const int nx = 3 + n;
    const Matrix zT = expm(M * T) * z0;

    const double v_T = zT(1, 0);
    const double a_T = zT(2, 0);
    const double ls_T = zT(nx + 0, 0);
    const double lv_T = zT(nx + 1, 0);
    const double la_T = zT(nx + 2, 0);

    double aw_T = 0.0;
    for (int i = 0; i < n; ++i) aw_T += filter_.C[i] * zT(3 + i, 0);

    const double W_J = this->W_J();
    const double W_A = this->W_A();

    // H(T) = -la^2/(4*W_J) + W_A*aw^2 + w_T + ls*v + lv*a
    // (the lq.(Aq+Ba) term vanishes since lq(T)=0 by construction)
    return -(la_T * la_T) / (4.0 * W_J) + W_A * (aw_T * aw_T) + wT_ + ls_T * v_T + lv_T * a_T;
}

bool ComfortMatching::find_T(const LongitudinalState& start, const LongitudinalState& goal) {
    M_ = build_M();

    const int n_points = std::max(T_search_points_, 2);
    const double dT = (T_search_max_ - T_search_min_) / static_cast<double>(n_points - 1);

    double T_prev = T_search_min_;
    Matrix z0_prev = solve_for_T(M_, start, goal, T_prev);
    double g_prev = H_at_T(M_, z0_prev, T_prev);

    for (int i = 1; i < n_points; ++i) {
        const double T_curr = T_search_min_ + dT * static_cast<double>(i);
        const Matrix z0_curr = solve_for_T(M_, start, goal, T_curr);
        const double g_curr = H_at_T(M_, z0_curr, T_curr);

        if (g_prev < 0.0 && g_curr >= 0.0) {
            double lo = T_prev, hi = T_curr;
            for (int k = 0; k < 60; ++k) {
                const double mid = 0.5 * (lo + hi);
                const Matrix z0_mid = solve_for_T(M_, start, goal, mid);
                if (H_at_T(M_, z0_mid, mid) < 0.0) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            T_ = 0.5 * (lo + hi);
            z0_ = solve_for_T(M_, start, goal, T_);
            return true;
        }
        T_prev = T_curr;
        g_prev = g_curr;
    }
    return false;
}

double ComfortMatching::compute_cost(double T) const {
    const int n = filter_.n;
    const double W_J = this->W_J();
    const double W_A = this->W_A();

    auto integrand = [&](double t) {
        const Matrix zt = expm(M_ * t) * z0_;
        const int nx = 3 + n;
        const double la = zt(nx + 2, 0);
        const double jx = -la / (2.0 * W_J);
        double aw = 0.0;
        for (int i = 0; i < n; ++i) aw += filter_.C[i] * zt(3 + i, 0);
        return W_J * jx * jx + W_A * aw * aw;
    };

    const int n_intervals = 200; // even, for composite Simpson
    const double h = T / n_intervals;
    double sum = integrand(0.0) + integrand(T);
    for (int i = 1; i < n_intervals; ++i) {
        const double t = i * h;
        sum += (i % 2 == 0 ? 2.0 : 4.0) * integrand(t);
    }
    const double integral = sum * h / 3.0;

    return wT_ * T + integral;
}

bool ComfortMatching::build(const LongitudinalState& start, const LongitudinalState& goal) {
    is_valid_ = false;

    if (!find_T(start, goal)) {
        return false;
    }

    cost_ = compute_cost(T_);
    is_valid_ = true;
    return true;
}

LongitudinalState ComfortMatching::eval(double t) const {
    t = std::clamp(t, 0.0, T_);
    const int n = filter_.n;
    const int nx = 3 + n;
    const Matrix zt = expm(M_ * t) * z0_;

    LongitudinalState state;
    state.s = zt(0, 0);
    state.v = zt(1, 0);
    state.a = zt(2, 0);
    const double la = zt(nx + 2, 0);
    state.j = -la / (2.0 * W_J());
    return state;
}

double ComfortMatching::AW(double t) const {
    t = std::clamp(t, 0.0, T_);
    const int n = filter_.n;
    const Matrix zt = expm(M_ * t) * z0_;
    double aw = 0.0;
    for (int i = 0; i < n; ++i) aw += filter_.C[i] * zt(3 + i, 0);
    return aw;
}
