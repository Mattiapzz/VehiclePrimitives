#include "include/SpeedMatchingSickness.hh"

#include <algorithm>
#include <cmath>

using namespace vprim;

LTIFilter vprim::default_motion_sickness_filter() {
    // fn ~= 0.2288 Hz, Q ~= 0.5715 (motion_sickness_filter_minT_notes.md section 2,
    // "2 (used in all tests)" row). Controllable canonical form, D = 0.
    const double fn = 0.2288;
    const double Qf = 0.5715;
    const double wn = 2.0 * M_PI * fn;

    LTIFilter f;
    f.n = 2;
    f.A = {-wn / Qf, -(wn * wn),
           1.0, 0.0};
    f.B = {1.0, 0.0};
    f.C = {wn / Qf, 0.0};
    return f;
}

std::array<double, 4> SpeedMatchingSickness::accel_coeffs(
    double T, const LongitudinalState& start, const LongitudinalState& goal) const
{
    const double u0 = start.v, a0 = start.a;
    const double ds = goal.s - start.s, uT = goal.v, aT = goal.a;
    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    Matrix Mc(4, 4);
    Mc(0, 0) = 1;      Mc(0, 1) = 0;      Mc(0, 2) = 0;       Mc(0, 3) = 0;
    Mc(1, 0) = T;      Mc(1, 1) = T2 / 2; Mc(1, 2) = T3 / 3;  Mc(1, 3) = T4 / 4;
    Mc(2, 0) = T2 / 2; Mc(2, 1) = T3 / 6; Mc(2, 2) = T4 / 12; Mc(2, 3) = T5 / 20;
    Mc(3, 0) = 1;      Mc(3, 1) = T;      Mc(3, 2) = T2;      Mc(3, 3) = T3;

    Matrix b(4, 1);
    b(0, 0) = a0;
    b(1, 0) = uT - u0;
    b(2, 0) = ds - u0 * T;
    b(3, 0) = aT;

    Matrix c = Matrix::solve(Mc, b);
    return {c(0, 0), c(1, 0), c(2, 0), c(3, 0)};
}

std::array<double, 4> SpeedMatchingSickness::accel_coeffs_derivative(
    double T, const LongitudinalState& start, const LongitudinalState& goal) const
{
    const double u0 = start.v, a0 = start.a;
    const double ds = goal.s - start.s, uT = goal.v, aT = goal.a;
    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    Matrix Mc(4, 4);
    Mc(0, 0) = 1;      Mc(0, 1) = 0;      Mc(0, 2) = 0;       Mc(0, 3) = 0;
    Mc(1, 0) = T;      Mc(1, 1) = T2 / 2; Mc(1, 2) = T3 / 3;  Mc(1, 3) = T4 / 4;
    Mc(2, 0) = T2 / 2; Mc(2, 1) = T3 / 6; Mc(2, 2) = T4 / 12; Mc(2, 3) = T5 / 20;
    Mc(3, 0) = 1;      Mc(3, 1) = T;      Mc(3, 2) = T2;      Mc(3, 3) = T3;

    Matrix dMc(4, 4);
    dMc(0, 0) = 0; dMc(0, 1) = 0;      dMc(0, 2) = 0;       dMc(0, 3) = 0;
    dMc(1, 0) = 1; dMc(1, 1) = T;      dMc(1, 2) = T2;      dMc(1, 3) = T3;
    dMc(2, 0) = T; dMc(2, 1) = T2 / 2; dMc(2, 2) = T3 / 3;  dMc(2, 3) = T4 / 4;
    dMc(3, 0) = 0; dMc(3, 1) = 1;      dMc(3, 2) = 2 * T;   dMc(3, 3) = 3 * T2;

    Matrix b(4, 1);
    b(0, 0) = a0;
    b(1, 0) = uT - u0;
    b(2, 0) = ds - u0 * T;
    b(3, 0) = aT;

    Matrix c = Matrix::solve(Mc, b);

    Matrix db(4, 1);
    db(0, 0) = 0;
    db(1, 0) = 0;
    db(2, 0) = -u0;
    db(3, 0) = 0;

    Matrix rhs = db - (dMc * c);
    Matrix dc = Matrix::solve(Mc, rhs);
    return {dc(0, 0), dc(1, 0), dc(2, 0), dc(3, 0)};
}

Matrix SpeedMatchingSickness::build_F() const {
    const int n = filter_.n;
    const size_t N = static_cast<size_t>(n) + 4;
    Matrix F(N, N, 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            F(i, j) = filter_.A[i * n + j];
        }
        F(i, n) = filter_.B[i];
    }
    F(n, n + 1) = 1.0;
    F(n + 1, n + 2) = 1.0;
    F(n + 2, n + 3) = 1.0;
    return F;
}

Matrix SpeedMatchingSickness::build_H() const {
    const int n = filter_.n;
    const size_t N = static_cast<size_t>(n) + 4;
    Matrix H(1, N, 0.0);
    for (int i = 0; i < n; ++i) {
        H(0, i) = filter_.C[i];
    }
    return H;
}

std::pair<Matrix, Matrix> SpeedMatchingSickness::z0_dz0(
    double T, const LongitudinalState& start, const LongitudinalState& goal) const
{
    const int n = filter_.n;
    const size_t N = static_cast<size_t>(n) + 4;
    const auto c = accel_coeffs(T, start, goal);
    const auto dc = accel_coeffs_derivative(T, start, goal);

    Matrix z0(N, 1, 0.0);
    for (int i = 0; i < n; ++i) z0(i, 0) = q0_[i];
    z0(n + 0, 0) = c[0];
    z0(n + 1, 0) = c[1];
    z0(n + 2, 0) = 2.0 * c[2];
    z0(n + 3, 0) = 6.0 * c[3];

    Matrix dz0(N, 1, 0.0);
    dz0(n + 0, 0) = dc[0];
    dz0(n + 1, 0) = dc[1];
    dz0(n + 2, 0) = 2.0 * dc[2];
    dz0(n + 3, 0) = 6.0 * dc[3];

    return {z0, dz0};
}

std::pair<Matrix, Matrix> SpeedMatchingSickness::P_and_EFT(const Matrix& F, const Matrix& H, double T) const {
    const size_t N = F.rows();
    const Matrix HH = H.transpose() * H;

    Matrix V(2 * N, 2 * N, 0.0);
    V.set_block(0, 0, F.transpose() * (-1.0));
    V.set_block(0, N, HH);
    V.set_block(N, N, F);

    const Matrix E = expm(V * T);
    const Matrix EFT = E.get_block(N, N, N, N);
    const Matrix G2 = E.get_block(0, N, N, N);
    const Matrix P = EFT.transpose() * G2;
    return {P, EFT};
}

double SpeedMatchingSickness::J(double T, const LongitudinalState& start, const LongitudinalState& goal) const {
    const Matrix F = build_F();
    const Matrix H = build_H();
    const auto [z0, dz0] = z0_dz0(T, start, goal);
    const auto [P, EFT] = P_and_EFT(F, H, T);
    const Matrix Jmat = z0.transpose() * (P * z0);
    return Jmat(0, 0) / accel_scale_ + wT_ * T;
}

double SpeedMatchingSickness::dJ(double T, const LongitudinalState& start, const LongitudinalState& goal) const {
    const Matrix F = build_F();
    const Matrix H = build_H();
    const auto [z0, dz0] = z0_dz0(T, start, goal);
    const auto [P, EFT] = P_and_EFT(F, H, T);

    const Matrix aw_mat = H * (EFT * z0);
    const double aw = aw_mat(0, 0);

    const Matrix cross = dz0.transpose() * (P * z0);
    return (aw * aw + 2.0 * cross(0, 0)) / accel_scale_ + wT_;
}

bool SpeedMatchingSickness::find_T(const LongitudinalState& start, const LongitudinalState& goal) {
    const int n_points = std::max(T_search_points_, 2);
    const double dT = (T_search_max_ - T_search_min_) / static_cast<double>(n_points - 1);

    double T_prev = T_search_min_;
    double g_prev = dJ(T_prev, start, goal);

    for (int i = 1; i < n_points; ++i) {
        const double T_curr = T_search_min_ + dT * static_cast<double>(i);
        const double g_curr = dJ(T_curr, start, goal);

        if (g_prev < 0.0 && g_curr >= 0.0) {
            double lo = T_prev, hi = T_curr;
            for (int k = 0; k < 60; ++k) {
                const double mid = 0.5 * (lo + hi);
                if (dJ(mid, start, goal) < 0.0) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            T_ = 0.5 * (lo + hi);
            return true;
        }
        T_prev = T_curr;
        g_prev = g_curr;
    }
    return false;
}

bool SpeedMatchingSickness::build_with_time(
    const LongitudinalState& start, const LongitudinalState& goal, double T)
{
    T_ = T;
    const double ds = goal.s - start.s;
    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    const double c0 = start.s;
    const double c1 = start.v;
    const double c2 = start.a / 2.0;
    const double c3 = ((-3.0 * start.a + goal.a) * T2 + (-12.0 * start.v - 8.0 * goal.v) * T + 20.0 * ds) / (2.0 * T3);
    const double c4 = ((3.0 * start.a - 2.0 * goal.a) * T2 + (16.0 * start.v + 14.0 * goal.v) * T - 30.0 * ds) / (2.0 * T4);
    const double c5 = ((-start.a + goal.a) * T2 + (-6.0 * start.v - 6.0 * goal.v) * T + 12.0 * ds) / (2.0 * T5);

    poly_s_ = {c0, c1, c2, c3, c4, c5};
    poly_v_ = poly_s_.derivative();
    poly_a_ = poly_v_.derivative();
    poly_j_ = poly_a_.derivative();

    cost_ = J(T, start, goal);
    z0_ = z0_dz0(T, start, goal).first;
    return true;
}

bool SpeedMatchingSickness::build(const LongitudinalState& start, const LongitudinalState& goal) {
    start_state_ = start;
    goal_state_ = goal;
    is_valid_ = false;

    if (!find_T(start, goal)) {
        return false;
    }
    is_valid_ = build_with_time(start, goal, T_);
    return is_valid_;
}

LongitudinalState SpeedMatchingSickness::eval(double t) const {
    t = std::clamp(t, 0.0, T_);
    LongitudinalState state;
    state.s = poly_s_.eval(t);
    state.v = poly_v_.eval(t);
    state.a = poly_a_.eval(t);
    state.j = poly_j_.eval(t);
    return state;
}

double SpeedMatchingSickness::AW(double t) const {
    t = std::clamp(t, 0.0, T_);
    const Matrix F = build_F();
    const Matrix H = build_H();
    const Matrix zt = expm(F * t) * z0_;
    const Matrix aw = H * zt;
    return aw(0, 0);
}
