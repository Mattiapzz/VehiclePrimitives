"""
Code generator for the min-jerk min-time longitudinal primitive: the quintic
position shape shared by SpeedMatching (src/SpeedMatching.cc) and
SpeedMatchingSickness (src/SpeedMatchingSickness.cc), and the min-time sextic
used by SpeedMatching alone.

This is the worked example / template for adding a new primitive to
codegen/: derive the closed form symbolically in sympy, emit matching C++
(see common.py), and numerically validate the generated expressions against
the exact hardcoded formulas already in the C++ sources before trusting them.
Nothing here writes into src/*.cc -- it's a draft generator to diff against
by hand.

Run directly:
    python3 codegen/speed_matching.py

Background math: motion_sickness_filter_minT_notes.md section 1 ("Cubic
coefficients from the BCs"), motionsick/sympy_port.py sections 1-2 (same
derivation, parameterized via the cubic-acceleration c0..c3 representation
instead of the direct quintic c3/c4/c5 representation hardcoded in the C++ --
both are equivalent closed forms of the same free-time min-jerk BVP).
"""

import numpy as np
import sympy as sp

from common import assert_numeric_match, substitute_T_powers, to_cxx

# ---------------------------------------------------------------------------
# Symbols. Naming matches motion_sickness_filter_minT_notes.md /
# motionsick/sympy_port.py: u0/a0 = start velocity/accel, uT/aT = goal
# velocity/accel, ds = goal.s - start.s (NOT absolute goal position), T =
# maneuver duration.
# ---------------------------------------------------------------------------

t, T = sp.symbols("t T", positive=True)
u0, a0, uT, aT, ds = sp.symbols("u0 a0 u_T a_T ds", real=True)
c3, c4, c5 = sp.symbols("c3 c4 c5", real=True)

RENAME = {
    "u0": "start.v",
    "a0": "start.a",
    "u_T": "goal.v",
    "a_T": "goal.a",
    "ds": "ds",
}

# ---------------------------------------------------------------------------
# Part 1: quintic position shape, solved DIRECTLY for c3,c4,c5 -- the exact
# representation SpeedMatching.cc/SpeedMatchingSickness.cc hardcode (not the
# cubic-acceleration c0..c3 representation in sympy_port.py section 1, which
# is an equivalent but differently-parameterized closed form of the same BVP).
# c0=start.s, c1=start.v, c2=start.a/2 are fixed directly by the t=0 BCs, so
# only c3,c4,c5 need solving from the t=T BCs.
# ---------------------------------------------------------------------------

s_expr = u0 * t + a0 / 2 * t**2 + c3 * t**3 + c4 * t**4 + c5 * t**5
v_expr = sp.diff(s_expr, t)
a_expr = sp.diff(v_expr, t)

eqs = [
    sp.Eq(s_expr.subs(t, T), ds),
    sp.Eq(v_expr.subs(t, T), uT),
    sp.Eq(a_expr.subs(t, T), aT),
]
shape_sol = sp.solve(eqs, [c3, c4, c5])

print("=== Quintic position shape (shared by SpeedMatching.cc / SpeedMatchingSickness.cc) ===")
for sym in (c3, c4, c5):
    expr = substitute_T_powers(sp.simplify(shape_sol[sym]), T)
    print(f"double {sym.name} = {to_cxx(expr, RENAME)};")


def cxx_c3(u0v, a0v, uTv, aTv, dsv, Tv):
    T2, T3 = Tv**2, Tv**3
    return ((-3.0 * a0v + aTv) * T2 + (-12.0 * u0v - 8.0 * uTv) * Tv + 20.0 * dsv) / (2.0 * T3)


def cxx_c4(u0v, a0v, uTv, aTv, dsv, Tv):
    T2, T4 = Tv**2, Tv**4
    return ((3.0 * a0v - 2.0 * aTv) * T2 + (16.0 * u0v + 14.0 * uTv) * Tv - 30.0 * dsv) / (2.0 * T4)


def cxx_c5(u0v, a0v, uTv, aTv, dsv, Tv):
    T2, T5 = Tv**2, Tv**5
    return ((-a0v + aTv) * T2 + (-6.0 * u0v - 6.0 * uTv) * Tv + 12.0 * dsv) / (2.0 * T5)


shape_syms = (u0, a0, uT, aT, ds, T)
for sym, cxx_fn in [(c3, cxx_c3), (c4, cxx_c4), (c5, cxx_c5)]:
    assert_numeric_match(shape_sol[sym], shape_syms, cxx_fn)
print("  -> matches src/SpeedMatching.cc / src/SpeedMatchingSickness.cc hardcoded c3/c4/c5. OK\n")

# ---------------------------------------------------------------------------
# Part 2: min-time sextic (dJ/dT = 0 for J = w_T*T + int(jx^2 dt, 0, T)),
# used only by SpeedMatching.cc -- SpeedMatchingSickness.cc replaces this
# stationarity condition with the filtered-cost one instead (see
# motionsick/motion_sickness_primitive_roadmap.md section 3). The cubic-
# acceleration representation (not the quintic c3/c4/c5 one from Part 1) is
# the convenient parameterization for this integral.
# ---------------------------------------------------------------------------

cc0, cc1, cc2, cc3 = sp.symbols("cc0 cc1 cc2 cc3", real=True)
Mc = sp.Matrix(
    [
        [1, 0, 0, 0],
        [T, T**2 / 2, T**3 / 3, T**4 / 4],
        [T**2 / 2, T**3 / 6, T**4 / 12, T**5 / 20],
        [1, T, T**2, T**3],
    ]
)
bc_vec = sp.Matrix([a0, uT - u0, ds - u0 * T, aT])
c_sol = Mc.solve(bc_vec)
jx_expr = sp.diff(sum(c_sol[k] * t**k for k in range(4)), t)

wT_sym = sp.symbols("w_T", real=True)
Ju_expr = sp.integrate(jx_expr**2, (t, 0, T))
dJ_dT = sp.diff(wT_sym * T + Ju_expr, T)
GJT = sp.expand(sp.together(dJ_dT) * T**6)
sextic_coeffs = [sp.simplify(sp.Poly(GJT, T).coeff_monomial(T**k)) for k in range(7)]

print("=== Min-time sextic T_poly coefficients (SpeedMatching.cc only) ===")
for k, c in enumerate(sextic_coeffs):
    print(f"T_poly[{k}] = {to_cxx(c, RENAME)};  // coefficient of T^{k}")


def expected_sextic_coeffs(u0v, a0v, uTv, aTv, dsv):
    return [
        -3600.0 * dsv * dsv,
        2880.0 * dsv * (u0v + uTv),
        (360.0 * a0v - 360.0 * aTv) * dsv - 576.0 * u0v**2 - 1008.0 * u0v * uTv - 576.0 * uTv**2,
        (-144.0 * u0v - 96.0 * uTv) * a0v + 96.0 * (u0v + 1.5 * uTv) * aTv,
        -9.0 * a0v**2 + 6.0 * a0v * aTv - 9.0 * aTv**2,
        0.0,
        # T_poly[6] is w_T itself, set directly in SpeedMatching.cc -- nothing
        # to derive/compare for it.
    ]


rng = np.random.default_rng(0)
sextic_fns = [sp.lambdify((u0, a0, uT, aT, ds), c, "numpy") for c in sextic_coeffs[:6]]
for _ in range(10):
    u0v, a0v, uTv, aTv = rng.uniform(1.0, 10.0, size=4)
    dsv = rng.uniform(1.0, 100.0)
    got = [fi(u0v, a0v, uTv, aTv, dsv) for fi in sextic_fns]
    expected = expected_sextic_coeffs(u0v, a0v, uTv, aTv, dsv)
    for g, e in zip(got, expected):
        assert abs(g - e) < 1e-6, (g, e)
print("  -> matches src/SpeedMatching.cc T_poly[0..5] exactly (T_poly[6]=w_T set directly). OK")
