"""
Shared sympy -> C++ code-generation utilities for vprim motion primitives.

Pattern (see speed_matching.py for the worked example):
  1. Derive the closed-form math symbolically in sympy.
  2. Substitute powers of T for the T2/T3/T4/T5 helper variables the
     hand-written C++ already declares (so generated code matches the
     existing style -- `T3` instead of `pow(T, 3)`).
  3. Print with sympy's C++ code printer, then rename sympy symbol names to
     the actual C++ identifiers (start.v, goal.a, ds, ...) used in src/*.cc.
  4. Numerically validate the generated expression against the hardcoded C++
     formula (or a literal Python transcription of it) before pasting it in.

Codegen output is a draft to diff against existing code, not an automatic
build step -- nothing here writes into src/*.cc directly. Always run a
module's __main__ (which does the validation in step 4) before trusting its
printed snippets.
"""

import re

import sympy as sp


def substitute_T_powers(expr, T, max_power=5):
    """Replace T**2..T**max_power with symbols T2..T{max_power}, matching the
    hand-written style where SpeedMatching.cc/SpeedMatchingSickness.cc declare
    `T2 = T*T; T3 = T2*T; ...` once and reuse them in every coefficient.

    Uses xreplace (pure syntactic tree substitution), NOT subs: subs applies
    assumption-based rewriting, so once T2:=T**2 is in the substitution set
    and T is declared positive, subs will rewrite any leftover bare T as
    sqrt(T2) to "use" the substitution -- exactly the opposite of what we
    want here (we want T1 left alone as plain T).
    """
    subs = {}
    for k in range(2, max_power + 1):
        Tk = sp.Symbol(f"T{k}")
        subs[T**k] = Tk
        subs[T**-k] = 1 / Tk  # denominators appear as T**(-k), not 1/T**k
    return expr.xreplace(subs)


def to_cxx(expr, rename=None):
    """sympy expr -> C++ expression string, with sympy symbol names replaced
    by the actual C++ identifiers in `rename` (dict[str, str]). Replacement is
    word-boundary-safe so e.g. renaming "a0" won't clobber part of "a0_other".
    """
    code = sp.cxxcode(expr)
    if rename:
        for old, new in rename.items():
            code = re.sub(rf"\b{re.escape(old)}\b", new, code)
    return code


def assert_numeric_match(sympy_expr, symbols, cxx_fn, n_trials=20, rng=None, atol=1e-6):
    """Evaluate `sympy_expr` (function of `symbols`, a tuple/list of sympy
    Symbols) against `cxx_fn` (a plain Python callable taking the same
    positional arguments -- usually a literal transcription of the hardcoded
    C++ formula) at random inputs. Raises AssertionError on mismatch.

    This numeric-parity check is what makes generated code trustworthy: sympy
    algebra can be correct yet differ from the *specific* closed form already
    hardcoded in src/*.cc (different but equivalent factorizations), so a
    symbolic-only check isn't enough -- always compare numerically against the
    literal existing formula.
    """
    import numpy as np

    rng = rng or np.random.default_rng(0)
    f = sp.lambdify(symbols, sympy_expr, "numpy")
    for _ in range(n_trials):
        vals = rng.uniform(1.0, 10.0, size=len(symbols))
        lhs = float(f(*vals))
        rhs = float(cxx_fn(*vals))
        if abs(lhs - rhs) > atol:
            raise AssertionError(f"mismatch at {dict(zip(symbols, vals))}: sympy={lhs} vs cxx_fn={rhs}")
