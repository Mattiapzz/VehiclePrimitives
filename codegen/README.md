# codegen/

Sympy-based code generators that derive the closed-form math behind `vprim`
motion primitives and emit matching C++ snippets, with numeric validation
against the hardcoded formulas already in `src/*.cc`.

## What this is (and isn't)

- **Is**: a draft generator. Run a module directly (e.g.
  `python3 codegen/speed_matching.py`), read the printed C++ snippets, and
  diff them by hand against `src/*.cc` before changing anything. The printed
  code is algebraically equivalent to the hardcoded formulas (verified
  numerically, see below) but not always textually identical -- sympy may
  factor differently (e.g. pulling out a `1/2` instead of folding it into
  `2.0 * T3`).
- **Isn't**: a build step. Nothing here is invoked by CMake or `pip install`,
  and nothing writes into `src/`. Treat generator output the same way you'd
  treat a colleague's suggested diff.

## Why this exists

Every closed-form primitive in this library (quintic position shapes, the
min-time sextic, the motion-sickness filtered-cost Van Loan construction) was
originally derived by hand or in Maple, then hand-transcribed into C++. That
transcription step is exactly where silent algebra mistakes creep in. Deriving
the same math independently in sympy and checking it against the hardcoded
C++ *numerically* (not just "it looks algebraically similar") catches that
class of bug, and gives a fast way to re-derive variants (different boundary
conditions, a new cost term) without redoing Maple/hand algebra from scratch.

This is also the generalization vehicle for "most of the motion primitives":
the min-jerk quintic shape in `speed_matching.py` is reused, unchanged, by
both `SpeedMatching` and `SpeedMatchingSickness` -- any future primitive built
on the same free-time min-jerk family (e.g. a lateral min-jerk-in-curvature
primitive) can import that same derivation instead of re-deriving it.

## Pattern for adding a new primitive (see `speed_matching.py`)

1. **Derive symbolically.** Define sympy symbols matching the naming already
   used in `motion_sickness_filter_minT_notes.md` / `motionsick/sympy_port.py`
   (`u0`/`a0` = start velocity/accel, `uT`/`aT` = goal velocity/accel, `ds` =
   `goal.s - start.s`, `T` = duration). Set up the boundary-condition
   equations and `sp.solve` for the unknown coefficients.
2. **Clean up for printing.** Use `common.substitute_T_powers(expr, T)` to
   replace `T**2..T**5` with `T2..T5` symbols, matching the hand-written style
   where the C++ declares `T2 = T*T; T3 = T2*T; ...` once and reuses them.
   (It uses `xreplace`, not `subs` -- `subs` will rewrite a leftover bare `T`
   as `sqrt(T2)` once `T2` is in the substitution set, because `T` is declared
   `positive`. `xreplace` is purely syntactic and won't do that.)
3. **Print and rename.** `common.to_cxx(expr, rename_dict)` runs
   `sympy.cxxcode` then renames sympy symbol names to the real C++
   identifiers (`u0` -> `start.v`, etc.).
4. **Validate numerically before trusting it.** Write a plain Python function
   that's a literal transcription of the existing hardcoded C++ formula (copy
   the arithmetic, don't get clever), and check
   `common.assert_numeric_match(sympy_expr, symbols, that_function)` passes
   across random inputs. This is the step that actually matters -- two
   algebraically-equivalent expressions can still both be wrong in the same
   way if you only "simplify and eyeball it", so always compare against an
   independent numeric transcription of what's really in the `.cc` file today.
5. Only after step 4 passes, consider pasting the generated snippet into
   `src/*.cc` -- and even then, re-run `ctest` (`build/`) afterward.

## Files

- `common.py` -- shared utilities (`substitute_T_powers`, `to_cxx`,
  `assert_numeric_match`). No primitive-specific math.
- `speed_matching.py` -- the worked example: the min-jerk quintic position
  shape (shared by `SpeedMatching.cc` and `SpeedMatchingSickness.cc`) and the
  min-time sextic (`SpeedMatching.cc` only). Run it directly; it prints the
  generated C++ and validates both against the literal hardcoded formulas.

## Known gap

The sextic's leading-term generalization for a jerk limit (`SpeedMatching`'s
`jmax_`, which divides coefficients `T_poly[0..5]` by `jmax^2`) isn't modeled
here -- `speed_matching.py` derives the base `jmax=1` math only. It's a
trivial constant-factor extension on top of the generated coefficients, not
re-derived math, so it wasn't worth complicating the generator for.
