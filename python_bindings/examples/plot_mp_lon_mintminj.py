"""
Python/vprim_py replication of motionsick/matlab-old/TestAndPlot/Plot_MP_LON_MINTMINJ.m

Original MATLAB: SpeedMatching_aT, min-jerk + free-time (min-T) longitudinal primitive.
Sweeps the time weight w_T and plots v(s), a(s), j(s) for each weight.

Boundary conditions (v0=16, sT=50, vT=18, a0=aT=0) are copied verbatim from the
BCs still hardcoded at the top of Plot_MP_LON_MINTMINJ.m -- these do match the
archived figure (figures_export/MP_LON_MINTMINJ_states_exported_tikz.pdf).

NOTE on jerk_limit: SpeedMatching_aT.m's minimumTime() hardcodes j_max=1.0
internally (not a tunable property), so jmax=1 is the *literal* MATLAB cost. But
with jmax=1 and these BCs, the min-T sextic's smallest positive root (the true
cost-minimizer -- verified by evaluating J(T) at every candidate root) barely
moves at all as w_T goes 0->1 (T*: 2.9422s -> 2.9417s), giving six nearly
overlapping curves with no overshoot -- this does NOT match the archived figure,
which shows clear w_T-driven separation and velocity overshoot above v_T. The
sextic's only other candidate root for these BCs is a much higher-cost (~6-7x),
physically nonsensical trajectory (position overshoots to +90m and swings back
past the start before reaching s_T). So the archived figure could not have come
from today's hardcoded (BCs, j_max=1) pair -- some parameter (almost certainly
jmax, since the BCs already match) was lost when the .m file was edited after
that figure was produced.

jmax=20 was chosen to visually reproduce the archived figure's character
(overshoot, v_max~18.8-19, a in [-1.9, +2.5] m/s^2 for w_T=1) -- a deliberate
tuning choice for a representative example, NOT a literal MATLAB fidelity claim.
Lowering jmax back toward 1 recovers the literal (but visually flat) MATLAB
behavior; see motion_sickness_filter_minT_notes.md-style investigation in this
session's history for the full root/cost analysis.
"""

import numpy as np
import matplotlib.pyplot as plt
import vprim_py as vprim
from plot_utils import legend_above

# Boundary conditions: verbatim from Plot_MP_LON_MINTMINJ.m
v0, a0 = 16.0, 0.0
sT, vT, aT = 50.0, 18.0, 0.0

start = vprim.LongitudinalState(s=0.0, v=v0, a=a0, j=0.0)
goal = vprim.LongitudinalState(s=sT, v=vT, a=aT, j=0.0)

WTs = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
JMAX = 20.0

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(7, 7.5))

for wT in WTs:
    # weight_time=0 degenerates the sextic (drops a degree); nudge by epsilon so
    # the solver still finds the same (now near-unweighted) cost-minimizing root.
    sm = vprim.SpeedMatching(weight_time=max(wT, 1e-6))
    sm.set_jerk_limit(JMAX)
    if not sm.build(start, goal):
        print(f"Failed to build trajectory for wT = {wT:.2f}")
        continue

    T = sm.T
    t = np.linspace(0.0, T, 101)
    states = [sm.eval(ti) for ti in t]
    s = np.array([st.s for st in states])
    v = np.array([st.v for st in states])
    a = np.array([st.a for st in states])
    j = np.array([st.j for st in states])

    label = rf"$w_T={wT:.2f}$"
    ax1.plot(s, v, lw=1, label=label)
    ax2.plot(s, a, lw=1)
    ax3.plot(s, j, lw=1)

ax1.set_ylabel(r"$v_x$ [$m/s$]")
ax2.set_ylabel(r"$a_x$ [$m/s^2$]")
ax3.set_ylabel(r"$j_x$ [$m/s^3$]")
ax3.set_xlabel(r"$s$ [$m$]")

for ax in (ax1, ax2, ax3):
    ax.grid(True)
    ax.set_xlim(0, sT)
    lo, hi = ax.get_ylim()
    margin = 0.15 * (hi - lo)
    ax.set_ylim(lo - margin, hi + margin)

legend_above(ax1, ncol=3, fontsize=8, y=1.1)
fig.suptitle(
    f"MP_LON_MINTMINJ: min-jerk min-time, time-weight sweep\n"
    f"(v0={v0:.0f}, sT={sT:.0f}, vT={vT:.0f}, jmax={JMAX:.0f} -- tuned to match archived figure, see module docstring)",
    fontsize=10,
)
fig.tight_layout(rect=[0, 0, 1, 0.80])

if __name__ == "__main__":
    fig.savefig("MP_LON_MINTMINJ_states.png", dpi=150)
    plt.show()
