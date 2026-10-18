"""
Example / plots for the new SpeedMatchingSickness primitive (part 5 of the
motion-sickness roadmap, motionsick/motion_sickness_primitive_roadmap.md).

Two figures, mirroring the style of plot_mp_lon_mintminj.py (part 1) and of
motionsick/test_filter_vs_nofilter.mpl (the Maple comparison this primitive's
math was validated against):

  1. w_T sweep for the filtered (motion-sickness) primitive alone: v(s), a(s),
     a_w(s), j(s) -- same structure as plot_mp_lon_mintminj.py plus the new
     filtered-acceleration a_w track.

  2. Filtered vs unfiltered side-by-side comparison, same boundary conditions
     and w_T: SpeedMatching (jerk-squared cost, the library's existing
     primitive) vs SpeedMatchingSickness (frequency-weighted-acceleration
     cost). Reproduces the qualitative finding from
     motion_sickness_filter_minT_notes.md section 5: the filtered optimum
     trades more acceleration for a different duration, because the comfort
     term (a_w^2) is not the same quantity as the shape-defining cost (jx^2).

Boundary conditions (v0=16, sT=50, vT=18, a0=aT=0) match
plot_mp_lon_mintminj.py's, copied verbatim from Plot_MP_LON_MINTMINJ.m, for a
direct side-by-side comparison. SpeedMatching here uses jmax=20 (not the
literal jmax=1 in SpeedMatching_aT.m) to match the tuning chosen in
plot_mp_lon_mintminj.py's docstring.

Notable result at these BCs: with the literal cost (accel_scale=1), the
filtered primitive's T* is almost flat across the whole w_T sweep (~2.95s ->
~2.947s) and a_w stays small (~0.3-0.6 m/s^2) -- unlike the dramatic w_T
sensitivity seen at rest-to-rest/100m (T*: 26s -> 17.8s, see
src_tests/test_vprim_SpeedMatchingSickness.cc). This is physically meaningful,
not a bug: the ISO 2631-1 Wf filter's passband is ~0.1-0.5 Hz (periods of
~2-10s), but this maneuver's whole duration is ~3s, well below one filter
period, so the filter barely has time to respond before the manoeuvre ends --
fast cruise-speed adjustments carry little low-frequency "motion sickness"
content regardless of how aggressively they're executed.

ACCEL_SCALE divides the comfort term (integrated a_w^2) by a constant before
adding w_T*T (SpeedMatchingSickness::set_accel_scale, mirrors MDV_2.mw's
J_SM_bis := w__T + w__a*a(t)^2/50 normalization pattern), making comfort
cheaper relative to time so w_T actually matters on this short maneuver.
accel_scale=20 gives clear differentiation (T*: 2.9506s -> 2.8840s, a_max:
1.03 -> 1.46, aw_max: 0.61 -> 0.98) without going as far as the unfiltered
plot's visual drama (overshoot to v_max~18.8, a in [-1.1, +2.5] at jmax=20).
"""

import numpy as np
import matplotlib.pyplot as plt
import vprim_py as vprim
from plot_utils import legend_above

# ---------------------------------------------------------------------------
# Figure 1: w_T sweep, same BCs as plot_mp_lon_mintminj.py (v0=16, sT=50,
# vT=18, verbatim from Plot_MP_LON_MINTMINJ.m)
# ---------------------------------------------------------------------------

v0, a0 = 16.0, 0.0
sT, vT, aT = 50.0, 18.0, 0.0
start = vprim.LongitudinalState(s=0.0, v=v0, a=a0, j=0.0)
goal = vprim.LongitudinalState(s=sT, v=vT, a=aT, j=0.0)

WTs = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
ACCEL_SCALE = 20.0

fig1, axes1 = plt.subplots(4, 1, sharex=True, figsize=(7, 9.5))
ax_v, ax_a, ax_aw, ax_j = axes1

for wT in WTs:
    # weight_time=0 degenerates the stationarity scan's edge case; nudge by
    # epsilon, same trick as plot_mp_lon_mintminj.py.
    sms = vprim.SpeedMatchingSickness(weight_time=max(wT, 1e-6))
    # accel_scale=10 divides the comfort term (integrated a_w^2) by 10 before
    # adding w_T*T, making comfort cheaper relative to time -- without this,
    # a_w stays so small over this ~3s maneuver that T* barely moves at all
    # across the w_T sweep (see git history / this script's prior revision).
    sms.set_accel_scale(ACCEL_SCALE)
    # T* for these BCs is ~3s (much shorter than the rest-to-rest/100m case),
    # so a tighter, denser search interval than the notes.md default [0.5,25]s.
    sms.set_search_bounds(0.1, 15.0, 600)
    if not sms.build(start, goal):
        print(f"[sweep] failed to build for wT={wT:.2f}")
        continue
    T = sms.T
    t = np.linspace(0.0, T, 151)
    states = [sms.eval(ti) for ti in t]
    s = np.array([st.s for st in states])
    v = np.array([st.v for st in states])
    a = np.array([st.a for st in states])
    j = np.array([st.j for st in states])
    aw = np.array([sms.AW(ti) for ti in t])

    label = rf"$w_T={wT:.2f}$ ($T^*={T:.2f}$s)"
    ax_v.plot(s, v, lw=1, label=label)
    ax_a.plot(s, a, lw=1)
    ax_aw.plot(s, aw, lw=1)
    ax_j.plot(s, j, lw=1)

ax_v.set_ylabel(r"$v_x$ [$m/s$]")
ax_a.set_ylabel(r"$a_x$ [$m/s^2$]")
ax_aw.set_ylabel(r"$a_w$ [$m/s^2$]" "\n(filtered)")
ax_j.set_ylabel(r"$j_x$ [$m/s^3$]")
ax_j.set_xlabel(r"$s$ [$m$]")

for ax in axes1:
    ax.grid(True)
    ax.set_xlim(0, sT)
    lo, hi = ax.get_ylim()
    margin = 0.15 * (hi - lo)
    ax.set_ylim(lo - margin, hi + margin)

legend_above(ax_v, ncol=3, fontsize=7, y=1.1)
fig1.suptitle(
    f"SpeedMatchingSickness: time-weight sweep\n(v0={v0:.0f}, sT={sT:.0f}, vT={vT:.0f}, accel_scale={ACCEL_SCALE:.0f})",
    fontsize=10,
)
fig1.tight_layout(rect=[0, 0, 1, 0.80])
fig1.savefig("motion_sickness_sweep.png", dpi=150)

# ---------------------------------------------------------------------------
# Figure 2: filtered vs unfiltered, same BCs, w_T=1 (mirrors
# test_filter_vs_nofilter.mpl's run_case("r2r", ...))
# ---------------------------------------------------------------------------

wT_compare = 1.0

sm = vprim.SpeedMatching(weight_time=wT_compare)
sm.set_jerk_limit(20.0)
sm_ok = sm.build(start, goal)

sms = vprim.SpeedMatchingSickness(weight_time=wT_compare)
sms.set_accel_scale(ACCEL_SCALE)
sms.set_search_bounds(0.1, 15.0, 600)
sms_ok = sms.build(start, goal)

print(f"=== v0={v0:.0f}, sT={sT:.0f}, vT={vT:.0f}, w_T={wT_compare:.1f}: filtered vs unfiltered ===")
print(f"  unfiltered (SpeedMatching, jerk^2 cost):      T* = {sm.T:.4f} s   cost = {sm.get_cost():.4f}")
print(f"  filtered   (SpeedMatchingSickness, a_w^2 cost): T* = {sms.T:.4f} s   cost = {sms.get_cost():.4f}")

fig2, (bx_v, bx_a) = plt.subplots(2, 1, sharex=True, figsize=(7, 6.5))

# x-axis is curvilinear abscissa s (arc length), not time -- T* differs between
# the two primitives, but both traverse the same [0, sT] interval in space.
if sm_ok:
    t_u = np.linspace(0.0, sm.T, 151)
    states_u = [sm.eval(ti) for ti in t_u]
    s_u = np.array([st.s for st in states_u])
    v_u = np.array([st.v for st in states_u])
    a_u = np.array([st.a for st in states_u])
    bx_v.plot(s_u, v_u, color="tab:blue", lw=2, label=rf"unfiltered ($T^*={sm.T:.2f}$s)")
    bx_a.plot(s_u, a_u, color="tab:blue", lw=1, ls="--", label=r"$a$ (unfiltered)")

if sms_ok:
    t_f = np.linspace(0.0, sms.T, 151)
    states_f = [sms.eval(ti) for ti in t_f]
    s_f = np.array([st.s for st in states_f])
    v_f = np.array([st.v for st in states_f])
    a_f = np.array([st.a for st in states_f])
    aw_f = np.array([sms.AW(ti) for ti in t_f])
    bx_v.plot(s_f, v_f, color="tab:red", lw=2, label=rf"filtered ($T^*={sms.T:.2f}$s)")
    bx_a.plot(s_f, a_f, color="tab:red", lw=1, ls="--", label=r"$a$ (filtered)")
    bx_a.plot(s_f, aw_f, color="tab:red", lw=2, label=r"$a_w$ (filtered)")

bx_v.set_xlim(0, sT)
bx_v.tick_params(labelbottom=False)  # shared x-axis: only the bottom panel labels it
bx_v.set_ylabel(r"$v$ [$m/s$]")
bx_v.grid(True)
legend_above(bx_v, ncol=2, fontsize=8, y=1.15)

bx_a.set_xlabel(r"$s$ [$m$]")
bx_a.set_xlim(0, sT)
bx_a.set_ylabel(r"$m/s^2$")
bx_a.grid(True)
# Small (3-entry) legend: fits inside the panel's own empty upper-right
# corner (curves are near their negative trough there) -- "above axes"
# doesn't work for a non-top row, since that space is the gap to the row
# above it, not reserved margin (see bx_v's legend_above for the top row).
bx_a.legend(fontsize=7, loc="upper right")

fig2.suptitle(
    f"Filtered (motion-sickness, accel_scale={ACCEL_SCALE:.0f}) vs unfiltered (jerk-squared, jmax=20)\n"
    f"v0={v0:.0f}, sT={sT:.0f}, vT={vT:.0f}, $w_T$={wT_compare:.1f}",
    fontsize=10,
)
fig2.tight_layout(rect=[0, 0, 1, 0.80])
fig2.savefig("motion_sickness_vs_unfiltered.png", dpi=150)

if __name__ == "__main__":
    plt.show()
