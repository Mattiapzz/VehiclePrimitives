"""
Example / plots for the new ComfortMatching primitive: the JOINTLY optimal
motion-sickness primitive (filter baked directly into the free-time OCP's
running cost), as opposed to SpeedMatchingSickness's decoupled approximation
(reuse SpeedMatching's fixed min-jerk shape, only change duration selection).
See src/include/ComfortMatching.hh for the full derivation and
motionsick/motion_sickness_primitive_roadmap.md for the design history.

Boundary conditions: rest-to-rest, 100 m -- NOT the v0=16/sT=50/vT=18 case used
in plot_mp_lon_mintminj.py / plot_motion_sickness_primitive.py. That ~3s
maneuver is too fast for either comfort primitive to show much w_T
sensitivity (the filter never gets a full cycle to respond -- see
plot_motion_sickness_primitive.py's docstring). The rest-to-rest/100m case
(~10-20s maneuvers) is where the comfort-vs-urgency trade-off this primitive
exists for is actually visible, and is the case the joint-vs-decoupled design
validation (T*: 20.1s, 18% lower integrated a_w^2) was run on.

Cost: w_T*T + w_j*(jx/jmax)^2 + w_a*(a_w/a_max)^2 dt, with jmax=10 (matches
SpeedMatching's own default jerk limit) and a_max=10 (matches the MATLAB
SpeedMatching_aT.m ax_max convention) as physically-grounded normalizations,
rather than the dimensionless accel_scale tuning SpeedMatchingSickness needed.

Two figures:
  1. w_T sweep for ComfortMatching alone: v(s), a(s), a_w(s), j(s).
  2. Three-way comparison at w_T=1: SpeedMatching (no comfort term) vs
     SpeedMatchingSickness (decoupled approximation) vs ComfortMatching
     (jointly optimal), each at its own T*. Demonstrates the validated
     finding: ComfortMatching achieves lower integrated a_w^2 than the
     decoupled primitive even though both have access to the same filter.
"""

import numpy as np
import matplotlib.pyplot as plt
import vprim_py as vprim
from plot_utils import legend_above

v0, a0 = 0.0, 0.0
sT, vT, aT = 100.0, 0.0, 0.0
start = vprim.LongitudinalState(s=0.0, v=v0, a=a0, j=0.0)
goal = vprim.LongitudinalState(s=sT, v=vT, a=aT, j=0.0)

JMAX = 10.0
A_MAX = 10.0

# ---------------------------------------------------------------------------
# Figure 1: w_T sweep, ComfortMatching alone
# ---------------------------------------------------------------------------

WTs = [0.1, 0.3, 0.6, 1.0]

fig1, axes1 = plt.subplots(4, 1, sharex=True, figsize=(7, 9.5))
ax_v, ax_a, ax_aw, ax_j = axes1

for wT in WTs:
    cm = vprim.ComfortMatching(weight_time=wT)
    cm.set_jerk_limit(JMAX)
    cm.set_accel_limit(A_MAX)
    cm.set_accel_weight(1.0)
    cm.set_search_bounds(0.5, 25.0, 1200)
    if not cm.build(start, goal):
        print(f"[sweep] failed to build for wT={wT:.2f}")
        continue
    T = cm.T
    t = np.linspace(0.0, T, 201)
    states = [cm.eval(ti) for ti in t]
    s = np.array([st.s for st in states])
    v = np.array([st.v for st in states])
    a = np.array([st.a for st in states])
    j = np.array([st.j for st in states])
    aw = np.array([cm.AW(ti) for ti in t])

    label = rf"$w_T={wT:.2f}$ ($T^*={T:.2f}$s)"
    ax_v.plot(s, v, lw=1.2, label=label)
    ax_a.plot(s, a, lw=1.2)
    ax_aw.plot(s, aw, lw=1.2)
    ax_j.plot(s, j, lw=1.2)

ax_v.set_ylabel(r"$v_x$ [$m/s$]")
ax_a.set_ylabel(r"$a_x$ [$m/s^2$]")
ax_aw.set_ylabel(r"$a_w$ [$m/s^2$]" "\n(filtered)")
ax_j.set_ylabel(r"$j_x$ [$m/s^3$]")
ax_j.set_xlabel(r"$s$ [$m$]")

for ax in axes1:
    ax.grid(True)
    ax.set_xlim(0, sT)

legend_above(ax_v, ncol=2, fontsize=7, y=1.1)
fig1.suptitle(
    f"ComfortMatching: time-weight sweep (jointly optimal)\n"
    f"(rest-to-rest, {sT:.0f} m, jmax={JMAX:.0f}, a_max={A_MAX:.0f}, w_j=w_a=1)",
    fontsize=10,
)
fig1.tight_layout(rect=[0, 0, 1, 0.82])
fig1.savefig("comfort_matching_sweep.png", dpi=150)

# ---------------------------------------------------------------------------
# Figure 2: three-way comparison at w_T=1
# ---------------------------------------------------------------------------

wT_compare = 1.0

sm = vprim.SpeedMatching(weight_time=wT_compare)
sm.set_jerk_limit(JMAX)
sm_ok = sm.build(start, goal)

cm = vprim.ComfortMatching(weight_time=wT_compare)
cm.set_jerk_limit(JMAX)
cm.set_accel_limit(A_MAX)
cm.set_accel_weight(1.0)
cm.set_search_bounds(0.5, 25.0, 1200)
cm_ok = cm.build(start, goal)

# Match SpeedMatchingSickness's accel_scale to ComfortMatching's T* by
# bisection, so the decoupled-vs-joint comparison below is apples-to-apples
# (same duration, same ballpark of "how hard are we discouraging acceleration")
# instead of three independently-chosen, incomparable tunings. Mirrors the
# design-validation methodology (motionsick scratchpad prototypes).
assert cm_ok, "ComfortMatching must build to calibrate the comparison"


def decoupled_T(accel_scale):
    sms_try = vprim.SpeedMatchingSickness(weight_time=wT_compare)
    sms_try.set_accel_scale(accel_scale)
    sms_try.set_search_bounds(0.5, 40.0, 1200)
    return sms_try.T if sms_try.build(start, goal) else None


lo, hi = 0.01, 5000.0
for _ in range(60):
    mid = (lo + hi) / 2
    Td = decoupled_T(mid)
    if Td is None:
        hi = mid
        continue
    if Td > cm.T:
        lo = mid
    else:
        hi = mid
accel_scale_matched = (lo + hi) / 2

sms = vprim.SpeedMatchingSickness(weight_time=wT_compare)
sms.set_accel_scale(accel_scale_matched)
sms.set_search_bounds(0.5, 40.0, 1200)
sms_ok = sms.build(start, goal)
print(f"  (SpeedMatchingSickness accel_scale={accel_scale_matched:.4f} chosen to match ComfortMatching's T*={cm.T:.4f}s)")


def integrated_aw_sq(eval_aw, T, n=400):
    t = np.linspace(0.0, T, n + 1)
    aw = np.array([eval_aw(ti) for ti in t])
    return np.trapezoid(aw**2, t)


print(f"=== rest-to-rest, {sT:.0f} m, w_T={wT_compare:.1f}: three-way comparison ===")
if sm_ok:
    print(f"  SpeedMatching (no comfort term):         T* = {sm.T:.4f} s")
if sms_ok:
    aw_sms = integrated_aw_sq(sms.AW, sms.T)
    print(f"  SpeedMatchingSickness (decoupled):        T* = {sms.T:.4f} s   int(a_w^2) = {aw_sms:.4f}")
if cm_ok:
    aw_cm = integrated_aw_sq(cm.AW, cm.T)
    print(f"  ComfortMatching (jointly optimal):         T* = {cm.T:.4f} s   int(a_w^2) = {aw_cm:.4f}")

fig2, (bx_v, bx_a, bx_aw) = plt.subplots(3, 1, sharex=True, figsize=(7, 9))

if sm_ok:
    t_u = np.linspace(0.0, sm.T, 201)
    states_u = [sm.eval(ti) for ti in t_u]
    s_u = np.array([st.s for st in states_u])
    v_u = np.array([st.v for st in states_u])
    a_u = np.array([st.a for st in states_u])
    bx_v.plot(s_u, v_u, color="tab:blue", lw=2, label=rf"SpeedMatching ($T^*={sm.T:.2f}$s)")
    bx_a.plot(s_u, a_u, color="tab:blue", lw=1.5, label="SpeedMatching")

if sms_ok:
    t_d = np.linspace(0.0, sms.T, 201)
    states_d = [sms.eval(ti) for ti in t_d]
    s_d = np.array([st.s for st in states_d])
    v_d = np.array([st.v for st in states_d])
    a_d = np.array([st.a for st in states_d])
    aw_d = np.array([sms.AW(ti) for ti in t_d])
    bx_v.plot(s_d, v_d, color="tab:orange", lw=2, label=rf"SpeedMatchingSickness ($T^*={sms.T:.2f}$s)")
    bx_a.plot(s_d, a_d, color="tab:orange", lw=1.5, label="SpeedMatchingSickness")
    bx_aw.plot(s_d, aw_d, color="tab:orange", lw=1.5, label="SpeedMatchingSickness (decoupled)")

if cm_ok:
    t_j = np.linspace(0.0, cm.T, 201)
    states_j = [cm.eval(ti) for ti in t_j]
    s_j = np.array([st.s for st in states_j])
    v_j = np.array([st.v for st in states_j])
    a_j = np.array([st.a for st in states_j])
    aw_j = np.array([cm.AW(ti) for ti in t_j])
    bx_v.plot(s_j, v_j, color="tab:red", lw=2, label=rf"ComfortMatching ($T^*={cm.T:.2f}$s)")
    bx_a.plot(s_j, a_j, color="tab:red", lw=1.5, label="ComfortMatching")
    bx_aw.plot(s_j, aw_j, color="tab:red", lw=1.5, label="ComfortMatching (jointly optimal)")

bx_v.set_xlim(0, sT)
bx_v.tick_params(labelbottom=False)  # shared x-axis: only the bottom panel labels it
bx_v.set_ylabel(r"$v$ [$m/s$]")
bx_v.grid(True)
legend_above(bx_v, ncol=1, fontsize=8, y=1.15)

bx_a.set_xlim(0, sT)
bx_a.tick_params(labelbottom=False)
bx_a.set_ylabel(r"$a$ [$m/s^2$]")
bx_a.grid(True); bx_a.legend(fontsize=7, loc="upper right")

bx_aw.set_xlabel(r"$s$ [$m$]"); bx_aw.set_xlim(0, sT)
bx_aw.set_ylabel(r"$a_w$ [$m/s^2$]")
bx_aw.grid(True); bx_aw.legend(fontsize=7, loc="upper right")

fig2.suptitle(
    f"SpeedMatching vs SpeedMatchingSickness (decoupled) vs ComfortMatching (jointly optimal)\n"
    f"rest-to-rest, {sT:.0f} m, $w_T$={wT_compare:.1f}",
    fontsize=10,
)
fig2.tight_layout(rect=[0, 0, 1, 0.84])
fig2.savefig("comfort_matching_comparison.png", dpi=150)

if __name__ == "__main__":
    plt.show()
