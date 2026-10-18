"""
Side-by-side velocity/acceleration comparison of SpeedMatching (min-jerk
min-time, jerk-squared cost) vs SpeedMatchingSickness (motion-sickness,
frequency-weighted-acceleration cost), across a shared w_T sweep.

Boundary conditions (v0=16, sT=50, vT=18, a0=aT=0) are the same ones used in
plot_mp_lon_mintminj.py and plot_motion_sickness_primitive.py (copied verbatim
from Plot_MP_LON_MINTMINJ.m), with the same tuning choices made there:
SpeedMatching uses jmax=20, SpeedMatchingSickness uses accel_scale=20.

Two stacked subplots (velocity, acceleration), both vs curvilinear abscissa s
(arc length, not time -- the two primitives have different T* for the same
w_T, but both traverse the same [0, sT] interval in space). Color encodes
w_T; line style encodes the primitive (solid = min-jerk min-time, dashed =
motion-sickness).
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import vprim_py as vprim
from plot_utils import legend_above

v0, a0 = 16.0, 0.0
sT, vT, aT = 50.0, 18.0, 0.0
start = vprim.LongitudinalState(s=0.0, v=v0, a=a0, j=0.0)
goal = vprim.LongitudinalState(s=sT, v=vT, a=aT, j=0.0)

JMAX = 20.0
ACCEL_SCALE = 20.0
WTs = [0.0, 0.3, 0.6, 1.0]

colors = cm.viridis(np.linspace(0.15, 0.9, len(WTs)))

fig, (ax_v, ax_a) = plt.subplots(2, 1, sharex=True, figsize=(7, 7))

for wT, color in zip(WTs, colors):
    wT_eff = max(wT, 1e-6)  # weight_time=0 degenerates the search edge case

    sm = vprim.SpeedMatching(weight_time=wT_eff)
    sm.set_jerk_limit(JMAX)
    sm_ok = sm.build(start, goal)

    sms = vprim.SpeedMatchingSickness(weight_time=wT_eff)
    sms.set_accel_scale(ACCEL_SCALE)
    sms.set_search_bounds(0.1, 15.0, 600)
    sms_ok = sms.build(start, goal)

    if sm_ok:
        t_u = np.linspace(0.0, sm.T, 151)
        states_u = [sm.eval(ti) for ti in t_u]
        s_u = np.array([st.s for st in states_u])
        v_u = np.array([st.v for st in states_u])
        a_u = np.array([st.a for st in states_u])
        label = rf"$w_T={wT:.1f}$ min-jerk min-T ($T^*={sm.T:.2f}$s)"
        ax_v.plot(s_u, v_u, color=color, lw=1.8, ls="-", label=label)
        ax_a.plot(s_u, a_u, color=color, lw=1.8, ls="-")
    else:
        print(f"[min-jerk min-T] failed to build for wT={wT:.2f}")

    if sms_ok:
        t_f = np.linspace(0.0, sms.T, 151)
        states_f = [sms.eval(ti) for ti in t_f]
        s_f = np.array([st.s for st in states_f])
        v_f = np.array([st.v for st in states_f])
        a_f = np.array([st.a for st in states_f])
        label = rf"$w_T={wT:.1f}$ sickness ($T^*={sms.T:.2f}$s)"
        ax_v.plot(s_f, v_f, color=color, lw=1.8, ls="--", label=label)
        ax_a.plot(s_f, a_f, color=color, lw=1.8, ls="--")
    else:
        print(f"[sickness] failed to build for wT={wT:.2f}")

ax_v.set_ylabel(r"$v_x$ [$m/s$]")
ax_v.set_xlim(0, sT)
ax_v.tick_params(labelbottom=False)  # shared x-axis: only the bottom panel labels it
ax_v.grid(True)
legend_above(ax_v, ncol=4, fontsize=6, y=1.12)

ax_a.set_ylabel(r"$a_x$ [$m/s^2$]")
ax_a.set_xlabel(r"$s$ [$m$]")
ax_a.set_xlim(0, sT)
ax_a.grid(True)

fig.suptitle(
    "Min-jerk min-time (solid, jmax=20) vs motion-sickness (dashed, accel_scale=20)\n"
    f"v0={v0:.0f}, sT={sT:.0f}, vT={vT:.0f}, $w_T \\in \\{{0, 0.3, 0.6, 1.0\\}}$",
    fontsize=10,
)
fig.tight_layout(rect=[0, 0, 1, 0.82])
fig.savefig("mintminj_vs_sickness_wT_sweep.png", dpi=150)

if __name__ == "__main__":
    plt.show()
