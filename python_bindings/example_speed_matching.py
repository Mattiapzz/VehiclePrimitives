import vprim_py as vprim
import numpy as np
import matplotlib.pyplot as plt

def run_speed_matching_example():
    """
    Demonstrates the use of the SpeedMatching primitive, generates trajectories
    with different time weights, and plots the kinematic profiles.
    """
    # Define the start and goal longitudinal states (s, v, a, j)
    # Note: jerk (j) is an output, so its initial value is not used in the 'build' method.
    start_state = vprim.LongitudinalState(s=0, v=10, a=0, j=0)
    goal_state = vprim.LongitudinalState(s=100, v=20, a=0, j=0)

    # Define a range of weights for the time component of the cost function
    time_weights = np.linspace(0, 1, 5)

    # --- Data storage for plotting ---
    results_vs_t = []
    results_vs_s = []

    print("--- Building SpeedMatching Primitives ---")
    for wT in time_weights:
        sm = vprim.SpeedMatching(weight_time=wT)
        
        # Build the trajectory
        if sm.build(start_state, goal_state):
            T = sm.T
            print(f"Success for wT = {wT:.2f}: Optimal Time T = {T:.3f} s")

            # Generate time vector for evaluation
            t_vec = np.linspace(0, T, 150)
            
            # Evaluate the trajectory at each time step
            s_vec = np.array([sm.eval(t).s for t in t_vec])
            v_vec = np.array([sm.eval(t).v for t in t_vec])
            a_vec = np.array([sm.eval(t).a for t in t_vec])
            j_vec = np.array([sm.eval(t).j for t in t_vec])

            # Store results for plotting
            results_vs_t.append({'wT': wT, 't': t_vec, 'v': v_vec, 'a': a_vec, 'j': j_vec})
            results_vs_s.append({'wT': wT, 's': s_vec, 'v': v_vec, 'a': a_vec, 'j': j_vec})
        else:
            print(f"Failed to build trajectory for wT = {wT:.2f}")

    # --- Plotting ---

    # Figure 1: Kinematics vs. Time
    fig1, axes1 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig1.suptitle('Longitudinal Kinematics vs. Time', fontsize=16)
    
    for res in results_vs_t:
        label = f'wT = {res["wT"]:.1f}'
        axes1[0].plot(res['t'], res['v'], label=label)
        axes1[1].plot(res['t'], res['a'])
        axes1[2].plot(res['t'], res['j'])

    axes1[0].set_ylabel('Velocity (m/s)')
    axes1[1].set_ylabel('Acceleration (m/s²)')
    axes1[2].set_ylabel('Jerk (m/s³)')
    axes1[2].set_xlabel('Time (s)')
    
    for ax in axes1:
        ax.grid(True)
    
    # Set the x-axis limit to the maximum time across all trajectories to ensure
    # all plots are fully visible.
    if results_vs_t:
        max_t = max(res['t'][-1] for res in results_vs_t)
        axes1[-1].set_xlim(0, max_t)

    fig1.legend(loc='upper right')
    fig1.tight_layout(rect=[0, 0, 1, 0.96]) # Adjust for suptitle

    # Figure 2: Kinematics vs. Arc Length (s)
    fig2, axes2 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fig2.suptitle('Longitudinal Kinematics vs. Arc Length', fontsize=16)

    for res in results_vs_s:
        label = f'wT = {res["wT"]:.1f}'
        axes2[0].plot(res['s'], res['v'], label=label)
        axes2[1].plot(res['s'], res['a'])
        axes2[2].plot(res['s'], res['j'])

    axes2[0].set_ylabel('Velocity (m/s)')
    axes2[1].set_ylabel('Acceleration (m/s²)')
    axes2[2].set_ylabel('Jerk (m/s³)')
    axes2[2].set_xlabel('Arc Length (m)')
    
    for ax in axes2:
        ax.grid(True)
    
    # The final arc length is fixed by the goal state, so we can set the limit directly.
    axes2[-1].set_xlim(0, goal_state.s)
    
    fig2.legend(loc='upper right')
    fig2.tight_layout(rect=[0, 0, 1, 0.96])

    plt.show()

if __name__ == '__main__':
    run_speed_matching_example()