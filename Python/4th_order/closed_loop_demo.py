from source.aircraft import ClosedLoop
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import os
def main():
    """
    Demonstrating use of ClosedLoop class for simulating NDI
    controller for pitch control.
    """
    # ---------------- Load aero model ----------------
    dir_path = os.path.dirname(os.path.abspath(__file__))
    aero_data_path = os.path.join(dir_path, "all_data.npz")

    aero = np.load(aero_data_path)
    # ---------------- Initial conditions ----------------
    q0     = -0.0008        # rad/s
    alpha0 = 0.0577         # rad
    V0     = 266.2419       # m/s
    theta0 = -0.0090        # rad

    dstab0 = -5.0           # deg (initial stabilator)

    # Augmented state: [alpha, V, q, theta, dstab]
    z0 = np.array([alpha0, V0, q0, theta0, dstab0])

    # ---------------- Desired command ----------------
    theta_des = np.deg2rad(5.0) + theta0

    # ---------------- Time span ----------------
    tspan = (0.0, 14.0)
    dstab_rate = 60  # dstab max acuator rate, testing random values for now

    model = ClosedLoop(aero)

    sol = solve_ivp(
        lambda t, z: model.pitch_NDI(t, z, dstab_rate, theta_des, theta_dot_des=0, p_gain=1.0, d_gain=1.0),
        tspan,
        z0,
        # max_step=0.02,
        # rtol=1e-8,
        # atol=1e-10
    )
    X = sol.y[:4].T
    dstab_log = sol.y[4]

    # ---------------- Extract results ----------------
    t = sol.t
    alpha = sol.y[0]
    V = sol.y[1]
    q = sol.y[2]
    theta = sol.y[3]
    dstab = sol.y[4]

    # ---------------- Plots ----------------
    fig, axs = plt.subplots(4, 1, sharex=True)

    axs[0].plot(t, np.rad2deg(alpha))
    axs[0].set_ylabel(r'$\alpha$ [deg]')

    axs[1].plot(t, V)
    axs[1].set_ylabel('V [m/s]')

    axs[2].plot(t, np.rad2deg(q))
    axs[2].set_ylabel('q [deg/s]')

    axs[3].plot(t, np.rad2deg(theta))
    axs[3].set_ylabel(r'$\theta$ [deg]')
    axs[3].set_xlabel('Time [s]')

    # ---- Tracking ----
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(t, np.rad2deg(theta), label='Actual')
    plt.axhline(np.rad2deg(theta_des), color='r', linestyle='--', label='Desired')
    plt.ylabel(r'$\theta$ [deg]')
    plt.legend()
    plt.title('Pitch Tracking')

    plt.subplot(2,1,2)
    plt.plot(t, dstab)
    plt.ylabel('Stabilator [deg]')
    plt.xlabel('Time [s]')
    plt.title('Control Input')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()