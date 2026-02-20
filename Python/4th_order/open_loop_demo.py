from source.aircraft import OpenLoop
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import os


def main():
    """
    Demonstrating use of Open Loop class for simulating aircraft
    longitudinal model.
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

    # Initial State
    x0 = np.array([alpha0, V0, q0, theta0])
    # ---------------- Time span ----------------
    tspan = (0.0, 14.0)

    model = OpenLoop(aero)

    def control_func(t):
        """Simulating constant control inputs"""
        dstab0 = -5.0           # deg (initial stabilator)
        thrust = 10000          # Thrust Newtons, will stay fixed
        cg     = 25             # c.g MAC%
        return np.array([dstab0, cg, thrust])
    

    t, x = model.simulate_eom(x0, control_func, tspan)
    # Its being simulated near trim so not much change going on

    # ---------------- Extract results ----------------
    alpha = x[0]
    V = x[1]
    q = x[2]
    theta = x[3]

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

    plt.show()


if __name__ == "__main__":
    main()