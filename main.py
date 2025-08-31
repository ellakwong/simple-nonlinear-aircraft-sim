"""
This non-linear 6DOF simulator is has been made as a sort of
minimum viable simulator without all the complexity and capability
of other simulation tools you may have found and tried to use.
It uses the flat Earth equations of motion.

It is initially set up with the same aircraft and a very similar
configuration to my aircraft stability tool found here: https://simmeon.github.io/aircraft-stability/

You should be able to compare the response from both tools and they should
(in theory, if I haven't messed up) be similar. But there will and should
be differences since this code uses non-linear equations while the website
uses linear equations of motion.

Hopefully this code is set out in a way that is readable, decently
well commented, and can help you understand the main steps that
go into an aircraft simulation.

"""

import numpy as np
import matplotlib.pyplot as plt
import copy

import cessnalike_aircraft
import solver
import trimmer
from aircraft import AircraftState

RAD2DEG = 180 / np.pi
DEG2RAD = np.pi / 180


# Storage arrays
t_hist = []
x_hist, y_hist, alt_hist = [], [], []
phi_hist, tht_hist, psi_hist = [], [], []
u_hist, v_hist, w_hist = [], [], []
p_hist, q_hist, r_hist = [], [], []
de_hist, da_hist, dr_hist, thrust_hist = [], [], [], []


def log_state(t, state: AircraftState):
    t_hist.append(t)
    x_hist.append(state.x_m)
    y_hist.append(state.y_m)
    alt_hist.append(state.altitude_m)

    phi_hist.append(state.phi_rad * RAD2DEG)
    tht_hist.append(state.tht_rad * RAD2DEG)
    psi_hist.append(state.psi_rad * RAD2DEG)

    u_hist.append(state.u_m_s)
    v_hist.append(state.v_m_s)
    w_hist.append(state.w_m_s)

    p_hist.append(state.p_rad_s * RAD2DEG)
    q_hist.append(state.q_rad_s * RAD2DEG)
    r_hist.append(state.r_rad_s * RAD2DEG)

    de_hist.append(state.de_rad * RAD2DEG)
    da_hist.append(state.da_rad * RAD2DEG)
    dr_hist.append(state.dr_rad * RAD2DEG)
    thrust_hist.append(state.thrust_N)


def run_sim():
    # Physical properties and aerodynamic coefficients for a Cessna-like aircraft
    cessna_properties, cessna_coeffs = cessnalike_aircraft.get_cessna_info()

    # Initial conditions to find trim at
    altitude_m_trim = 1524
    tas_m_s_trim = 67
    fpa_rad_trim = 0.0

    # Trim for initial state
    trimmed_state = trimmer.trim(
        altitude_m_trim, tas_m_s_trim, fpa_rad_trim, cessna_properties, cessna_coeffs
    )

    # To preserve trim state for reference, we'll make a copy to use for simulation
    state = copy.deepcopy(trimmed_state)

    # Simulation
    dt = 0.01
    T = 60
    t = 0.0

    while t <= T:
        # Log current state for plotting
        log_state(t, state)

        # Add elevator pulse 10–12 s into sim
        if 10 <= t <= 12:
            state.de_rad = trimmed_state.de_rad - 2 * DEG2RAD
        else:
            state.de_rad = trimmed_state.de_rad

        # Step simulation forward
        state = solver.step(dt, state, cessna_coeffs, cessna_properties)
        t += dt


def plot_response():
    # === Plotting ===
    fig, axs = plt.subplots(5, 1, figsize=(10, 14), sharex=True)

    # Position / altitude
    axs[0].plot(t_hist, alt_hist, label="Altitude [m]")
    axs[0].plot(t_hist, x_hist, label="x [m]")
    axs[0].plot(t_hist, y_hist, label="y [m]")
    axs[0].set_ylabel("Position")
    axs[0].legend()
    axs[0].grid()

    # Attitude angles
    axs[1].plot(t_hist, phi_hist, label="Roll [deg]")
    axs[1].plot(t_hist, tht_hist, label="Pitch [deg]")
    axs[1].plot(t_hist, psi_hist, label="Yaw [deg]")
    axs[1].set_ylabel("Angles")
    axs[1].legend()
    axs[1].grid()

    # Body velocities
    axs[2].plot(t_hist, u_hist, label="u [m/s]")
    axs[2].plot(t_hist, v_hist, label="v [m/s]")
    axs[2].plot(t_hist, w_hist, label="w [m/s]")
    axs[2].set_ylabel("Velocities")
    axs[2].legend()
    axs[2].grid()

    # Angular rates
    axs[3].plot(t_hist, p_hist, label="p [deg/s]")
    axs[3].plot(t_hist, q_hist, label="q [deg/s]")
    axs[3].plot(t_hist, r_hist, label="r [deg/s]")
    axs[3].set_ylabel("Rates")
    axs[3].legend()
    axs[3].grid()

    # Control inputs
    # Left y-axis for control surfaces
    ax_ctrl = axs[4]
    ax_ctrl.plot(t_hist, de_hist, label="Elevator [deg]")
    ax_ctrl.plot(t_hist, da_hist, label="Aileron [deg]")
    ax_ctrl.plot(t_hist, dr_hist, label="Rudder [deg]")
    ax_ctrl.set_ylabel("Control deflections [deg]")
    ax_ctrl.legend(loc="upper left")
    ax_ctrl.grid()

    # Right y-axis for thrust
    ax_thrust = ax_ctrl.twinx()
    ax_thrust.plot(t_hist, thrust_hist, color="k", linestyle="--", label="Thrust [N]")
    ax_thrust.set_ylabel("Thrust [N]")
    ax_thrust.legend(loc="upper right")

    axs[4].set_xlabel("Time [s]")

    plt.tight_layout()
    plt.show()


def main():
    run_sim()
    plot_response()


if __name__ == "__main__":
    main()
