"""
Solve the flat earth equations of motion for a given vehicle.
"""

import numpy as np
from aircraft import AircraftState, AircraftCoeffs, AircraftPhysicalProperties

import calculate_forces_and_moments


def dxdt(
    state: AircraftState, coeffs: AircraftCoeffs, props: AircraftPhysicalProperties
):
    """
    Calculate the state derivative based on the current state.
    Note: I'm doing most of this in vector form because that's just nicer to work with,
    and the form of these equations is mostly in the body frame.
    """
    # Calculate forces/moments
    FX, FY, FZ, Mx, My, Mz = calculate_forces_and_moments.calculate(
        state, coeffs, props
    )
    F = np.array([FX, FY, FZ])
    M = np.array([Mx, My, Mz])

    # Calculate inertia matrix
    I = np.array(
        [[props.Ixx, 0, -props.Ixz], [0, props.Iyy, 0], [-props.Ixz, 0, props.Izz]]
    )

    # Body frame velocity vector
    v_b = np.array([state.u_m_s, state.v_m_s, state.w_m_s])

    # Body frame angular velocity vector
    w_b = np.array([state.p_rad_s, state.q_rad_s, state.r_rad_s])

    # Kinematic ZYX orientation matrix
    c_phi = np.cos(state.phi_rad)
    s_phi = np.sin(state.phi_rad)
    c_tht = np.cos(state.tht_rad)
    s_tht = np.sin(state.tht_rad)
    c_psi = np.cos(state.psi_rad)
    s_psi = np.sin(state.psi_rad)

    R_kinematic = np.linalg.inv(
        np.array(
            [[1, 0, -s_tht], [0, c_phi, s_phi * c_tht], [0, -s_phi, c_phi * c_tht]]
        )
    )

    # Body frame to inertial frame ZYX rotation matrix
    R_bi = np.array(
        [
            [c_tht * c_psi, c_tht * s_psi, -s_tht],
            [
                s_phi * s_tht * c_psi - c_phi * s_psi,
                c_phi * c_psi + s_phi * s_tht * s_psi,
                s_phi * c_tht,
            ],
            [
                s_phi * s_psi + c_phi * s_tht * c_psi,
                -s_phi * c_psi + c_phi * s_tht * s_psi,
                c_phi * c_tht,
            ],
        ]
    )  # to body, from inertial
    R_ib = np.linalg.inv(R_bi)  # to inertial, from body

    # Derivatives calculation
    Pdot = R_ib @ v_b  # where P = [x, y, z], z = -alt
    Omegadot = R_kinematic @ w_b  # where Omega = [phi, tht, psi]
    vdot_b = 1 / props.mass * F - np.cross(w_b, v_b)
    wdot_b = np.linalg.inv(I) @ (M - np.cross(w_b, I @ w_b))

    xdot = Pdot[0]
    ydot = Pdot[1]
    zdot = Pdot[2]

    phidot = Omegadot[0]
    thtdot = Omegadot[1]
    psidot = Omegadot[2]

    udot = vdot_b[0]
    vdot = vdot_b[1]
    wdot = vdot_b[2]

    pdot = wdot_b[0]
    qdot = wdot_b[1]
    rdot = wdot_b[2]

    return np.array(
        [xdot, ydot, zdot, phidot, thtdot, psidot, udot, vdot, wdot, pdot, qdot, rdot]
    )


def step(
    dt: float,
    state: AircraftState,
    coeffs: AircraftCoeffs,
    props: AircraftPhysicalProperties,
):
    """
    Take an integration step based on current state and aircraft properties. Returns new state.
    """
    # Old state in vector form - to be compatible with state derivative vector form
    x_old = np.array(
        [
            state.x_m,
            state.y_m,
            -state.altitude_m,
            state.phi_rad,
            state.tht_rad,
            state.psi_rad,
            state.u_m_s,
            state.v_m_s,
            state.w_m_s,
            state.p_rad_s,
            state.q_rad_s,
            state.r_rad_s,
        ]
    )

    # Calculate state derivative
    xdot = dxdt(state, coeffs, props)

    # Euler integration step
    x_new = x_old + dt * xdot

    # Put new state info into AircraftState struct
    state.x_m = x_new[0]
    state.y_m = x_new[1]
    state.altitude_m = -x_new[2]

    state.phi_rad = x_new[3]
    state.tht_rad = x_new[4]
    state.psi_rad = x_new[5]

    state.u_m_s = x_new[6]
    state.v_m_s = x_new[7]
    state.w_m_s = x_new[8]

    state.p_rad_s = x_new[9]
    state.q_rad_s = x_new[10]
    state.r_rad_s = x_new[11]

    return state
