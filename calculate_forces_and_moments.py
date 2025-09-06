"""
Calculates the forces and moments in the body frame
based on the current aircraft state.
"""

import numpy as np

from aircraft import AircraftState, AircraftCoeffs, AircraftPhysicalProperties
from atmosphere import ussa1976


def calculate(
    state: AircraftState, coeffs: AircraftCoeffs, props: AircraftPhysicalProperties
):
    """
    Calculates the body frame forces and moments: FX, FY, FZ, L, M, N
    """
    # Things we'll need, derived from the state
    Vtas = np.sqrt(state.u_m_s**2 + state.v_m_s**2 + state.w_m_s**2)
    bta = np.arcsin(state.v_m_s / Vtas)
    aph = np.arcsin(state.w_m_s / (Vtas * np.cos(bta)))
    rho = ussa1976(state.altitude_m)
    qbar = 0.5 * rho * Vtas**2
    g = 9.81

    # Aliases
    S = props.S
    c = props.c
    b = props.b
    m = props.mass

    # The coefficients are typically derived using non-dimensionalised states, which is why the p, q, r parts
    # have extra stuff going on in these expansions to non-dimensionalise them.
    # aph, bta and the control deflections are already non-dimensional.

    # Lift
    CL = (
        coeffs.CL_0
        + coeffs.CL_a * aph
        + coeffs.CL_q * state.q_rad_s * c / (2 * Vtas)
        + coeffs.CL_de * state.de_rad
    )
    Lift = CL * qbar * S

    # Drag
    CD = coeffs.CD_0 + coeffs.CD_a * aph + coeffs.CD_de * state.de_rad
    D = CD * qbar * S

    # Side force
    CY = (
        coeffs.CY_b * bta
        + coeffs.CY_p * state.p_rad_s * b / (2 * Vtas)
        + coeffs.CY_r * state.r_rad_s * b / (2 * Vtas)
        + coeffs.CY_da * state.da_rad
        + coeffs.CY_dr * state.dr_rad
    )
    FAY = CY * qbar * S

    # Rolling Moment L
    Cl = (
        coeffs.Cl_b * bta
        + coeffs.Cl_p * state.p_rad_s * b / (2 * Vtas)
        + coeffs.Cl_r * state.r_rad_s * b / (2 * Vtas)
        + coeffs.Cl_da * state.da_rad
        + coeffs.Cl_dr * state.dr_rad
    )
    L = Cl * qbar * S * b  # this was overwriting lift before oops..

    # Pitching Moment M
    Cm = (
        coeffs.Cm_0
        + coeffs.Cm_a * aph
        + coeffs.Cm_q * state.q_rad_s * c / (2 * Vtas)
        + coeffs.Cm_de * state.de_rad
    )
    M = Cm * qbar * S * c

    # Yawing Moment N
    Cn = (
        coeffs.Cn_b * bta
        + coeffs.Cn_p * state.p_rad_s * b / (2 * Vtas)
        + coeffs.Cn_r * state.r_rad_s * b / (2 * Vtas)
        + coeffs.Cn_da * state.da_rad
        + coeffs.Cn_dr * state.dr_rad
    )
    N = Cn * qbar * S * b

    # Transform lift and drag (which are defined in stability frame) into body frame
    FAX = -D * np.cos(aph) + Lift * np.sin(aph)
    FAZ = -Lift * np.cos(aph) - D * np.sin(aph)

    # Thrust force is just specified directly in the state to be simple, we will assume it acts in body X direction
    FTX = state.thrust_N

    # Sum aero + thrust + gravity forces in body frame
    FX = FAX + FTX - m * g * np.sin(state.tht_rad)
    FY = FAY + m * g * np.sin(state.phi_rad) * np.cos(state.tht_rad)
    FZ = FAZ + m * g * np.cos(state.phi_rad) * np.cos(state.tht_rad)

    return (FX, FY, FZ, L, M, N)
