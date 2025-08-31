"""
Simple trim algorithm.
"""

from aircraft import AircraftState, AircraftCoeffs, AircraftPhysicalProperties
from atmosphere import ussa1976
from math import sin, cos
from scipy.optimize import minimize


def trim(
    altitude_m: float,
    tas_m_s: float,
    fpa_rad: float,
    props: AircraftPhysicalProperties,
    coeffs: AircraftCoeffs,
    x0: list = [0, 0, 0],
):
    """
    Find a trimmed aircraft state given an altitude, true airspeed, and flight path angle.
    """
    rho = ussa1976(altitude_m)
    sol = minimize(
        J,
        x0=x0,
        args=(rho, tas_m_s, fpa_rad, props, coeffs),
        method="Nelder-Mead",
        tol=1e-8,
    )

    aph_rad = sol.x[0]
    de_rad = sol.x[1]
    thrust_N = sol.x[2]

    u_m_s = tas_m_s * cos(aph_rad)
    w_m_s = tas_m_s * sin(aph_rad)
    tht_rad = fpa_rad + aph_rad

    trimmed_state = AircraftState()
    trimmed_state.altitude_m = altitude_m
    trimmed_state.tht_rad = tht_rad
    trimmed_state.u_m_s = u_m_s
    trimmed_state.w_m_s = w_m_s
    trimmed_state.de_rad = de_rad
    trimmed_state.thrust_N = thrust_N

    return trimmed_state


def J(
    x: list,
    rho: float,
    tas_m_s: float,
    fpa_rad: float,
    props: AircraftPhysicalProperties,
    coeffs: AircraftCoeffs,
) -> float:
    # States to solve for
    aph = x[0]
    de = x[1]
    FTX1 = x[2]

    fpa = fpa_rad  # set fpa to given value

    # Derived states
    tht = aph + fpa

    # Derived values
    qbar = 0.5 * rho * tas_m_s**2

    # Aliases
    m = props.mass
    S = props.S
    c = props.c

    CD_0 = coeffs.CD_0
    CD_a = coeffs.CD_a
    CD_de = coeffs.CD_de

    CL_0 = coeffs.CL_0
    CL_a = coeffs.CL_a
    CL_de = coeffs.CL_de

    Cm_0 = coeffs.Cm_0
    Cm_a = coeffs.Cm_a
    Cm_de = coeffs.Cm_de

    # Gravity
    g = 9.81

    # Steady state coefficients
    CD_1 = CD_0 + CD_a * aph + CD_de * de
    CL_1 = CL_0 + CL_a * aph + CL_de * de
    Cm_1 = Cm_0 + Cm_a * aph + Cm_de * de

    # Force/Moment Calculations
    D1 = CD_1 * qbar * S
    L1 = CL_1 * qbar * S

    FAX1 = -D1 * cos(aph) + L1 * sin(aph)

    FX1 = FAX1 + FTX1 - m * g * sin(tht)

    FZ1 = -L1 * cos(aph) - D1 * sin(aph) + m * g * cos(tht)
    M1 = Cm_1 * qbar * S * c

    return FX1**2 + FZ1**2 + M1**2
