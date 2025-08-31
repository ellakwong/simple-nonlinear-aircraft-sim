"""
Aircraft related structures for passing data around
"""

from dataclasses import dataclass


@dataclass
class AircraftPhysicalProperties:
    # Mass properties
    mass: float = 0.0  # kg

    # Moments of inertia about body axes
    Ixx: float = 0.0  # kg*m^2, roll inertia
    Iyy: float = 0.0  # kg*m^2, pitch inertia
    Izz: float = 0.0  # kg*m^2, yaw inertia
    Ixz: float = 0.0  # kg*m^2, product of inertia

    # Reference areas and lengths
    S: float = 0.0  # usually wing area
    c: float = 0.0  # usually mean chord length
    b: float = 0.0  # usually wingspan


@dataclass
class AircraftCoeffs:
    # For drag
    CD_0: float = 0.0
    CD_a: float = 0.0
    CD_de: float = 0.0

    # For lift
    CL_0: float = 0.0
    CL_a: float = 0.0
    CL_q: float = 0.0
    CL_de: float = 0.0

    # For pitching moment M
    Cm_0: float = 0.0
    Cm_a: float = 0.0
    Cm_q: float = 0.0
    Cm_de: float = 0.0

    # For side force Y
    CY_b: float = 0.0
    CY_p: float = 0.0
    CY_r: float = 0.0
    CY_da: float = 0.0
    CY_dr: float = 0.0

    # For rolling moment L
    Cl_b: float = 0.0
    Cl_p: float = 0.0
    Cl_r: float = 0.0
    Cl_da: float = 0.0
    Cl_dr: float = 0.0

    # For yawing moment N
    Cn_b: float = 0.0
    Cn_p: float = 0.0
    Cn_r: float = 0.0
    Cn_da: float = 0.0
    Cn_dr: float = 0.0


@dataclass
class AircraftState:
    # Position in flat-earth inertial frame
    altitude_m: float = 0.0
    x_m: float = 0.0
    y_m: float = 0.0

    # Euler angle orientation
    phi_rad: float = 0.0
    tht_rad: float = 0.0
    psi_rad: float = 0.0

    # Translational velocity in body frame
    u_m_s: float = 0.0
    v_m_s: float = 0.0
    w_m_s: float = 0.0

    # Rotational velocity in body frame
    p_rad_s: float = 0.0
    q_rad_s: float = 0.0
    r_rad_s: float = 0.0

    # Controls
    da_rad: float = 0.0
    de_rad: float = 0.0
    dr_rad: float = 0.0
    thrust_N: float = 0.0
