"""
Specific information for a Boeing 737-500.
Data adapted from JSBSim aircraft definition (737-500.xml).
Some coefficients are approximated or assumed due to lack of direct data.
"""

import aircraft


def get_737_500_info() -> (
    tuple[aircraft.AircraftPhysicalProperties, aircraft.AircraftCoeffs]
):
    # Physical properties
    properties = aircraft.AircraftPhysicalProperties()

    # Mass and inertia from <mass_balance>
    # Convert empty weight (lbs) to slugs (1 slug = 32.174 lb)
    #empty_weight_lbs = 69030
    #properties.mass = empty_weight_lbs / 32.174  # ~2146 slugs
    properties.mass = 60555 #kg

    properties.Ixx = 646725    # kgm^2
    properties.Iyy = 1464283
    properties.Izz = 1884586
    properties.Ixz = 8541

    # Wing geometry from <metrics>
    properties.S = 105.4      # m^2
    properties.c = 3.41        # mean chord (m)
    properties.b = 28.88        # wingspan (m)

    properties.max_thrust = 82000 #N
    properties.max_speed = 253 #N

    # Stability and control derivatives
    coeffs = aircraft.AircraftCoeffs()

    # Drag (from aero/CD0 etc.)
    coeffs.CD_0 = 0.021     # from table at alpha=0 xml
    coeffs.CD_a = 0.043     # approximate induced drag factor xml
    coeffs.CD_de = 0.059    # from elevator drag function xml

    # Lift (from CLalpha function)
    coeffs.CL_0 = 0.20      # at alpha=0 xml
    coeffs.CL_a = 5.2       # slope ~ ΔCL/Δα from XML table (-0.68 to 1.20 over 0.43 rad)
    coeffs.CL_q = 2.5 #estimation
    coeffs.CL_de = 0.2      # from CLde value

    # Pitching moment
    coeffs.Cm_0 = -0.1       # estimation
    coeffs.Cm_a = -0.6      # from Cmalpha function
    coeffs.Cm_q = -27.0     # from Cmq
    coeffs.Cm_de = -1.2     # approx from Cmde table at Mach 0

    # Side force derivatives
    coeffs.CY_b = -1.0      # from CYb product
    coeffs.CY_p = 0.05       # estimation
    coeffs.CY_r = -0.55       # estimation
    coeffs.CY_da = 0.02 #estimation
    coeffs.CY_dr = 0.35 #estimation

    # Roll moments
    coeffs.Cl_b = -0.09
    coeffs.Cl_p = -0.4
    coeffs.Cl_r = 0.09
    coeffs.Cl_da = 0.1      # from Clda table at Mach 0
    coeffs.Cl_dr = 0.01

    # Yaw moments
    coeffs.Cn_b = 0.26
    coeffs.Cn_p = 0.0       # not given
    coeffs.Cn_r = -0.35
    coeffs.Cn_da = -0.025 #estimation
    coeffs.Cn_dr = -0.2

    return properties, coeffs
