"""
Specific information for an aircraft like a Cessna 182.
From Appendix B of Jan Roskam - Airplane Flight Dynamics and Automatic Flight Controls
"""

import aircraft


def get_cessna_info() -> (
    tuple[aircraft.AircraftPhysicalProperties, aircraft.AircraftCoeffs]
):
    # Physical properties
    properties = aircraft.AircraftPhysicalProperties()

    properties.mass = 1202

    properties.Ixx = 1285
    properties.Iyy = 1825
    properties.Izz = 2667
    properties.Ixz = 0

    properties.S = 16.16
    properties.c = 1.5
    properties.b = 11

    # Stability and control derivatives
    coeffs = aircraft.AircraftCoeffs()

    coeffs.CD_0 = 0.027
    coeffs.CD_a = 0.38  # pretty non-linear and changes at different aph, etc.
    coeffs.CD_de = 0.0

    coeffs.CL_0 = 0.307
    coeffs.CL_a = 4.41
    coeffs.CL_q = 3.9
    coeffs.CL_de = 0.43

    coeffs.Cm_0 = 0.04
    coeffs.Cm_a = -0.65
    coeffs.Cm_q = -15.2
    coeffs.Cm_de = -1.369

    coeffs.CY_b = -0.404
    coeffs.CY_p = -0.145
    coeffs.CY_r = 0.267
    coeffs.CY_da = 0
    coeffs.CY_dr = 0.187

    coeffs.Cl_b = -0.0895
    coeffs.Cl_p = -0.487
    coeffs.Cl_r = 0.1869
    coeffs.Cl_da = 0.229
    coeffs.Cl_dr = 0.0147

    coeffs.Cn_b = 0.0907
    coeffs.Cn_p = -0.0649
    coeffs.Cn_r = -0.1199
    coeffs.Cn_da = -0.0504
    coeffs.Cn_dr = -0.0805

    return properties, coeffs
