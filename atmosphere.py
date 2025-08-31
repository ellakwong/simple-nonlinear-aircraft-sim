import math


def ussa1976(h):
    """
    Returns air density [kg/m^3] from the U.S. Standard Atmosphere 1976 model
    for a given geometric altitude h [m].
    Valid up to 86 km.
    """
    # Constants
    g0 = 9.80665  # m/s^2
    R = 287.05287  # J/(kg*K), specific gas constant for dry air

    # Define atmospheric layers (base geopotential height [m], base temperature [K], lapse rate [K/m], base pressure [Pa])
    layers = [
        (0, 288.15, -0.0065, 101325.0),
        (11000, 216.65, 0.0, 22632.06),
        (20000, 216.65, 0.001, 5474.889),
        (32000, 228.65, 0.0028, 868.019),
        (47000, 270.65, 0.0, 110.906),
        (51000, 270.65, -0.0028, 66.9389),
        (71000, 214.65, -0.002, 3.95642),
    ]

    # Determine which layer h is in
    for i in range(len(layers) - 1):
        h_b, T_b, L_b, p_b = layers[i]
        h_next = layers[i + 1][0]
        if h < h_next:
            break
    else:
        # If higher than last defined layer (71 km), use the last one (valid to 86 km)
        h_b, T_b, L_b, p_b = layers[-1]

    # Calculate temperature at altitude
    if L_b == 0.0:
        T = T_b
    else:
        T = T_b + L_b * (h - h_b)

    # Calculate pressure at altitude
    if L_b == 0.0:
        p = p_b * math.exp(-g0 * (h - h_b) / (R * T_b))
    else:
        p = p_b * (T_b / T) ** (g0 / (R * L_b))

    # Density from ideal gas law
    rho = p / (R * T)
    return rho
