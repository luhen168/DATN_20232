import numpy as np

def to_cross_matrix(vector):
    return np.array([
        [0, -vector[2], vector[1]],
        [vector[2], 0, -vector[0]],
        [-vector[1], vector[0], 0]
    ])

def sgn(x):
    return (x > 0) - (x < 0)

def deg_to_rad(deg):
    return np.pi / 180.0 * deg

def rad_to_deg(rad):
    return 180.0 / np.pi * rad

def ms2_to_g(ms2):
    return ms2 / 9.81

def g_to_ms2(g):
    return g * 9.81
