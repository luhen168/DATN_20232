
# Function to compute skew-symmetric matrix
def skew_symmetric(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

# (21) Function to compute partial derivative of p_dot with respect to p
def partial_p_dot_partial_p(v_N, v_E, R_Phi, h, partial_R_Phi_lat, lat):
    sec_lat = 1 / np.cos(lat)
    tan_lat = np.tan(lat)
    return np.array([
        [v_N / (R_Phi + h)**2 * partial_R_Phi_lat + v_E * sec_lat * tan_lat / (R_Phi + h), -v_E * sec_lat / (R_Phi + h), 0],
        [0, 0, 0],
        [0, 0, 0]
    ])

# (22) Function to compute partial derivative of p_dot with respect to v_N
def partial_p_dot_partial_vN(R_Phi, h, lat):
    sec_lat = 1 / np.cos(lat)
    return np.array([
        [1 / (R_Phi + h), 0, 0],
        [0, 1 / ((R_Phi + h) * sec_lat), 0],
        [0, 0, -1]
    ])

# (23) Function to compute partial derivative of v_N_dot with respect to p
def partial_vN_dot_partial_p(Y11, Y12, Y13, Y21, Y23, Y31, Y33):
    return np.array([
        [Y11, Y12, Y13],
        [Y21, Y23, 0],
        [Y31, 0, Y33]
    ])

# Estimated quaternion
q_hat = np.array([1, 0, 0, 0])
ANB = quaternion_to_rotation_matrix(q_hat)

# Compute F11
F11 = -skew_symmetric(omega_BI - b_g_hat)  # Nhân ma trận 

# Compute F12
F12 = -np.dot(ANB, partial_omega_NI_partial_p)

# Compute F13
F13 = -np.dot(ANB, partial_omega_NI_partial_vN)

# Compute F14
F14 = -np.eye(3)

# Compute F22
F22 = partial_p_dot_partial_p

# Compute F23
F23 = partial_p_dot_partial_vN

# Compute F31
F31 = -np.dot(ANB, skew_symmetric(a_B_hat))

# Compute F32
F32 = partial_vN_dot_partial_p

# Compute F33
F33 = partial_vN_dot_partial_vN

# Compute F35
F35 = -ANB


"""
    
"""
# (20a) Function to compute partial derivative of R_Phi with respect to latitude
def partial_R_Phi_partial_lat(a, e, lat):
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    return (a * e**2 * sin_lat * cos_lat) / (1 - e**2 * sin_lat**2)**(3/2)

# Function to compute partial derivative of R_lambda with respect to latitude
def partial_R_lambda_partial_lat(a, e, lat):
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    return (3 * a * (1 - e**2) * e**2 * sin_lat * cos_lat) / (1 - e**2 * sin_lat**2)**(5/2)

# Partial derivative values
partial_R_Phi_lat = partial_R_Phi_partial_lat(a, e, lat)
partial_R_lambda_lat = partial_R_lambda_partial_lat(a, e, lat)

print("Partial derivative of R_Phi with respect to latitude:", partial_R_Phi_lat)
print("Partial derivative of R_lambda with respect to latitude:", partial_R_lambda_lat)

# Example values for partial derivatives (these should be replaced with actual derivatives from your system)
v_N = 5.0
v_E = 3.0
h = 100.0








"""
    20a - 23
"""
# Function to compute partial derivative of R_Phi with respect to latitude
def partial_R_Phi_partial_lat(a, e, lat):
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    return (a * e**2 * sin_lat * cos_lat) / (1 - e**2 * sin_lat**2)**(3/2)

# Function to compute partial derivative of R_lambda with respect to latitude
def partial_R_lambda_partial_lat(a, e, lat):
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    return (3 * a * (1 - e**2) * e**2 * sin_lat * cos_lat) / (1 - e**2 * sin_lat**2)**(5/2)


# Function to compute partial derivative of p_dot with respect to p
def partial_p_dot_partial_p(v_N, v_E, R_Phi, h, partial_R_Phi_lat, lat):
    sec_lat = 1 / np.cos(lat)
    tan_lat = np.tan(lat)
    return np.array([
        [v_N / (R_Phi + h)**2 * partial_R_Phi_lat + v_E * sec_lat * tan_lat / (R_Phi + h), -v_E * sec_lat / (R_Phi + h), 0],
        [0, 0, 0],
        [0, 0, 0]
    ])

# Function to compute partial derivative of p_dot with respect to v_N
def partial_p_dot_partial_vN(R_Phi, h, lat):
    sec_lat = 1 / np.cos(lat)
    return np.array([
        [1 / (R_Phi + h), 0, 0],
        [0, 1 / ((R_Phi + h) * sec_lat), 0],
        [0, 0, -1]
    ])

# Function to compute partial derivative of v_N_dot with respect to p
def partial_vN_dot_partial_p(Y11, Y12, Y13, Y21, Y23, Y31, Y33):
    return np.array([
        [Y11, Y12, Y13],
        [Y21, Y23, 0],
        [Y31, 0, Y33]
    ])

"""

"""

# Function to compute partial derivative of R_Phi with respect to latitude
def partial_R_Phi_partial_lat(a, e, lat):
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    return (a * e**2 * sin_lat * cos_lat) / (1 - e**2 * sin_lat**2)**(3/2)

# Function to compute partial derivative of R_lambda with respect to latitude
def partial_R_lambda_partial_lat(a, e, lat):
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    return (3 * a * (1 - e**2) * e**2 * sin_lat * cos_lat) / (1 - e**2 * sin_lat**2)**(5/2)

# Partial derivative values
partial_R_Phi_lat = partial_R_Phi_partial_lat(a, e, lat)
partial_R_lambda_lat = partial_R_lambda_partial_lat(a, e, lat)

# Example values for velocities (these should be replaced with actual values from your system)
v_N = 5.0
v_E = 3.0
v_D = -2.0
h = 100.0
omega_e = 7.292115e-5  # Earth's rotation rate (rad/s)

# Function to compute Y11
def compute_Y11(v_E, R_Phi, h, partial_R_Phi_lat, v_N, v_D, R_lambda, partial_R_lambda_lat, omega_e, lat):
    sec_lat = 1 / np.cos(lat)
    tan_lat = np.tan(lat)
    term1 = -v_E**2 * sec_lat**2 / (R_Phi + h)
    term2 = v_E**2 * tan_lat / (R_Phi + h)**2 * partial_R_Phi_lat
    term3 = v_N * v_D / (R_lambda + h)**2 * partial_R_lambda_lat
    term4 = -2 * omega_e * v_E * np.cos(lat)
    term5 = -v_N * v_D / (R_lambda + h)**2 * partial_R_lambda_lat
    return term1 + term2 + term3 + term4 + term5

# Function to compute Y13
def compute_Y13(v_E, R_Phi, h, v_N, v_D, R_lambda):
    tan_lat = np.tan(lat)
    term1 = v_E**2 * tan_lat / (R_Phi + h)**2
    term2 = -v_N * v_D / (R_lambda + h)**2
    return term1 + term2

# Function to compute Y21
def compute_Y21(v_E, v_N, R_Phi, h, partial_R_Phi_lat, omega_e, lat, v_D):
    sec_lat = 1 / np.cos(lat)
    tan_lat = np.tan(lat)
    term1 = v_E * v_N * sec_lat**2 / (R_Phi + h)
    term2 = v_E * v_N * tan_lat / (R_Phi + h)**2 * partial_R_Phi_lat
    term3 = 2 * omega_e * v_N * np.cos(lat)
    term4 = -v_E * v_N * tan_lat / (R_Phi + h)**2 * partial_R_Phi_lat
    term5 = -2 * omega_e * v_D * np.sin(lat)
    return term1 + term2 + term3 + term4 + term5

# Function to compute Y23
def compute_Y23(v_E, v_N, lat, v_D, R_Phi, h):
    tan_lat = np.tan(lat)
    term = v_E * (v_N * tan_lat + v_D) / (R_Phi + h)**2
    return -term

# Function to compute Y31
def compute_Y31(v_E, R_Phi, h, partial_R_Phi_lat, v_N, R_lambda, partial_R_lambda_lat, omega_e, lat, partial_g_lat):
    term1 = v_E**2 / (R_Phi + h)**2 * partial_R_Phi_lat
    term2 = v_N**2 / (R_lambda + h)**2 * partial_R_lambda_lat
    term3 = 2 * omega_e * v_E * np.sin(lat)
    term4 = partial_g_lat
    return term1 + term2 + term3 + term4

# Function to compute Y33
def compute_Y33(v_E, R_Phi, h, v_N, R_lambda, partial_g_h):
    term1 = v_E**2 / (R_Phi + h)**2
    term2 = v_N**2 / (R_lambda + h)**2
    term3 = partial_g_h
    return term1 + term2 + term3

# Compute partial derivative of gravity with respect to height
def partial_g_partial_h(A, B, C, D, E, F, lat, h):
    sin_lat = np.sin(lat)
    sin_2lat = np.sin(2 * lat)
    term1 = -(D - E * sin_lat**2)
    term2 = 2 * F * h
    return term1 + term2

"""

"""
# Function to compute partial derivative of gravity with respect to height
def partial_g_partial_h(D, E, F, lat):
    sin_lat = np.sin(lat)
    sin_2lat = np.sin(2 * lat)
    term1 = -D + E * sin_lat**2
    term2 = 2 * F
    return term1 + term2

# Partial derivative value
partial_g_h = partial_g_partial_h(D, E, F, lat)

# Example values for velocities and Earth's parameters
v_N = 5.0
v_E = 3.0
v_D = -2.0
h = 100.0
R_lambda = 6378137.0  # Example value, replace with actual calculation
R_Phi = 6378137.0  # Example value, replace with actual calculation
omega_e = 7.292115e-5  # Earth's rotation rate (rad/s)

# Function to compute Z11
def compute_Z11(v_D, R_lambda, h):
    return v_D / (R_lambda + h)

# Function to compute Z12
def compute_Z12(v_E, R_Phi, h, omega_e, lat):
    tan_lat = np.tan(lat)
    return -2 * v_E * tan_lat / (R_Phi + h) + 2 * omega_e * np.sin(lat)

# Function to compute Z13
def compute_Z13(v_N, R_lambda, h):
    return v_N / (R_lambda + h)

# Function to compute Z21
def compute_Z21(v_E, R_Phi, h, omega_e, lat):
    tan_lat = np.tan(lat)
    return v_E * tan_lat / (R_Phi + h) + 2 * omega_e * np.sin(lat)

# Function to compute Z22
def compute_Z22(v_D, v_N, R_Phi, h, lat):
    tan_lat = np.tan(lat)
    return (v_D + v_N * tan_lat) / (R_Phi + h)

# Function to compute Z23
def compute_Z23(v_E, R_Phi, h, omega_e, lat):
    return v_E / (R_Phi + h) + 2 * omega_e * np.cos(lat)

# Function to compute Z31
def compute_Z31(v_N, R_lambda, h):
    return -2 * v_N / (R_lambda + h)

# Function to compute Z32
def compute_Z32(v_E, R_Phi, h, omega_e, lat):
    return -2 * v_E / (R_Phi + h) + 2 * omega_e * np.cos(lat)

# Compute Z matrix components
Z11 = compute_Z11(v_D, R_lambda, h)
Z12 = compute_Z12(v_E, R_Phi, h, omega_e, lat)
Z13 = compute_Z13(v_N, R_lambda, h)
Z21 = compute_Z21(v_E, R_Phi, h, omega_e, lat)
Z22 = compute_Z22(v_D, v_N, R_Phi, h, lat)
Z23 = compute_Z23(v_E, R_Phi, h, omega_e, lat)
Z31 = compute_Z31(v_N, R_lambda, h)
Z32 = compute_Z32(v_E, R_Phi, h, omega_e, lat)

# Jacobian matrix
Jacobian = np.array([
    [Z11, Z12, Z13],
    [Z21, Z22, Z23],
    [Z31, Z32, 0]
])