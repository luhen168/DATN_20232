import numpy as np

# Function to update quaternion kinematics
def quaternion_kinematics(q, omega_BN, dt):
    q_dot = 0.5 * np.array([
        [-q[1], -q[2], -q[3]],
        [q[0], -q[3], q[2]],
        [q[3], q[0], -q[1]],
        [-q[2], q[1], q[0]]
    ]).dot(omega_BN)
    q = q + q_dot * dt
    return q / np.linalg.norm(q)

# Function to update latitude
def latitude_rate(v_N, R_lambda, h, dt):
    lat_dot = v_N / (R_lambda + h)
    return lat_dot * dt

# Function to update longitude
def longitude_rate(v_E, R_phi, h, lat, dt):
    lon_dot = v_E / ((R_phi + h) * np.cos(lat))
    return lon_dot * dt

# Function to update height
def height_rate(v_D, dt):
    h_dot = -v_D
    return h_dot * dt

# Function to update north velocity
def north_velocity_rate(v_N, v_E, v_D, R_phi, h, lat, omega_e, a_N, dt):
    v_N_dot = -((v_E / ((R_phi + h) * np.cos(lat)) + 2 * omega_e) * v_E * np.sin(lat)) + (v_N * v_D / (R_lambda + h)) + a_N
    return v_N_dot * dt

# Function to update east velocity
def east_velocity_rate(v_N, v_E, v_D, R_phi, h, lat, omega_e, a_E, dt):
    v_E_dot = ((v_E / ((R_phi + h) * np.cos(lat)) + 2 * omega_e) * v_N * np.sin(lat)) + (v_E * v_D / (R_phi + h)) + 2 * omega_e * v_D * np.cos(lat) + a_E
    return v_E_dot * dt

# Function to update down velocity
def down_velocity_rate(v_N, v_E, v_D, R_phi, R_lambda, h, lat, omega_e, g, a_D, dt):
    v_D_dot = - (v_E**2 / (R_phi + h)) - (v_N**2 / (R_lambda + h)) - 2 * omega_e * v_E * np.cos(lat) + g + a_D
    return v_D_dot * dt

# (12a) Function to correct angular rate
def corrected_angular_rate(omega_BI, bg, eta_gv, q, omega_NI):
    ANB = quaternion_to_rotation_matrix(q)  # Transform matrix from NED to body frame
    omega_BN = (omega_BI - bg - eta_gv) - np.dot(ANB, omega_NI)
    return omega_BN

# (12b) Function to correct specific force
def corrected_specific_force(a_B, ba, eta_av):
    return a_B - ba - eta_av

# (13a) Function to compute Earth's radius of curvature in the meridian
def radius_of_curvature_meridian(a, e, lat):
    return a * (1 - e**2) / (1 - e**2 * np.sin(lat)**2)**1.5

# (13b) Function to compute Earth's radius of curvature in the prime vertical
def radius_of_curvature_prime_vertical(a, e, lat):
    return a / (1 - e**2 * np.sin(lat)**2)**0.5

# (14) Function to compute gravity in the NED frame
def gravity_ned(lat, h):
    A = 9.780327
    B = 5.3024e-3
    C = 5.8e-6
    D = 3.0877e-6
    E = 4.4e-9
    F = 7.2e-14
    
    sin_lat = np.sin(lat)
    sin_2lat = np.sin(2 * lat)
    g = A * (1 + B * sin_lat**2 - C * sin_2lat**2) - (D - E * sin_lat**2) * h + F * h**2
    return g

# (15a) Function to update quaternion kinematics
def quaternion_kinematics(q, omega_BN, dt):
    q_dot = 0.5 * np.array([
        [-q[1], -q[2], -q[3]],
        [q[0], -q[3], q[2]],
        [q[3], q[0], -q[1]],
        [-q[2], q[1], q[0]]
    ]).dot(omega_BN)
    q = q + q_dot * dt
    return q / np.linalg.norm(q)

# (15b) Function to update estimated latitude
def latitude_rate_estimated(v_N_hat, R_lambda, h_hat, dt):
    lat_dot = v_N_hat / (R_lambda + h_hat)
    return lat_dot * dt

# (15c) Function to update estimated longitude
def longitude_rate_estimated(v_E_hat, R_phi, h_hat, lat, dt):
    lon_dot = v_E_hat / ((R_phi + h_hat) * np.cos(lat))
    return lon_dot * dt

# (15d) Function to update estimated height
def height_rate_estimated(v_D_hat, dt):
    h_dot = -v_D_hat
    return h_dot * dt

# (15e) Function to update estimated north velocity
def north_velocity_rate_estimated(v_N_hat, v_E_hat, v_D_hat, R_phi, h_hat, lat_hat, omega_e, a_N_hat, dt):
    v_N_dot = -((v_E_hat / ((R_phi + h_hat) * np.cos(lat_hat)) + 2 * omega_e) * v_E_hat * np.sin(lat_hat)) + (v_N_hat * v_D_hat / (R_lambda + h_hat)) + a_N_hat
    return v_N_dot * dt

# (15f) Function to update estimated east velocity
def east_velocity_rate_estimated(v_N_hat, v_E_hat, v_D_hat, R_phi, h_hat, lat_hat, omega_e, a_E_hat, dt):
    v_E_dot = ((v_E_hat / ((R_phi + h_hat) * np.cos(lat_hat)) + 2 * omega_e) * v_N_hat * np.sin(lat_hat)) + (v_E_hat * v_D_hat / (R_phi + h_hat)) + 2 * omega_e * v_D_hat * np.cos(lat_hat) + a_E_hat
    return v_E_dot * dt

# (15g) Function to update estimated down velocity
def down_velocity_rate_estimated(v_N_hat, v_E_hat, v_D_hat, R_phi, R_lambda, h_hat, lat_hat, omega_e, g_hat, a_D_hat, dt):
    v_D_dot = - (v_E_hat**2 / (R_phi + h_hat)) - (v_N_hat**2 / (R_lambda + h_hat)) - 2 * omega_e * v_E_hat * np.cos(lat_hat) + g_hat + a_D_hat
    return v_D_dot * dt

# (15h) Function to transform specific force from body frame to NED frame
def transform_specific_force(q, a_B):
    A_NB = quaternion_to_rotation_matrix(q)
    a_N = np.dot(A_NB, a_B)
    return a_N

# (15i) Function to correct specific force
def correct_specific_force(a_B_measured, b_a_hat):
    a_B_corrected = a_B_measured - b_a_hat
    return a_B_corrected

# (17a) Example values for the state, error, and process noise vectors
q = np.array([1, 0, 0, 0])  # Quaternion
p = np.array([45.0, 45.0, 100.0])  # Position (latitude, longitude, height)
v_N = np.array([5.0, 3.0, -2.0])  # Velocity in NED frame
b_g = np.array([0.01, 0.01, 0.01])  # Gyroscope biases
b_a = np.array([0.01, 0.01, 0.01])  # Accelerometer biases
b_c = np.array([0.0, 0.0, 0.0])  # Compass biases

# State vector
x = np.concatenate((q, p, v_N, b_g, b_a, b_c))

# Error vector
delta_x = np.zeros_like(x)  # Initialized to zero

# Process noise vector
w = np.array([0.001, 0.001, 0.001, 0.002, 0.002, 0.002, 0.003, 0.003, 0.003, 0.004, 0.004, 0.004])

# Measurement update matrix H_k (example, specific implementation depends on actual measurement function)
H_k = np.zeros((1, len(x)))
H_k[0, 3] = -1  # Example assignment for illustration
H_k[0, -1] = 1

# (17b) Process noise covariance matrix Q
sigma_gv = 0.01
sigma_gu = 0.02
sigma_av = 0.03
sigma_au = 0.04

"""
    Ma trận Q hiệp phương sai lỗi ! 
"""
Q = np.block([
    [sigma_gv**2 * np.eye(3), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3))],
    [np.zeros((3, 3)), sigma_gu**2 * np.eye(3), np.zeros((3, 3)), np.zeros((3, 3))],
    [np.zeros((3, 3)), np.zeros((3, 3)), sigma_av**2 * np.eye(3), np.zeros((3, 3))],
    [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), sigma_au**2 * np.eye(3)]
])

# (18a) Function to create the state transition matrix F
def state_transition_matrix(F11, F12, F13, F14, F22, F31, F32, F33, F35):
    F = np.block([
        [F11, F12, F13, F14, np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), F22, np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [F31, F32, F33, F35, np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 1))]
    ])
    return F

# (18b) Function to create the control matrix G
def control_matrix(ANB):
    G = np.block([
        [-np.eye(3), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.eye(3), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), -ANB, np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.eye(3), np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)), np.ones((1, 1))]
    ])
    return G

# Function to compute skew-symmetric matrix
def skew_symmetric(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

# Function to compute quaternion to rotation matrix
def quaternion_to_rotation_matrix(q):
    # Example implementation, replace with actual calculation
    return np.eye(3)

# Example values for derivatives (these should be replaced with actual derivatives from your system)
partial_omega_NI_partial_p = np.eye(3) * 0.1
partial_omega_NI_partial_vN = np.eye(3) * 0.01
partial_p_dot_partial_p = np.eye(3) * 0.1
partial_p_dot_partial_vN = np.eye(3) * 0.01
partial_vN_dot_partial_p = np.eye(3) * 0.1
partial_vN_dot_partial_vN = np.eye(3) * 0.01

# Estimated quaternion
q_hat = np.array([1, 0, 0, 0])
ANB = quaternion_to_rotation_matrix(q_hat)

# Compute F11
F11 = -skew_symmetric(omega_BI - b_g_hat)

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

print("Partial derivative of R_Phi with respect to latitude:", partial_R_Phi_lat)
print("Partial derivative of R_lambda with respect to latitude:", partial_R_lambda_lat)

# Example values for partial derivatives (these should be replaced with actual derivatives from your system)
v_N = 5.0
v_E = 3.0
h = 100.0

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

