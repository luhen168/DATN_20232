import numpy as np
import pandas as pd
import pymap3d as pm
from numpy.linalg import inv  # Thêm hàm inv từ numpy.linalg
from scipy.spatial.distance import mahalanobis

# Hàm chuyển tiếp trạng thái sử dụng dữ liệu IMU
def state_transition_function(state, imu_accel, imu_gyro, dt):
    position = state[:3]
    velocity = state[3:6]

    # Gia tốc và vận tốc góc từ IMU
    accel = imu_accel
    gyro = imu_gyro

    # Cập nhật vị trí và vận tốc
    new_position = position + velocity * dt + 0.5 * accel * dt**2
    new_velocity = velocity + accel * dt

    new_state = np.hstack((new_position, new_velocity))
    return new_state

# Hàm đo lường từ GPS
def measurement_function(state):
    # Giả sử đo lường bao gồm vị trí và vận tốc từ GPS
    return state

# Hàm tính toán điểm sigma
def compute_sigma_points(state, P, alpha=1e-3, beta=2, kappa=0):
    n = state.shape[0]
    lambda_ = alpha**2 * (n + kappa) - n
    sigma_points = np.zeros((2 * n + 1, n))
    sigma_points[0] = state
    sqrt_matrix = np.linalg.cholesky((lambda_ + n) * P)
    for i in range(n):
        sigma_points[i + 1] = state + sqrt_matrix[:, i]
        sigma_points[n + i + 1] = state - sqrt_matrix[:, i]
    
    Wm = np.full(2 * n + 1, 0.5 / (n + lambda_))
    Wc = np.full(2 * n + 1, 0.5 / (n + lambda_))
    Wm[0] = lambda_ / (n + lambda_)
    Wc[0] = lambda_ / (n + lambda_) + (1 - alpha**2 + beta)
    
    return sigma_points, Wm, Wc

# UKF Prediction Step
def ukf_predict(state, P, imu_accel, imu_gyro, Q, dt):
    # Tính toán các điểm sigma
    sigma_points, Wm, Wc = compute_sigma_points(state, P)
    predicted_sigma_points = np.array([state_transition_function(sigma, imu_accel, imu_gyro, dt) for sigma in sigma_points])
    
    # Dự đoán trạng thái trung bình và hiệp phương sai
    state_predicted = np.dot(Wm, predicted_sigma_points)
    P_predicted = Q.copy()
    for i in range(len(Wc)):
        y = predicted_sigma_points[i] - state_predicted
        P_predicted += Wc[i] * np.outer(y, y)
    
    return state_predicted, P_predicted

# UKF Update Step
def ukf_update(state_predicted, P_predicted, measurement, R):
    # Tính toán các điểm sigma sau dự đoán
    sigma_points, Wm, Wc = compute_sigma_points(state_predicted, P_predicted)
    predicted_measurements = np.array([measurement_function(sigma) for sigma in sigma_points])
    
    # Dự đoán giá trị đo lường và hiệp phương sai
    measurement_predicted = np.dot(Wm, predicted_measurements)
    Pyy = R.copy()
    for i in range(len(Wc)):
        y = predicted_measurements[i] - measurement_predicted
        Pyy += Wc[i] * np.outer(y, y)
    
    Pxy = np.zeros((state_predicted.shape[0], measurement.shape[0]))
    for i in range(len(Wc)):
        x_diff = sigma_points[i] - state_predicted
        y_diff = predicted_measurements[i] - measurement_predicted
        Pxy += Wc[i] * np.outer(x_diff, y_diff)
    
    # Cập nhật trạng thái và hiệp phương sai
    K = np.dot(Pxy, np.linalg.inv(Pyy))
    state_updated = state_predicted + np.dot(K, (measurement - measurement_predicted))
    P_updated = P_predicted - np.dot(K, np.dot(Pyy, K.T))
    
    return state_updated, P_updated

# Đọc dữ liệu từ file CSV
imu_data = pd.read_csv(r'C:\Users\luan1\OneDrive\Desktop\DATN_20232\Code\KalmanFilter\imu_ecef.csv')
gps_data = pd.read_csv(r'C:\Users\luan1\OneDrive\Desktop\DATN_20232\Code\GNSS-Positioning\position_velocity_cov_ecef.csv')

# Kiểm tra kích thước của dữ liệu
imu_data_len = len(imu_data)
gps_data_len = len(gps_data)

print(f"IMU Data Length: {imu_data_len}")
print(f"GPS Data Length: {gps_data_len}")

# Đảm bảo rằng chỉ lặp qua số lượng hàng ít hơn
min_length = min(imu_data_len, gps_data_len)

# Khởi tạo trạng thái ban đầu và hiệp phương sai
initial_state = np.zeros(6)  # [x, y, z, vx, vy, vz]
P = np.eye(6)  # Hiệp phương sai ban đầu

# Ma trận hiệp phương sai của nhiễu quá trình và nhiễu đo lường
Q = np.eye(6) * 0.01
R = np.eye(6) * 0.1

# Khởi tạo thời gian
dt = 1.0  # Giả sử khoảng thời gian giữa các lần đo lường là 1 giây

# Khởi tạo danh sách để lưu trữ kết quả
results = []

# Xác định ngưỡng cho khoảng cách Mahalanobis
threshold = 30.0  # Ngưỡng có thể thay đổi tùy thuộc vào yêu cầu cụ thể

# Vòng lặp qua các dữ liệu
for idx in range(min_length):
    # Lấy dữ liệu IMU và GPS tại thời điểm hiện tại
    imu_accel = imu_data.iloc[idx][['accel_ecef_x', 'accel_ecef_y', 'accel_ecef_z']].values
    imu_gyro = imu_data.iloc[idx][['gyro_ecef_x', 'gyro_ecef_y', 'gyro_ecef_z']].values
    gps_position = gps_data.iloc[idx][['x', 'y', 'z']].values
    gps_velocity = gps_data.iloc[idx][['v_x', 'v_y', 'v_z']].values
    measurement = np.hstack((gps_position, gps_velocity))
    
    # Tính khoảng cách Mahalanobis cho đo lường hiện tại
    mean_measurement = np.mean(gps_data[['x', 'y', 'z', 'v_x', 'v_y', 'v_z']], axis=0)
    cov_measurement = np.cov(gps_data[['x', 'y', 'z', 'v_x', 'v_y', 'v_z']].values, rowvar=False)
    inv_cov_measurement = inv(cov_measurement)
    mahal_distance = mahalanobis(measurement, mean_measurement, inv_cov_measurement)
    
    # Loại bỏ các điểm bất thường
    if mahal_distance > threshold:
        continue
    
    # Bước dự đoán
    state_predicted, P_predicted = ukf_predict(initial_state, P, imu_accel, imu_gyro, Q, dt)
    
    # Bước cập nhật
    state_updated, P_updated = ukf_update(state_predicted, P_predicted, measurement, R)
    
    # Cập nhật trạng thái ban đầu cho lần lặp tiếp theo
    initial_state = state_updated
    P = P_updated

    # Thêm giá trị thời gian vào kết quả
    utcTimeMillis = imu_data.iloc[idx]['utcTimeMillis']
    result_with_time = np.hstack(([utcTimeMillis], state_updated))

    # Lưu kết quả vào danh sách
    results.append(result_with_time)

# Chuyển danh sách kết quả thành DataFrame
results_df = pd.DataFrame(results, columns=['utcTimeMillis', 'x', 'y', 'z', 'vx', 'vy', 'vz'])

# Xuất kết quả ra file CSV
results_df.to_csv('UKF_pos_vec_ecef_filtered.csv', index=False)

# Giả sử bạn đã có DataFrame results_df từ đoạn mã trước đó
# Đọc dữ liệu từ results_df để lấy các giá trị x, y, z
utcTimeMillis = results_df['utcTimeMillis'].values
x = results_df['x'].values
y = results_df['y'].values
z = results_df['z'].values


# Chuyển đổi từ ECEF sang tọa độ địa lý (Latitude, Longitude, Altitude)
llh_ukf = np.array(pm.ecef2geodetic(x, y, z)).T

# Tạo mảng vị trí với thời gian và tọa độ địa lý
position_array = [(time, lat, lon, alt) for time, (lat, lon, alt) in zip(utcTimeMillis, llh_ukf)]

# Chuyển đổi mảng vị trí thành DataFrame và xuất ra file CSV
df = pd.DataFrame(position_array, columns=["utcTimeMillis", "Latitude", "Longitude", "Altitude"])
df.to_csv('ukf_gnss_latlong.csv', index=False)
print('Data successfully exported to ukf_gnss_latlong.csv')

# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.spatial import distance

# class KalmanFilter:
#     def __init__(self, delta_t):
#         self.sigma_mahalanobis = 30.0 
#         self.delta_t = delta_t
#         self.F = self.state_transition_matrix(delta_t)
#         self.G = self.control_matrix(delta_t)
#         self.H = np.eye(6)  # Measurement matrix for position and velocity
#         self.I = np.eye(6)  # Identity matrix

#     def state_transition_matrix(self, delta_t):
#         F = np.array([[1, 0, 0, delta_t, 0, 0],
#                       [0, 1, 0, 0, delta_t, 0],
#                       [0, 0, 1, 0, 0, delta_t],
#                       [0, 0, 0, 1, 0, 0],
#                       [0, 0, 0, 0, 1, 0],
#                       [0, 0, 0, 0, 0, 1]])
#         return F

#     def control_matrix(self, delta_t):
#         G = np.array([[0.5 * delta_t**2, 0, 0],
#                       [0, 0.5 * delta_t**2, 0],
#                       [0, 0, 0.5 * delta_t**2],
#                       [delta_t, 0, 0],
#                       [0, delta_t, 0],
#                       [0, 0, delta_t]])
#         return G

#     def predict(self, x, u, P, Q):
#         x_pred = self.F @ x + self.G @ u  # State Extrapolation Equation
#         P_pred = self.F @ P @ self.F.T + Q  # Covariance Extrapolation Equation 
#         return x_pred, P_pred

#     def estimate(self, x_pred, P_pred, z, R):
#         y = z - self.H @ x_pred  # Measurement residal
#         S = self.H @ P_pred @ self.H.T + R  # Innovation (residual) covariance
#         K = P_pred @ self.H.T @ np.linalg.inv(S)  # Kalman Gain
#         x = x_pred + K @ y  # State Update Equation (Estimate)
#         P = (self.I - K @ self.H) @ P_pred  # Covariance Update Equation 
#         return x, P

#     def kalman_filter(self, imu_data, gps_data, Q, R, threshold=30.0):
#         min_length = min(len(imu_data), len(gps_data))
#         results = []
#         initial_state = np.zeros((6, 1))  # Initial state [x, y, z, vx, vy, vz]
#         P = 5**2 * np.eye(6)  # Initial state covariance

#         for idx in range(min_length):
#             # Lấy dữ liệu IMU và GPS tại thời điểm hiện tại
#             imu_accel = imu_data.iloc[idx][['accel_ecef_x', 'accel_ecef_y', 'accel_ecef_z']].values.reshape((3, 1))
#             imu_gyro = imu_data.iloc[idx][['gyro_ecef_x', 'gyro_ecef_y', 'gyro_ecef_z']].values.reshape((3, 1))
#             gps_position = gps_data.iloc[idx][['x', 'y', 'z']].values
#             gps_velocity = gps_data.iloc[idx][['v_x', 'v_y', 'v_z']].values
#             measurement = np.hstack((gps_position, gps_velocity)).reshape((6, 1))
            
#             # Tính khoảng cách Mahalanobis cho đo lường hiện tại
#             mean_measurement = np.mean(gps_data[['x', 'y', 'z', 'v_x', 'v_y', 'v_z']], axis=0)
#             cov_measurement = np.cov(gps_data[['x', 'y', 'z', 'v_x', 'v_y', 'v_z']].values, rowvar=False)
#             inv_cov_measurement = inv(cov_measurement)
#             mahal_distance = mahalanobis(measurement.flatten(), mean_measurement, inv_cov_measurement)
            
#             # Loại bỏ các điểm bất thường
#             if mahal_distance > threshold:
#                 continue
            
#             # Bước dự đoán
#             state_predicted, P_predicted = self.predict(initial_state, imu_accel, P, Q)
            
#             # Bước cập nhật
#             state_updated, P_updated = self.estimate(state_predicted, P_predicted, measurement, R)
            
#             # Cập nhật trạng thái ban đầu cho lần lặp tiếp theo
#             initial_state = state_updated
#             P = P_updated

#             # Thêm giá trị thời gian vào kết quả
#             utcTimeMillis = imu_data.iloc[idx]['utcTimeMillis']
#             result_with_time = np.hstack(([utcTimeMillis], state_updated.flatten()))

#             # Lưu kết quả vào danh sách
#             results.append(result_with_time)

#         return results
