import numpy as np
import pandas as pd
import pymap3d as pm

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

# Hàm tính toán điểm sigma
def compute_sigma_points(state, P):
    n = len(state)
    lambda_ = 3 - n
    sigma_points = [state]
    sqrt_P = np.linalg.cholesky((n + lambda_) * P)
    for i in range(n):
        sigma_points.append(state + sqrt_P[:, i])
        sigma_points.append(state - sqrt_P[:, i])
    sigma_points = np.array(sigma_points)
    
    Wm = np.full(2 * n + 1, 0.5 / (n + lambda_))
    Wc = np.full(2 * n + 1, 0.5 / (n + lambda_))
    Wm[0] = lambda_ / (n + lambda_)
    Wc[0] = lambda_ / (n + lambda_) + (1 - 1**2 + 2)
    
    return sigma_points, Wm, Wc

# Đọc dữ liệu từ file CSV
imu_data = pd.read_csv(r'D:\DATN_20232\Code\KalmanFilter\imu_ecef.csv')
gps_data = pd.read_csv(r'D:\DATN_20232\Code\GNSS-Positioning\position_velocity_cov_ecef.csv')

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

# Create a list to store the results
results = []

# Vòng lặp qua các dữ liệu
for idx in range(min_length):
    # Lấy dữ liệu IMU và GPS tại thời điểm hiện tại
    imu_accel = imu_data.iloc[idx][['accel_ecef_x', 'accel_ecef_y', 'accel_ecef_z']].values
    imu_gyro = imu_data.iloc[idx][['gyro_ecef_x', 'gyro_ecef_y', 'gyro_ecef_z']].values
    gps_position = gps_data.iloc[idx][['x', 'y', 'z']].values
    gps_velocity = gps_data.iloc[idx][['v_x', 'v_y', 'v_z']].values
    measurement = np.hstack((gps_position, gps_velocity))
    
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

    # # Lưu hoặc hiển thị kết quả nếu cần
    # print(f"Time {idx}: Updated State: {state_updated}")

# Chuyển danh sách kết quả thành DataFrame
results_df = pd.DataFrame(results, columns=['utcTimeMillis', 'x', 'y', 'z', 'vx', 'vy', 'vz'])

# Xuất kết quả ra file CSV
results_df.to_csv('UKF_pos_vec_ecef.csv', index=False)

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
