import numpy as np
import pandas as pd
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
import pymap3d as pm

def fx(x, dt):
    # State transition function
    F = np.array([[1, 0, 0, dt, 0, 0, 0.5*dt**2, 0, 0],
                  [0, 1, 0, 0, dt, 0, 0, 0.5*dt**2, 0],
                  [0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt**2],
                  [0, 0, 0, 1, 0, 0, dt, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, dt, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1]])
    return F @ x

def hx(x):
    # Measurement function
    return x[:9]  # Measurement includes position, velocity, and gyroscope

# Initial state vector
x = np.zeros(9)

# State covariance matrix
P = np.eye(9) * 0.1

# Process noise covariance matrix
Q = Q_discrete_white_noise(dim=3, dt=1, var=0.1, block_size=3, order_by_dim=False)

# Measurement noise covariance matrix
R = np.eye(9) * 0.1

# Unscented Kalman Filter setup
points = MerweScaledSigmaPoints(n=9, alpha=0.1, beta=2., kappa=1.)
ukf = UKF(dim_x=9, dim_z=9, fx=fx, hx=hx, dt=1, points=points)
ukf.x = x
ukf.P = P
ukf.R = R
ukf.Q = Q

# Đọc dữ liệu từ tệp CSV
gps_data = pd.read_csv(r'D:\DATN_20232\Code\GNSS-Positioning\position_velocity_cov_ecef.csv')
imu_data = pd.read_csv(r'D:\DATN_20232\Code\KalmanFilter\imu_convert_ecef.csv')

# Kết hợp dữ liệu dựa trên utcTimeMillis
merged_data = pd.merge(gps_data, imu_data, on='utcTimeMillis')

# Danh sách để lưu trữ các trạng thái
states = []
# Running the UKF
for i in range(len(merged_data)):

    if i == 5:
        break
    utc_time = merged_data.iloc[i]['utcTimeMillis']
    gps_measurement = merged_data.iloc[i][['x', 'y', 'z']].values
    imu_measurement = merged_data.iloc[i][['v_x', 'v_y', 'v_z', 'ax', 'ay', 'az']].values
    z = np.hstack((gps_measurement, imu_measurement))

    # Predict step
    ukf.predict()

    # Update step
    ukf.update(z)

    # Lưu trạng thái cập nhật vào danh sách
    state_with_time = np.hstack((utc_time, ukf.x.copy()))
    states.append(state_with_time)
    # Print the updated state
    print(f"Time step {i}, updated state: {ukf.x}")

# # Save the results to a CSV file
# columns = ['utcTimeMillis', 'position_x', 'position_y', 'position_z', 
#            'velocity_x', 'velocity_y', 'velocity_z',
#            'acceleration_x', 'acceleration_y', 'acceleration_z']
# results = pd.DataFrame(states, columns=columns)
# output_file = 'ukf_ecef.csv'
# results.to_csv(output_file, index=False)
# print(f"Results saved to {output_file}")

# Chuyển đổi tọa độ từ ECEF sang lat, long, alt
ecef_x = results['position_x'].values
ecef_y = results['position_y'].values
ecef_z = results['position_z'].values

llh = np.array(pm.ecef2geodetic(ecef_x, ecef_y, ecef_z)).T

results['latitude'] = llh[:, 0]
results['longitude'] = llh[:, 1]
results['altitude'] = llh[:, 2]

# # Chọn các cột cần lưu vào file CSV
# output_columns = ['utcTimeMillis', 'latitude', 'longitude', 'altitude']
# output_file = 'ukf_latlong.csv'
# results[output_columns].to_csv(output_file, index=False)
# print(f"Results saved to {output_file}")

# # In mẫu dữ liệu
# print(results[output_columns].head())
