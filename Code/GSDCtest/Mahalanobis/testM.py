import numpy as np
import pandas as pd
import pymap3d as pm

def compute_mahalanobis_distance(x, mean, cov):
    diff = x - mean
    return np.sqrt(np.dot(np.dot(diff.T, np.linalg.inv(cov)), diff))

def remove_outliers(data, threshold=0.001, max_iter=100):
    for _ in range(max_iter):
        mean = data.mean(axis=0)
        cov = np.cov(data, rowvar=False)
        distances = data.apply(lambda row: compute_mahalanobis_distance(row.values, mean, cov), axis=1)
        threshold_value = np.percentile(distances, 99)
        new_data = data[distances <= threshold_value]

        if len(new_data) == len(data):
            break

        data = new_data

        mean_new = data.mean(axis=0)
        if np.sqrt(np.sum((mean_new - mean)**2)) < threshold:
            break

    return data

# Đọc dữ liệu từ tệp CSV
data = pd.read_csv(r'C:\Users\luan1\OneDrive\Desktop\OneDrive\Desktop\DATN_20232\Code\GSDCtest\IMUtest\wls_ecef.csv')[['x', 'y', 'z']]

# Loại bỏ các giá trị ngoại lai
cleaned_data = remove_outliers(data)

# Lưu dữ liệu đã làm sạch
cleaned_data.to_csv('cleaned_gps_data.csv', index=False)

# Chuyển đổi tọa độ từ ECEF sang Geodetic
def ecef_to_geodetic(x, y, z):
    lat, lon, alt = pm.ecef2geodetic(x, y, z)
    return lon, lat, alt

# Đọc dữ liệu từ tệp cleaned_gps_data.csv
data = pd.read_csv('cleaned_gps_data.csv')



# Áp dụng chuyển đổi cho từng hàng dữ liệu
data[['longitude', 'latitude', 'altitude']] = data.apply(
    lambda row: ecef_to_geodetic(row['x'], row['y'], row['z']),
    axis=1, result_type='expand'
)
# Chỉ giữ lại các cột latitude, longitude, altitude
data = data[['latitude', 'longitude', 'altitude']]

# Lưu dữ liệu đã chuyển đổi sang tệp CSV mới
data.to_csv('converted_geodetic_data.csv', index=False)