import pandas as pd

# Đọc dữ liệu từ hai file CSV
# imu_data = pd.read_csv(r'C:\Users\luan1\OneDrive\Desktop\DATN_20232\Code\GNSS-Positioning\data\2020-08-13-21-42-us-ca-mtv-sf-280\pixel5\device_imu.csv')
# gnss_data = pd.read_csv(r'C:\Users\luan1\OneDrive\Desktop\DATN_20232\Code\GNSS-Positioning\data\2020-08-13-21-42-us-ca-mtv-sf-280\pixel5\device_gnss.csv')
imu_data = pd.read_csv(r'D:\DATN_20232\Code\GNSS-Positioning\data\2020-06-25-00-34-us-ca-mtv-sb-101\pixel4xl\device_imu.csv')
gnss_data = pd.read_csv(r'D:\DATN_20232\Code\GNSS-Positioning\data\2020-06-25-00-34-us-ca-mtv-sb-101\pixel4xl\device_gnss.csv')

# Khởi tạo danh sách để lưu kết quả trung bình
result = []

# Vòng lặp qua từng giá trị thời gian trong file GNSS
for epoch in gnss_data['utcTimeMillis'].unique():
    # Lọc dữ liệu IMU theo khoảng thời gian 1 giây tương ứng với giá trị thời gian GNSS
    last_three_digits = epoch % 1000
    epoch_start = epoch - last_three_digits
    epoch_end = epoch_start + 1000
    epoch_data = imu_data[(imu_data['utcTimeMillis'] >= epoch_start) & (imu_data['utcTimeMillis'] < epoch_end)]
    
    if not epoch_data.empty:
        # Tính giá trị trung bình cho các dữ liệu Accel, Gyro, Mag
        mean_accel = epoch_data[epoch_data['MessageType'] == 'UncalAccel'][['MeasurementX', 'MeasurementY', 'MeasurementZ']].mean()
        mean_gyro = epoch_data[epoch_data['MessageType'] == 'UncalGyro'][['MeasurementX', 'MeasurementY', 'MeasurementZ']].mean()
        mean_mag = epoch_data[epoch_data['MessageType'] == 'UncalMag'][['MeasurementX', 'MeasurementY', 'MeasurementZ']].mean()
        mean_bias_mag = epoch_data[epoch_data['MessageType'] == 'UncalMag'][['BiasX', 'BiasY', 'BiasZ']].mean()
        
        # Tạo dictionary lưu kết quả cho epoch hiện tại
        result.append({
            'utcTimeMillis': epoch,
            'mean_accel_x': mean_accel['MeasurementX'],
            'mean_accel_y': mean_accel['MeasurementY'],
            'mean_accel_z': mean_accel['MeasurementZ'],
            'mean_gyro_x': mean_gyro['MeasurementX'],
            'mean_gyro_y': mean_gyro['MeasurementY'],
            'mean_gyro_z': mean_gyro['MeasurementZ'],
            'mean_mag_x': mean_mag['MeasurementX'],
            'mean_mag_y': mean_mag['MeasurementY'],
            'mean_mag_z': mean_mag['MeasurementZ'],
            'mean_bias_mag_x': mean_bias_mag['BiasX'],
            'mean_bias_mag_y': mean_bias_mag['BiasY'],
            'mean_bias_mag_z': mean_bias_mag['BiasZ']
        })

# Chuyển danh sách kết quả thành DataFrame
result_df = pd.DataFrame(result)

# Xuất kết quả ra file CSV
result_df.to_csv('imu_gnss_averaged_data.csv', index=False)

# Hiển thị một số kết quả đầu tiên
print(result_df.head())
