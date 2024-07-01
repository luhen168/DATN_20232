import pandas as pd
import matplotlib.pyplot as plt

# Đọc dữ liệu từ tệp CSV
gps_data = pd.read_csv('pos_vel_ned.csv')

# Kiểm tra tên các cột
print("GPS Data Columns:", gps_data.columns)

# Vẽ đồ thị biểu diễn velocity x
plt.figure(figsize=(12, 6))

# Giả sử 'velocity_x' là tên cột chứa dữ liệu velocity x trong tệp CSV
plt.plot(gps_data['vx_ned'], label='x')

plt.xlabel('Index')
plt.ylabel('Velocity (m/s)')
plt.title('Calculated Velocity from GPS Data')
plt.legend()
plt.grid(True)

# Hiển thị đồ thị
plt.show()