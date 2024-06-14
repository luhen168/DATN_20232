import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Giá trị khởi tạo ban đầu
initial_value = 1000

# Giá trị đo được mỗi lần
measurements = [996, 994, 1021, 1000, 1002, 1010, 983, 971, 993, 1023]

# Hệ số alpha
alpha = 0.1

# Danh sách để lưu trữ các giá trị ước lượng
estimates = []

# Giá trị ước lượng ban đầu
estimate = initial_value
estimates.append(estimate)

# Áp dụng công thức ước lượng
for measurement in measurements:
    estimate = estimate + alpha * (measurement - estimate)
    estimates.append(estimate)

# Tạo DataFrame để hiển thị kết quả
results_df = pd.DataFrame({
    'Measurement': [initial_value] + measurements,
    'Estimate': estimates
})

print(results_df)

# Vẽ đồ thị biểu diễn số lần đo và kết quả
plt.figure(figsize=(10, 6))
plt.plot(range(len(estimates)), estimates, marker='o', linestyle='-', color='b', label='Estimate')
plt.plot(range(len(measurements) + 1), [initial_value] + measurements, marker='x', linestyle='--', color='r', label='Measurement')
plt.title('Weight Estimation Over Time')
plt.xlabel('Measurement Number')
plt.ylabel('Weight (g)')
plt.legend()
plt.grid(True)
plt.show()
