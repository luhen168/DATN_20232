import numpy as np

# Giả sử ta có thời gian delta t
delta_t = 1  # Thời gian giữa các bước

# Ma trận chuyển trạng thái F
F = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
])

# Ma trận hiệp phương sai trạng thái ban đầu P
sigma_position = 1.0
sigma_velocity = 1.0
P = np.diag([sigma_position, sigma_position, sigma_position])

# Ma trận hiệp phương sai của nhiễu quá trình Q
process_noise_position = 0.1
process_noise_velocity = 0.1
Q = np.diag([process_noise_velocity, process_noise_velocity, process_noise_velocity])

# Dự đoán ma trận hiệp phương sai P ở bước thời gian n+1
P_next = F @ P @ F.T + Q

print("P_next:")
print(P_next)
