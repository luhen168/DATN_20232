import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat

# Định nghĩa các tham số
a = 6378137.0        # Bán kính ellipsoid của Trái Đất tại xích đạo (m)
b = 6356752.3142     # Bán kính ellipsoid của Trái Đất tại cực (m)
ecc = 0.0818191908426  # Độ lệch tâm của ellipsoid Trái Đất
w_i_e = 7.2921150 * 10**-5  # Tốc độ quay của Trái Đất (rad/s)
mu = 3.986004418 * 10**14  # Hằng số hấp dẫn địa tâm (m^3/s^2)
f = 1/298.257223563  # Độ phẳng của ellipsoid Trái Đất
omega_ib_b = np.zeros((3, 3))
g0 = 0
R2D = 180 / np.pi
D2R = np.pi / 180

# Load dữ liệu
filename3 = 'IMAR0003.mat'
data = loadmat(filename3)

# Kiểm tra các biến đã tải vào
print("Loaded variables from IMAR0003.mat:", data.keys())

# Kiểm tra kích thước và cấu trúc của IMU và GPS
imu = data['imu']
gps = data['gps']

# Kiểm tra nội dung bên trong các mảng IMU và GPS
print("IMU content:", imu)
print("GPS content:", gps)

filename03 = 'UTM_IMAR0003.mat'
utm_data = loadmat(filename03)
UTM = utm_data['UTM']

print("UTM shape:", UTM.shape)

# Nếu các mảng IMU và GPS chỉ chứa một phần tử là một mảng khác, cần phải truy cập phần tử đó
imu_data = imu[0, 0]
gps_data = gps[0, 0]

# Kiểm tra nội dung bên trong phần tử
print("IMU data fields:", imu_data)
print("GPS data fields:", gps_data)

# Truy cập vào các trường bên trong IMU
acc_ib_b = imu_data[0]
omg_ib_b = imu_data[1]
rpy_ned = imu_data[2]
time_imu = imu_data[4]

# Khởi tạo các biến
x_0 = UTM[0, 0]
y_0 = UTM[0, 1]

# Kiểm tra kích thước của 'rpy_ned' để truy cập đúng cách
if rpy_ned.shape[1] >= 3:
    phi_0 = rpy_ned[0, 2]
else:
    print("Mảng 'rpy_ned' không có đủ phần tử để truy cập vào chỉ số 2.")
    phi_0 = 0  # hoặc một giá trị mặc định khác

phi_D_0 = 0
v_0 = 0
a_0 = 0

x_i = 0
y_i = 0
phi_i = 0
phi_D_i = 0
v_i = 0
a_i = 0

Delta_t = 0.001

Variance_phi_D = 0.0035
Varince_a = 0.01**2

Variance_x = 10**-4
Variance_y = 10**-4
Variance_phi = 10**-5

P_kM1 = np.array([
    [Variance_x, 0, 0, 0, 0, 0],
    [0, Variance_y, 0, 0, 0, 0],
    [0, 0, Variance_phi, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]
])

A = UTM.shape
Res = A[0]
B = imu.shape
cnt = 1
cnt_b = 1
Interval = 1
P_Interval = 10

plt.figure()

for cnt in range(0, Res, 15):
    plt.plot(UTM[cnt, 0], UTM[cnt, 1], 'r.')

cnt = Interval + 1
while cnt < Res:  # Đảm bảo cnt không vượt quá Res
    # Prediction step
    x_kM = np.array([
        x_0 + v_0 * Delta_t * np.cos(phi_0),
        y_0 + v_0 * Delta_t * np.sin(phi_0),
        phi_0 + phi_D_0 * Delta_t,
        phi_D_0,
        v_0 + a_0 * Delta_t,
        a_0
    ])

    Tans_Matrix = np.array([
        [1, 0, -v_0 * Delta_t * np.sin(phi_0), 0, Delta_t * np.cos(phi_0), 0],
        [0, 1, v_0 * Delta_t * np.cos(phi_0), 0, Delta_t * np.sin(phi_0), 0],
        [0, 0, 1, Delta_t, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, Delta_t],
        [0, 0, 0, 0, 0, 1]
    ])

    Q = np.array([
        [Variance_phi_D, 0],
        [0, Varince_a]
    ])

    GQG_T = np.array([
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, Variance_phi_D * Delta_t**2, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, Varince_a * Delta_t**2]
    ])

    P_k_M = (Tans_Matrix @ P_kM1 @ Tans_Matrix.T) + GQG_T

    # Correction step
    Varince_x_gps = 0.01
    Varince_y_gps = 0.01
    Variance_Acc = 0.001
    Variance_gyro = 5 * 10**-5

    if cnt_b < acc_ib_b.shape[0]:  # Kiểm tra để đảm bảo cnt_b không vượt quá giới hạn mảng acc_ib_b
        z_k = np.array([
            UTM[cnt, 0],
            UTM[cnt, 1],
            -acc_ib_b[cnt_b, 0],
            omg_ib_b[cnt_b, 2]
        ])
    else:
        print(f"Chỉ số cnt_b ({cnt_b}) vượt quá giới hạn mảng acc_ib_b ({acc_ib_b.shape[0]})")
        break

    R_gps = np.array([
        [Varince_x_gps, 0, 0, 0],
        [0, Varince_y_gps, 0, 0],
        [0, 0, Variance_Acc, 0],
        [0, 0, 0, Variance_gyro]
    ])

    H = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1],
        [0, 0, 0, 1, 0, 0]
    ])

    h = H @ x_kM

    K_k = P_k_M @ H.T @ np.linalg.inv(H @ P_k_M @ H.T + R_gps)

    x_k = x_kM + K_k @ (z_k - h)

    P_k = (np.eye(6) - K_k @ H) @ P_k_M

    if x_k[0] != 0:
        plt.plot(x_k[0], x_k[1], 'b')

    P_kM1 = P_k_M

    x_0 = x_k[0]
    y_0 = x_k[1]
    phi_0 = x_k[2]
    phi_D_0 = x_k[3]
    v_0 = x_k[4]
    a_0 = x_k[5]

    cnt += Interval
    cnt_b += P_Interval

    # Time fraction
    Delta_t = (time_imu[cnt_b, 0] - time_imu[cnt_b - P_Interval, 0])

plt.xlabel('UTM-East')
plt.ylabel('UTM-North')
plt.legend(['UTM', 'EKF'])
plt.show()
