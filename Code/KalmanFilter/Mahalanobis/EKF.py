# Cài đặt bộ lọc Kalman mở rộng (EKF) cho hệ tọa độ ECEF
class ECEF_EKF:
    def __init__(self, dt, process_noise_std, measurement_noise_std):
        self.dt = dt  # Thời gian giữa các lần đo
        self.Q = process_noise_std**2 * np.eye(6)  # Hiệp phương sai nhiễu quá trình
        self.R = measurement_noise_std**2 * np.eye(3)  # Hiệp phương sai nhiễu đo lường
        self.x = np.zeros((6, 1))  # Trạng thái ban đầu [x, y, z, vx, vy, vz]
        self.P = np.eye(6)  # Hiệp phương sai trạng thái ban đầu

    def predict(self, u):
        F = np.eye(6)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt

        B = np.zeros((6, 3))
        B[3, 0] = self.dt
        B[4, 1] = self.dt
        B[5, 2] = self.dt

        self.x = F.dot(self.x) + B.dot(u)
        self.P = F.dot(self.P).dot(F.T) + self.Q

    def update(self, z):
        H = np.zeros((3, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1

        y = z.reshape(3, 1) - H.dot(self.x)
        S = H.dot(self.P).dot(H.T) + self.R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))

        self.x = self.x + K.dot(y)
        self.P = self.P - K.dot(H).dot(self.P)