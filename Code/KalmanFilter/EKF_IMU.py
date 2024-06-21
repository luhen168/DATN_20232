import numpy as np
from quaternion import Quaternion
from utilities import to_cross_matrix, deg_to_rad, g_to_ms2

class ESKF:
    def __init__(self):
        self.init()

    def init(self):
        self.qref = Quaternion()
        self.x = np.zeros(15)
        self.P = np.eye(15)
        self.Q = np.eye(15)
        self.Q[:6, :6] *= 0.05
        self.Q[6:9, 6:9] *= 0.025
        self.Q[9:12, 9:12] *= 0.025
        self.Q[12:, 12:] *= 0.01

        self.R_Gyr = np.eye(3) * deg_to_rad(0.0000045494)
        self.R_Acc = np.eye(3) * g_to_ms2(0.0141615383)
        self.R_Mag = np.eye(3) * 0.00004

    def init_with_acc(self, ax, ay, az):
        self.init()
        roll = np.atan2(ay, sgn(-az) * np.sqrt(az**2 + 0.01 * ax**2))
        pitch = np.atan(-ax / np.sqrt(ay**2 + az**2))
        sr05, cr05 = np.sin(0.5 * roll), np.cos(0.5 * roll)
        sp05, cp05 = np.sin(0.5 * pitch), np.cos(0.5 * pitch)
        self.qref = Quaternion(sr05, 0, 0, cr05) * Quaternion(0, sp05, 0, cp05)

    def predict(self, dt):
        angular_velocity_quat = Quaternion(self.x[12], self.x[13], self.x[14], 0)
        self.qref += dt * (0.5 * angular_velocity_quat * self.qref)
        self.qref.normalize()

        A = np.eye(9)
        A[0:3, 3:6] = dt * np.eye(3)
        A[0:3, 6:9] = (dt**2 / 2) * np.eye(3)
        A[3:6, 6:9] = dt * np.eye(3)

        self.x[:9] = A @ self.x[:9]
        self.P[:9, :9] = A @ self.P[:9, :9] @ A.T + dt * self.Q[:9, :9]

        angular_velocity = self.x[12:15]
        error = self.x[9:12]
        Jac = np.zeros((6, 6))
        Jac[:3, :3] = to_cross_matrix(error)
        Jac[:3, 3:] = -to_cross_matrix(angular_velocity)
        G = np.eye(6)
        G[3:, 3:] *= -1

        self.P[9:, 9:] = Jac @ self.P[9:, 9:] @ Jac.T + G @ dt * self.Q[9:, 9:] @ G.T

    def correct_gyr(self, gx, gy, gz):
        z = np.array([gx, gy, gz]) * deg_to_rad
        K = self.P[12:, 12:] @ np.linalg.inv(self.P[12:, 12:] + self.R_Gyr)
        self.x[12:15] += K @ (z - self.x[12:15])
        self.P[12:, 12:] -= K @ self.P[12:, 12:]

    def correct_acc(self, ax, ay, az):
        z = np.array([ax, ay, az]) * g_to_ms2
        gravity = np.array([0.0, 0.0, -9.81])
        if abs(np.linalg.norm(z) - 9.81) < 0.7:
            z_acc = np.zeros(3)
            K_acc = self.P[6:9, 6:9] @ np.linalg.inv(self.P[6:9, 6:9] + self.R_Acc)
            self.x[6:9] += K_acc @ (z_acc - self.x[6:9])
            self.P[6:9, 6:9] -= K_acc @ self.P[6:9, 6:9]
            vi = gravity
            error = self.x[9:12]
            vb_pred = self.qref.to_rotation_matrix() @ vi
            Ha = to_cross_matrix(vb_pred)
            K = self.P[9:12, 9:12] @ Ha.T @ np.linalg.inv(Ha @ self.P[9:12, 9:12] @ Ha.T + self.R_Acc)
            self.x[9:12] += K @ (z - vb_pred - Ha @ error)
            self.P[9:12, 9:12] -= K @ Ha @ self.P[9:12, 9:12]
        else:
            vi = gravity + self.x[6:9]
            error = self.x[9:12]
            vb_pred = self.qref.to_rotation_matrix() @ vi
            H = np.zeros((3, 6))
            H[:, :3] = to_rotation_matrix(error) @ self.qref.to_rotation_matrix()
            H[:, 3:] = to_cross_matrix(vb_pred)
            K = self.P[6:12, 6:12] @ H.T @ np.linalg.inv(H @ self.P[6:12, 6:12] @ H.T + self.R_Acc)
            self.x[6:12] += K @ (z - vb_pred - H @ error)
            self.P[6:12, 6:12] -= K @ H @ self.P[6:12, 6:12]

    def correct_mag(self, mx, my, mz, incl, B, W, V):
        z = np.array([mx, my, mz])
        vi = np.array([np.cos(incl), 0, -np.sin(incl)]) * B
        error = self.x[9:12]
        vb_pred = self.qref.to_rotation_matrix() @ vi
        h = W @ vb_pred + V
        Ha = W @ to_cross_matrix(vb_pred)
        K = self.P[9:12,
