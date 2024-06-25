# 

import numpy as np
from scipy.spatial import distance

class KalmanFilter:
    def __init__(self):
        self.sigma_mahalanobis = 30.0 
        self.F = np.eye(3)  # Transition matrix
        self.G = np.eye(3)  # Control matrix
        self.H = np.eye(3)  # Measurement matrix
        self.I = np.eye(3)  # Identity matrix
        self.residuals = []  # List to store residuals for covariance calculation

    def predict(self, x, u, P, Q):
        """
        Func: Prediction step
        Args: x (initial or estimate), u(velocity), P, Q
        Returns: x_pred, P_pred
        """
        x_pred = self.F @ x + self.G @ u.T  # State Extrapolation Equation
        P_pred = self.F @ P @ self.F.T + Q  # Covariance Extrapolation Equation 
        return x_pred, P_pred

    def estimate(self, x_pred, P_pred, z):
        """
        Func: State update (estimation) step
        Args: x_pred, P_pred, z (measurement)
        Returns: x (estimate), P (estimate)
        """
        y = z.T - self.H @ x_pred  # Measurement residual 
        self.residuals.append(y)
        if len(self.residuals) > 1:
            R = np.cov(np.array(self.residuals).T)
        else:
            R = np.eye(self.H.shape[0])  # Initial R if not enough residuals

        K = P_pred @ self.H.T @ np.linalg.inv(self.H @ P_pred @ self.H.T + R)  # Kalman Gain
        x = x_pred + K @ y  # State Update Equation (Estimate)
        P = (self.I - K @ self.H) @ P_pred  # Covariance Update Equation 
        return x, P

    def kalman_filter(self, zs, vs, cov_zs, cov_vs):
        """
        Kalman filter process
        """
        # Initial variable
        n, dim_x = zs.shape
        x = zs[0, :3].T
        P = 5**2 * np.eye(3)
        x_kf = np.zeros([n, dim_x])
        P_kf = np.zeros([n, dim_x, dim_x])
        x_kf[0] = x.T
        P_kf[0] = P

        for i, (u, z) in enumerate(zip(vs, zs)):
            if i == 0:
                continue

            Q = cov_vs[i]

            # Prediction step
            x, P = self.predict(x, u, P, Q)

            # Check outliers for observation
            d = distance.mahalanobis(z, self.H @ x, np.linalg.inv(P))

            # Update step
            if d < self.sigma_mahalanobis:
                x, P = self.estimate(x, P, z)
            else:
                P += 10**2 * Q

            x_kf[i] = x.T 
            P_kf[i] = P

        return x_kf, P_kf
