import numpy as np
from scipy.spatial import distance

class KalmanFilter:
    def __init__(self):
        self.sigma_mahalanobis = 30.0 
        self.F = np.eye(3)  # Transition matrix
        self.G = np.eye(3)  # Control matrix
        self.H = np.eye(3)  # Measurement matrix
        self.I = np.eye(3)  # Identity matrix

    def predict(self, x, u, P, Q):
        """
        Func: Prediction step
        Args: x (initial or estimate), u(velocity), P, Q
        Returns: x_pred, P_pred
        """
        x_pred = self.F @ x + self.G @ u.T  # State Extrapolation Equation
        P_pred = self.F @ P @ self.F.T + Q  # Covariance Extrapolation Equation 
        return x_pred, P_pred

    def estimate(self, x_pred, P_pred, z, R):
        """
        Func: State update (estimation) step
        Args: x_pred, P_pred, z (measurement), R 
        Returns: x (estimate), P (estimate)
        """
        y = z.T - self.H @ x_pred                                               # Measurement residal
        K = P_pred @ self.H.T @ np.linalg.inv(self.H @ P_pred @ self.H.T + R)   # Kalman Gain
        x = x_pred + K @ y                                                      # State Update Equation (Estimate)
        P = (self.I - K @ self.H) @ P_pred                                      # Covariance Update Equation 
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
            R = cov_zs[i]

            # Prediction step
            x, P = self.predict(x, u, P, Q)
            print(f"Iteration {i}: After Prediction Step")
            print(f"x = {x}")
            print(f"P = {P}")

            # Check outliers for observation
            d = distance.mahalanobis(z, self.H @ x, np.linalg.inv(P))
            print(f"d = {d}")

            # Update step
            if d < self.sigma_mahalanobis:
                x, P = self.estimate(x, P, z, R)
                print(f"Iteration {i}: After Update Step (Estimate)")
            else:
                P += 10**2 * Q
                print(f"Iteration {i}: After Update Step (Increase Covariance)")

            print(f"x = {x}")
            print(f"P = {P}")

            x_kf[i] = x.T 
            P_kf[i] = P

        return x_kf, P_kf