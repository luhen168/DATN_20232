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
        # State Extrapolation Equation
        x_pred = self.F @ x + self.G @ u.T  
        # Covariance Extrapolation Equation 
        P_pred = self.F @ P @ self.F.T + Q  
        return x_pred, P_pred

    def update(self, x_pred, P_pred, z, R):
        """
        Func: State update (estimation) step
        Args: x_pred, P_pred, z (measurement), R 
        Returns: x (estimate), P (estimate)
        """
        # Measurement residal
        y = z.T - self.H @ x_pred                                               
        # Kalman Gain
        K = P_pred @ self.H.T @ np.linalg.inv(self.H @ P_pred @ self.H.T + R)   
        # State Update Equation (Estimate)
        x = x_pred + K @ y                
        # Covariance Update Equation                                       
        P = (self.I - K @ self.H) @ P_pred                                      
        return x, P

    def kalman_filter(self, x_wls, v_wls, cov_zs, cov_vs):
        """
        Kalman filter process
        """
        # Initial variable
        n, dim_x = x_wls.shape
        x = x_wls[0, :3].T
        P = 5**2 * np.eye(3)
        x_kf = np.zeros([n, dim_x])
        P_kf = np.zeros([n, dim_x, dim_x])
        x_kf[0] = x.T
        P_kf[0] = P
        vs = np.vstack([np.zeros([1, 3]), (v_wls[:-1] + v_wls[1:])/2])

        for i, (u, z) in enumerate(zip(vs, x_wls)):
            if i == 0:
                continue

            Q = cov_vs[i]
            R = cov_zs[i]

            # Prediction step
            x, P = self.predict(x, u, P, Q)

            # Check outliers for observation
            d = distance.mahalanobis(z, self.H @ x, np.linalg.inv(P))

            # Update step
            if d < self.sigma_mahalanobis:
                x, P = self.update(x, P, z, R)
            else:
                P += 10**2 * Q

            x_kf[i] = x.T 
            P_kf[i] = P

        return x_kf, P_kf
