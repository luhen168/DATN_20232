import numpy as np
from scipy.spatial import distance
# zs: array observe or measurement , each row is 3 dimension x,y,z
# us: array control (veclocity) , each row is 3 dimension v_x,v_y,v_z
# cov_zs: array stored covariance matrix of zs
# cov_us: array stored covariance matrix of us
def kalman_filter(zs, us, cov_zs, cov_us):
    # Initial parameter
    sigma_mahalanobis = 30.0            # Mahalanobis distance for rejecting innovation

    n, dim_x = zs.shape                 # n: number vector and dim_x: is dimension of each vecto 
    F = np.eye(3)                       # Transition matrix 3x3: use Identify Matrix
    G = np.eye(3)                       # Control matrix for velocity 3x3: use Identify Matrix
    H = np.eye(3)                       # Measurement matrix 3x3 state selection: use Identify Matrix

    # Initial state and covariance
    x = zs[0, :3].T                     # get pos 1st until have 3 elements and use T to trans row to column 3x1
    P = 5**2 * np.eye(3)                # Covariance estimate 
    I = np.eye(dim_x)                   # Identify Matrix                           

    x_kf = np.zeros([n, dim_x])         # Matrix zero n row and dim_x column      
    P_kf = np.zeros([n, dim_x, dim_x])  # n Matrix include: dim_x row and dim_x column 

    #Initial for first epoch
    x_kf[0] = x.T                       # pos 1st of x_kf equal x trans
    P_kf[0] = P                         # pos 1st of P_kf equal P

    # Kalman filtering
    # each index i have tuple u[x,y,z]:vecto velocity and z[x_z,y_z,z_z]: vecto measurement
    for i, (u, z) in enumerate(zip(us, zs)):        
        if i == 0:                      # if the first data to next step
            continue

        # Prediction step
        x = F @ x + G @ u.T         # State vecto  
        Q = cov_us[i]               # Estimated WLS velocity covariance
        P = F @ P @ F.T + Q         # Covariance 

        # # Check outliers for observation
        d = distance.mahalanobis(z, H @ x, np.linalg.inv(P))

        # Update step
        if d < sigma_mahalanobis:
            R = cov_zs[i]               # Estimated WLS position covariance
            y = z.T - H @ x
            K = (P @ H.T)  @ np.linalg.inv(H @ (P @ H.T) + R)
            x = x + K @ y
            P = (I - (K @ H)) @ P
        else:
            # If observation update is not available, increase covariance
            P += 10**2*Q

        # Output
        x_kf[i] = x.T
        P_kf[i] = P

    return x_kf, P_kf

# Forward + backward Kalman filter and smoothing
def kalman_smoothing(x_wls, v_wls, cov_x, cov_v):

    #Get the velocity between 2 epoch (median)
    v = np.vstack([np.zeros([1, 3]), (v_wls[:-1] + v_wls[1:])/2])
    x_f, P_f = kalman_filter(x_wls, v, cov_x, cov_v)    
    return x_f, P_f

