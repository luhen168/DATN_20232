import numpy as np
import scipy.optimize

class WLS:
    def __init__(self):
        self.CLIGHT = 299_792_458   # speed of light (m/s)
        self.RE_WGS84 = 6_378_137   # earth semimajor axis (WGS84) (m)
        self.OMGE = 7.2921151467E-5  # earth angular velocity (IS-GPS) (rad/s)
    
    def satellite_selection(self, df, column):
        """
        Args:
            df : DataFrame each epoch
            column : Column name
        Returns:
            df: DataFrame with eliminated satellite signals
        """
        idx = df[column].notnull()
        # idx &= df['CarrierErrorHz'] < 2.0e6  # carrier frequency error (Hz)
        # idx &= df['SvElevationDegrees'] > 10.0  # elevation angle (deg)
        idx &= df['Cn0DbHz'] > 18.0  # C/N0 (dB-Hz)
        idx &= df['MultipathIndicator'] == 0 # Multipath flag
        return df[idx]
    
    def los_vector(self, xusr, xsat):
        """
        Args:
            xusr : user position in ECEF (m)
            xsat : satellite position in ECEF (m)
        Returns:
            u: unit line-of-sight vector in ECEF (m)
            rng: distance between user and satellite (m)
        """
        u = xsat - xusr
        rng = np.linalg.norm(u, axis=1).reshape(-1, 1)
        u /= rng
        return u, rng.reshape(-1)
    
    def func_wls(self, x_rcv, x_sat, pr_obs, w):
        """
        Compute pseudorange residuals
        """
        b = x_rcv[3]
        r = pr_obs - b  # distance to each satellite [m]
        tau = r / self.CLIGHT  # signal flight time

        x = np.empty_like(x_sat)
        cosO = np.cos(self.OMGE * tau)
        sinO = np.sin(self.OMGE * tau)
        x[:, 0] =  cosO * x_sat[:, 0] + sinO * x_sat[:, 1]
        x[:, 1] = -sinO * x_sat[:, 0] + cosO * x_sat[:, 1]
        x[:, 2] = x_sat[:, 2]
        return w @ (np.sqrt(np.sum((x - x_rcv[:3])**2, axis=1)) - r)
    
    def jac_pr_residuals(self, x, xsat, pr, W):
        """
        Args:
            x : current position in ECEF (m)
            xsat : satellite position in ECEF (m)
            pr : pseudorange (m)
            W : weight matrix
        Returns:
            W*J : Jacobian matrix
        """
        u, _ = self.los_vector(x[:3], xsat)
        J = np.hstack([-u, np.ones([len(pr), 1])])  # J = [-ux -uy -uz 1]
        return W @ J
    
    # Compute pseudorange rate residuals
    def prr_residuals(self, v, vsat, prr, x, xsat, W):
        """
        Args:
            v : current velocity in ECEF (m/s)
            vsat : satellite velocity in ECEF (m/s)
            prr : pseudorange rate (m/s)
            x : current position in ECEF (m)
            xsat : satellite position in ECEF (m)
            W : weight matrix
        Returns:
            residuals*W : pseudorange rate residuals
        """
        u, rng = self.los_vector(x[:3], xsat)
        rate = np.sum((vsat-v[:3])*u, axis=1) \
            + self.OMGE / self.CLIGHT * (vsat[:, 1] * x[0] + xsat[:, 1] * v[0]
                            - vsat[:, 0] * x[1] - xsat[:, 0] * v[1])

        residuals = rate - (prr - v[3])

        return residuals @ W
    def jac_prr_residuals(self, v, vsat, prr, x, xsat, W):
        """
        Args:
            v : current velocity in ECEF (m/s)
            vsat : satellite velocity in ECEF (m/s)
            prr : pseudorange rate (m/s)
            x : current position in ECEF (m)
            xsat : satellite position in ECEF (m)
            W : weight matrix
        Returns:
            W*J : Jacobian matrix
        """
        u, _ = self.los_vector(x[:3], xsat)
        J = np.hstack([-u, np.ones([len(prr), 1])])
        return W @ J
    
    def WLS_onePosition(self, dataFrame):
        x0 = np.zeros(4)  # [x,y,z,tGPSL1]
        v0 = np.zeros(4)  # [vx,vy,vz,dtGPSL1]
        x_wls = np.full(3, np.nan)  # For saving position
        v_wls = np.full(3, np.nan)  # For saving velocity
        cov_x = np.full([3, 3], np.nan) # For saving position covariance
        cov_v = np.full([3, 3], np.nan) # For saving velocity covariance

        # Valid satellite selection
        df_pr = self.satellite_selection(dataFrame, 'RawPseudorangeMeters')
        df_prr = self.satellite_selection(dataFrame, 'PseudorangeRateMetersPerSecond')

        # Corrected pseudorange/pseudorange rate
        pr = (df_pr['RawPseudorangeMeters'] + df_pr['SvClockBiasMeters'] - df_pr['IsrbMeters']).to_numpy()
        prr = (df_prr['PseudorangeRateMetersPerSecond'] +
               df_prr['SvClockDriftMetersPerSecond']).to_numpy()
        
        # Satellite position/velocity
        xsat_pr = df_pr[['SvPositionXEcefMeters', 'SvPositionYEcefMeters',  
                             'SvPositionZEcefMeters']].to_numpy()
        xsat_prr = df_prr[['SvPositionXEcefMeters', 'SvPositionYEcefMeters',
                           'SvPositionZEcefMeters']].to_numpy()
        vsat = df_prr[['SvVelocityXEcefMetersPerSecond', 'SvVelocityYEcefMetersPerSecond',
                       'SvVelocityZEcefMetersPerSecond']].to_numpy()
        
         # Weight matrix for peseudorange/pseudorange rate
        Wx = np.diag(1 / df_pr['RawPseudorangeUncertaintyMeters'].to_numpy())
        Wv = np.diag(1 / df_prr['PseudorangeRateUncertaintyMetersPerSecond'].to_numpy())

        # Robust WLS
        if len(df_pr) >= 4:
            # Normal WLS
            if np.all(x0 == 0):
                opt = scipy.optimize.least_squares(
                    self.func_wls, x0, self.jac_pr_residuals, args=(xsat_pr, pr, Wx))
                x0 = opt.x 
            # Robust WLS for position estimation
            opt = scipy.optimize.least_squares(
                    self.func_wls, x0, self.jac_pr_residuals, args=(xsat_pr, pr, Wx), loss='soft_l1')
            if opt.status < 1 or opt.status == 2:
                print(f'position lsq status = {opt.status}\n') # Error convergence
            else:
                cov = np.linalg.inv(opt.jac.T @ Wx @ opt.jac)
                cov_x[:, :] = cov[:3, :3]
                x_wls[:] = opt.x[:3]
                x0 = opt.x
        else:
            print("not enough satellite") #Error

        # Velocity estimation
        if len(df_prr) >= 4:
            # Normal WLS
            if np.all(v0 == 0): 
                opt = scipy.optimize.least_squares(
                    self.prr_residuals, v0, self.jac_prr_residuals, args=(vsat, prr, x0, xsat_prr, Wv))
                v0 = opt.x
            # Robust WLS for velocity estimation
            opt = scipy.optimize.least_squares(
                self.prr_residuals, v0, self.jac_prr_residuals, args=(vsat, prr, x0, xsat_prr, Wv), loss='soft_l1')
            if opt.status < 1:
                print(f'velocity lsq status = {opt.status}')
            else:
                # Covariance estimation
                cov = np.linalg.inv(opt.jac.T @ Wv @ opt.jac)
                cov_v[:, :] = cov[:3, :3]
                v_wls[:] = opt.x[:3]
                v0 = opt.x
        else:
            print("not enough satellite") #Error
        return x_wls#, v_wls, cov_x, cov_v

    def WLS_onePosition_rawPseudo(self, dataFrame):
            x0 = np.zeros(4)  # [x,y,z,tGPSL1]
            x_wls = np.full(3, np.nan)  # For saving position

            # Valid satellite selection
            df_pr = self.satellite_selection(dataFrame, 'RawPseudorangeMeters')

            # Corrected pseudorange/pseudorange rate
            pr = (df_pr['RawPseudorangeMeters'] + df_pr['SvClockBiasMeters']).to_numpy()
            
            # Satellite position/velocity
            xsat_pr = df_pr[['SvPositionXEcefMeters', 'SvPositionYEcefMeters',  
                                'SvPositionZEcefMeters']].to_numpy()

            # Weight matrix for peseudorange/pseudorange rate
            Wx = np.diag(1 / df_pr['RawPseudorangeUncertaintyMeters'].to_numpy())

            # Robust WLS
            if len(df_pr) >= 4:
                # Normal WLS
                if np.all(x0 == 0):
                    opt = scipy.optimize.least_squares(
                        self.func_wls, x0, self.jac_pr_residuals, args=(xsat_pr, pr, Wx))
                    x0 = opt.x 
                # Robust WLS for position estimation
                opt = scipy.optimize.least_squares(
                        self.func_wls, x0, self.jac_pr_residuals, args=(xsat_pr, pr, Wx), loss='soft_l1')
                if opt.status < 1 or opt.status == 2:
                    print(f'position lsq status = {opt.status}\n') # Error convergence
                else:
                    x_wls[:] = opt.x[:3]
            else:
                print("not enough satellite") #Error
            return x_wls