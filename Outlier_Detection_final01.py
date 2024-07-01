import numpy as np
import pymap3d as pm
from scipy.interpolate import Akima1DInterpolator

class OutlierDetectorInterpolator:
    def __init__(self):
        pass

    def akima_interpolation(self, data):
        indices = np.arange(len(data))
        valid_mask = ~np.isnan(data)
        interpolator = Akima1DInterpolator(indices[valid_mask], data[valid_mask])
        return interpolator(indices)

    def exclude_interpolate_outlier(self, x_wls, v_wls, cov_x, cov_v, threshold=3):
        # Calculate variance for NaN data
        magnitudes_x = np.linalg.norm(x_wls, axis=1)
        x_out = np.nanstd(magnitudes_x)
        x_out_sigma = np.sqrt(x_out**2 / 3)
        
        magnitudes_v = np.linalg.norm(v_wls, axis=1)
        v_out = np.nanstd(magnitudes_v)
        v_out_sigma = np.sqrt(v_out**2 / 3)

        # Coordinate conversion
        x_llh = np.array(pm.ecef2geodetic(x_wls[:, 0], x_wls[:, 1], x_wls[:, 2])).T
        x_llh_mean = np.nanmean(x_llh, axis=0)
        
        v_enu = np.array(pm.ecef2enuv(
            v_wls[:, 0], v_wls[:, 1], v_wls[:, 2], x_llh_mean[0], x_llh_mean[1])).T
        v_enu_valid = v_enu[np.isfinite(v_enu[:, 2])]
        x_llh_valid = x_llh[np.isfinite(x_llh[:, 2])]
        
        # Calculate the upper and lower thresholds
        mean_v = np.mean(v_enu_valid[:, 2])
        std_v = np.std(v_enu_valid[:, 2])
        std_x = np.std(x_llh_valid[:, 2])
        upper_threshold = mean_v + threshold * std_v
        lower_threshold = mean_v - threshold * std_v

        # Up velocity jump detection
        idx_v_out = (v_enu[:, 2] > upper_threshold) | (v_enu[:, 2] < lower_threshold)
        idx_v_out |= np.isnan(v_enu[:, 2])
        v_wls[idx_v_out, :] = np.nan
        cov_v[idx_v_out] = v_out_sigma**2 * np.eye(3)
        nan_count_v = np.sum(np.any(np.isnan(v_wls), axis=1))
        print("Number of outlier velocity: ", nan_count_v)

        # Height check
        hmedian = np.nanmedian(x_llh[:, 2])
        idx_x_out = np.abs(x_llh[:, 2] - hmedian) > threshold * std_x
        idx_x_out |= np.isnan(x_llh[:, 2])
        x_wls[idx_x_out, :] = np.nan
        cov_x[idx_x_out] = x_out_sigma**2 * np.eye(3)
        nan_count_x = np.sum(np.any(np.isnan(x_wls), axis=1))
        print("Number of outlier position: ", nan_count_x)

        # Interpolation Akima for x_wls
        x_wls[:, 0] = self.akima_interpolation(x_wls[:, 0])
        x_wls[:, 1] = self.akima_interpolation(x_wls[:, 1])
        x_wls[:, 2] = self.akima_interpolation(x_wls[:, 2])

        # Interpolation Akima for v_wls
        v_wls[:, 0] = self.akima_interpolation(v_wls[:, 0])
        v_wls[:, 1] = self.akima_interpolation(v_wls[:, 1])
        v_wls[:, 2] = self.akima_interpolation(v_wls[:, 2])

        return x_wls, v_wls, cov_x, cov_v

