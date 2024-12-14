from outlier import exclude_interpolate_outlier
from calculate_score import calc_score, vincenty_distance
from calculate_wls import *
from kalman_filter import KalmanFilter
from tqdm.auto import tqdm
from time import time 
import numpy as np
import pandas as pd
import scipy.optimize
import pymap3d as pm
import glob as gl
import pymap3d.vincenty as pmv
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# Constants
CLIGHT = 299_792_458   # speed of light (m/s)
RE_WGS84 = 6_378_137   # earth semimajor axis (WGS84) (m)
OMGE = 7.2921151467E-5  # earth angular velocity (IS-GPS) (rad/s)



def satellite_selection(df):
    """
    Args:
        df : DataFrame from device_gnss.csv
        column : Column name
    Returns:
        df: DataFrame with eliminated satellite signals
    """
    idx = df['RawPseudorangeMeters'].notnull()
    idx &= df['CarrierErrorHz'] < 2.0e6  # carrier frequency error (Hz)
    idx &= df['Cn0DbHz'] > 18.0  # C/N0 (dB-Hz)
    idx &= df['MultipathIndicator'] == 0 # Multipath flag
    idx &= df['ReceivedSvTimeUncertaintyNanos'] < 500

    return df[idx]

# Chuyển các mảng numpy thành các cột dữ liệu phù hợp để lưu vào file CSV
def flatten_covariances(covariances):
    """Chuyển đổi ma trận hiệp phương sai thành danh sách các giá trị"""
    flattened = []
    for cov in covariances:
        flattened.append(cov.flatten())
    return np.array(flattened)

def ecef_to_enu(x_wls, v_wls, ref_lat, ref_lon, ref_alt):
    """Chuyển đổi tọa độ ECEF sang ENU."""
    enu_positions = []
    enu_velocities = []

    # Lấy ma trận chuyển đổi từ ECEF sang ENU
    for (x, y, z), (vx, vy, vz)in zip(x_wls, v_wls):
        e, n, u = pm.ecef2enu(x, y, z, ref_lat, ref_lon, ref_alt)
        enu_positions.append([e, n, u])
        ve, vn, vu = pm.ecef2enu(vx, vy, vz, ref_lat, ref_lon, ref_alt)
        enu_velocities.append([ve, vn, vu])

    return (np.array(enu_positions), np.array(enu_velocities))

def enu_to_geodetic(enu_positions, ref_lat, ref_lon, ref_alt):
    """Chuyển đổi tọa độ ENU sang lat, long, alt."""
    latitudes = []
    longitudes = []
    altitudes = []
    for e, n, u in enu_positions:
        lat, lon, alt = pm.enu2geodetic(e, n, u, ref_lat, ref_lon, ref_alt)
        latitudes.append(lat)
        longitudes.append(lon)
        altitudes.append(alt)
    return np.array(latitudes), np.array(longitudes), np.array(altitudes)

def ecef_to_geodetic(ecef_positions, ref_lat, ref_lon, ref_alt):
    """Chuyển đổi tọa độ ENU sang lat, long, alt."""
    latitudes = []
    longitudes = []
    altitudes = []
    for x, y, z in ecef_positions:
        lat, lon, alt = pm.enu2geodetic(x, y, z, ref_lat, ref_lon, ref_alt)
        latitudes.append(lat)
        longitudes.append(lon)
        altitudes.append(alt)
    return np.array(latitudes), np.array(longitudes), np.array(altitudes)

def main():
    # path = '2020-06-25-00-34-us-ca-mtv-sb-101/pixel4xl'
    # path = '2021-07-19-20-49-us-ca-mtv-a/pixel4'
    # path = '2021-07-19-20-49-us-ca-mtv-a/mi8'
    # path = '2021-07-19-20-49-us-ca-mtv-a/pixel5'
    # path = '2021-07-19-20-49-us-ca-mtv-a/sm-g988b'
    # path = '2023-09-07-19-33-us-ca/pixel4xl'
    # path = '2023-09-07-19-33-us-ca/pixel5a'
    paths = [
        '2020-06-25-00-34-us-ca-mtv-sb-101/pixel4xl',
        # '2021-07-19-20-49-us-ca-mtv-a/pixel4',
        # '2021-07-19-20-49-us-ca-mtv-a/mi8',
        # '2021-07-19-20-49-us-ca-mtv-a/pixel5',
        '2021-07-19-20-49-us-ca-mtv-a/sm-g988b',
        # '2023-09-07-19-33-us-ca/pixel4xl',
        '2023-09-07-19-33-us-ca/pixel5a'
    ]

    # Lists to store scores
    all_scores_bl = []
    all_scores_wls = []
    all_scores_kf = []
    all_scores_rtk = []
    
    for path in paths:
        # gnss = pd.read_csv('data/%s/device_gnss.csv' % path, dtype={'SignalType': str})
        # gt = pd.read_csv('data/%s/ground_truth.csv' % path, dtype={'SignalType': str})
        # rtk = pd.read_csv('GSDC_2023/data/locations_train_06_03.csv', dtype={'SignalType': str})
        # rtk_train = rtk[rtk['tripId'] == path]
        print(f'Processing path: {path}')
        gnss = pd.read_csv(f'data/{path}/device_gnss.csv', dtype={'SignalType': str})
        gt = pd.read_csv(f'data/{path}/ground_truth.csv', dtype={'SignalType': str})
        rtk = pd.read_csv('GSDC_2023/data/locations_train_06_03.csv', dtype={'SignalType': str})
        rtk_train = rtk[rtk['tripId'] == path]

        # Add standard Frequency column
        frequency_median = gnss.groupby('SignalType')['CarrierFrequencyHz'].median()
        gnss = gnss.merge(frequency_median, how='left', on='SignalType', suffixes=('', 'Ref'))
        carrier_error = abs(gnss['CarrierFrequencyHz'] - gnss['CarrierFrequencyHzRef'])
        gnss['CarrierErrorHz'] = carrier_error
        utcTimeMillis = gnss['utcTimeMillis'].unique()
        nepoch = len(utcTimeMillis)
        gt_len = len(gt)
        x0 = np.zeros(4)  # [x,y,z,tGPSL1]
        v0 = np.zeros(4)  # [vx,vy,vz,dtGPSL1]
        x_wls = np.full([nepoch, 3], np.nan)  # For saving position
        v_wls = np.full([nepoch, 3], np.nan)  # For saving velocity
        cov_x = np.full([nepoch, 3, 3], np.nan) # For saving position covariance
        cov_v = np.full([nepoch, 3, 3], np.nan) # For saving velocity covariance  
        score_wls = []
        score_bl = [] 
        score_rtk = []
        score_kf = []
        path_names = []

        for i, (t_utc, df) in enumerate(tqdm(gnss.groupby('utcTimeMillis'), total=nepoch)):
            if (i ==0) and (nepoch != gt_len) : continue  #First position is not in ground truth in some phone
            df_pr = satellite_selection(df)
            df_prr = satellite_selection(df)

            # Corrected pseudorange/pseudorange rate
            pr = (df_pr['RawPseudorangeMeters'] + df_pr['SvClockBiasMeters'] - df_pr['IsrbMeters'] -
                df_pr['IonosphericDelayMeters'] - df_pr['TroposphericDelayMeters']).to_numpy()
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

            # Robust WLS requires accurate initial values for convergence,
            # so perform normal WLS for the first time
            if len(df_pr) >= 4:
                # Normal WLS
                if np.all(x0 == 0):
                    opt = scipy.optimize.least_squares(
                        f_wls, x0, jac_pr_residuals, args=(xsat_pr, pr, Wx))
                    x0 = opt.x 
                # Robust WLS for position estimation
                opt = scipy.optimize.least_squares(
                    f_wls, x0, jac_pr_residuals, args=(xsat_pr, pr, Wx), loss='soft_l1')
                if opt.status < 1 or opt.status == 2:
                    print(f'i = {i} position lsq status = {opt.status}')
                else:
                    # Covariance estimation
                    cov = np.linalg.inv(opt.jac.T @ Wx @ opt.jac)
                    cov_x[i, :, :] = cov[:3, :3]
                    x_wls[i, :] = opt.x[:3]
                    x0 = opt.x
                    
            # Velocity estimation
            if len(df_prr) >= 4:
                if np.all(v0 == 0): # Normal WLS
                    opt = scipy.optimize.least_squares(
                        prr_residuals, v0, jac_prr_residuals, args=(vsat, prr, x0, xsat_prr, Wv))
                    v0 = opt.x
                # Robust WLS for velocity estimation
                opt = scipy.optimize.least_squares(
                    prr_residuals, v0, jac_prr_residuals, args=(vsat, prr, x0, xsat_prr, Wv), loss='soft_l1')
                if opt.status < 1:
                    print(f'i = {i} velocity lsq status = {opt.status}')
                else:
                    # Covariance estimation
                    cov = np.linalg.inv(opt.jac.T @ Wv @ opt.jac)
                    cov_v[i, :, :] = cov[:3, :3]
                    v_wls[i, :] = opt.x[:3]
                    v0 = opt.x

            
            #RTK
            if nepoch == gt_len :
                llh_rtk = rtk_train[['LatitudeDegrees', 'LongitudeDegrees']].to_numpy()[i]
            else: 
                llh_rtk = rtk_train[['LatitudeDegrees', 'LongitudeDegrees']].to_numpy()[i-1]
            # Baseline
            bl = gnss[gnss['utcTimeMillis'] == utcTimeMillis[i]]
            x_bl = bl[['WlsPositionXEcefMeters', 'WlsPositionYEcefMeters', 'WlsPositionZEcefMeters']].mean().to_numpy()
            llh_bl = np.array(pm.ecef2geodetic(x_bl[ 0], x_bl[1], x_bl[2])).T
            # Ground Truth
            llh_wls = np.array(pm.ecef2geodetic(x_wls[i,0], x_wls[i,1], x_wls[i,2])).T
            gt_i = gt[gt['UnixTimeMillis'] == utcTimeMillis[i]]
            llh_gt = gt_i[['LatitudeDegrees', 'LongitudeDegrees']].to_numpy()[0]
            if not np.isnan(llh_wls).any():
                score_wls.append(vincenty_distance(llh_wls, llh_gt))
            score_bl.append(vincenty_distance(llh_bl, llh_gt))
            score_rtk.append(vincenty_distance(llh_gt, llh_rtk))

        #Remove 1st position to compare to ground truth
        if nepoch != gt_len :
            x_wls = x_wls[1:,:]
            v_wls = v_wls[1:,:]
            cov_x = cov_x[1:,:,:]
            cov_v = cov_v[1:,:,:]

        # Exclude velocity outliers
        x_wls, v_wls, cov_x, cov_v = exclude_interpolate_outlier(x_wls, v_wls, cov_x, cov_v)
        """
            Lưu trữ lại giá trị ecef
        """
        #Đảm bảo tất cả các mảng đều có cùng độ dài
        # min_length = min(len(utcTimeMillis), len(x_wls), len(v_wls), len(cov_x), len(cov_v))
        # utcTimeMillis = utcTimeMillis[:min_length]
        # ecef_positions = x_wls[:min_length]
        # ecef_velocities = v_wls[:min_length]
        # cov_x = cov_x[:min_length]
        # cov_v = cov_v[:min_length]

        # # # Chuyển đổi ma trận hiệp phương sai thành danh sách các giá trị
        # # flattened_cov_x = flatten_covariances(cov_x)
        # # flattened_cov_v = flatten_covariances(cov_v)

        # # # Tạo DataFrame với các dữ liệu cần thiết
        # result_df = pd.DataFrame({
        #     'utcTimeMillis': utcTimeMillis,
        #     'x': ecef_positions[:, 0],
        #     'y': ecef_positions[:, 1],
        #     'z': ecef_positions[:, 2],
        #     'v_x': ecef_velocities[:, 0],
        #     'v_y': ecef_velocities[:, 1],
        #     'v_z': ecef_velocities[:, 2],
        # #     'cov_x1': flattened_cov_x[:, 0],
        # #     'cov_x2': flattened_cov_x[:, 1],
        # #     'cov_x3': flattened_cov_x[:, 2],
        # #     'cov_x4': flattened_cov_x[:, 3],
        # #     'cov_x5': flattened_cov_x[:, 4],
        # #     'cov_x6': flattened_cov_x[:, 5],
        # #     'cov_x7': flattened_cov_x[:, 6],
        # #     'cov_x8': flattened_cov_x[:, 7],
        # #     'cov_x9': flattened_cov_x[:, 8],
        # #     'cov_v1': flattened_cov_v[:, 0],
        # #     'cov_v2': flattened_cov_v[:, 1],
        # #     'cov_v3': flattened_cov_v[:, 2],
        # #     'cov_v4': flattened_cov_v[:, 3],
        # #     'cov_v5': flattened_cov_v[:, 4],
        # #     'cov_v6': flattened_cov_v[:, 5],
        # #     'cov_v7': flattened_cov_v[:, 6],
        # #     'cov_v8': flattened_cov_v[:, 7],
        # #     'cov_v9': flattened_cov_v[:, 8],
        # })

        # #Lưu DataFrame vào file CSV
        # export_name = 'wls_ecef.csv'
        # result_df.to_csv(export_name, index=False)
        # print(f'Data successfully exported to {export_name}')

        llh_wls = np.array(pm.ecef2geodetic(x_wls[:,0], x_wls[:,1], x_wls[:,2])).T
        # position_array = [(time, lat, lon, alt) for time, (lat, lon, alt) in zip(utcTimeMillis, llh_wls)]
        # df = pd.DataFrame(position_array, columns=["utcTimeMillis", "Latitude", "Longitude", "Altitude"])
        # df.to_csv('llh_wls.csv', index=False)

        # Calculate score KF
        kf = KalmanFilter()
        x_kf, _ = kf.kalman_filter(x_wls, v_wls, cov_x, cov_v)               
        # # Tạo DataFrame với các dữ liệu cần thiết
        # result_df = pd.DataFrame({
        #     'utcTimeMillis': utcTimeMillis,
        #     'x': x_kf[:, 0],
        #     'y': x_kf[:, 1],
        #     'z': x_kf[:, 2],
        # })

        # #Lưu DataFrame vào file CSV
        # export_name = 'kf_ecef.csv'
        # result_df.to_csv(export_name, index=False)
        # print(f'Data successfully exported to {export_name}')


        llh_kf = np.array(pm.ecef2geodetic(x_kf[:,0], x_kf[:,1], x_kf[:,2])).T
        position_array = [(time, lat, lon, alt) for time, (lat, lon, alt) in zip(utcTimeMillis, llh_kf)]
        df = pd.DataFrame(position_array, columns=["utcTimeMillis", "Latitude", "Longitude", "Altitude"])
        df.to_csv('llh_kf.csv', index=False)
        llh_gt = gt[['LatitudeDegrees', 'LongitudeDegrees']].to_numpy()
        for i in range(len(llh_gt)):
            score_kf_i = vincenty_distance(llh_gt[i], llh_kf[i])
            score_kf.append(score_kf_i)


        # Calculate score all method
        score_all_wls = np.mean([np.quantile(score_wls, 0.50), np.quantile(score_wls, 0.95)])
        score_all_bl = np.mean([np.quantile(score_bl, 0.50), np.quantile(score_bl, 0.95)])
        score_all_kf = np.mean([np.quantile(score_kf, 0.50), np.quantile(score_kf, 0.95)])
        score_all_rtk = np.mean([np.quantile(score_rtk, 0.50), np.quantile(score_rtk, 0.95)])

        all_scores_bl.append(score_all_bl)
        all_scores_wls.append(score_all_wls)
        all_scores_kf.append(score_all_kf)
        all_scores_rtk.append(score_all_rtk)
        path_names.append(path.split('/')[-1])  # Lấy tên tệp từ đường dẫn

        print(f'Path: {path}')
        print("Baseline score: ", score_all_bl)
        # print("WLS score: ", score_all_wls)
        print("KF score: ", score_all_kf)
        # print("RTK score: ", score_all_rtk)


        #Plot distance error
        plt.figure()
        plt.title(f'Distance error for {path}')
        plt.ylabel('Distance error [m]')
        plt.plot(score_bl, label=f'Baseline, Score: {score_all_bl:.4f} m')
        plt.plot(score_wls, label=f'WLS, Score: {score_all_wls:.4f} m')
        plt.plot(score_kf, label=f'KF, Score: {score_all_kf:.4f} m')
        plt.plot(score_rtk, label=f'RTK, Score: {score_all_rtk:.4f} m')
        plt.legend()
        plt.grid()
        plt.ylim([0, 30])
        plt.show()

 # Plot distance error for all paths
    # bar_width = 0.2
    # index = np.arange(len(path_names))

    # plt.figure()
    # plt.title('Distance error for all paths')
    # plt.ylabel('Distance error [m]')
    # plt.xlabel('Path index')

    # plt.bar(index, all_scores_bl, bar_width, label='Baseline')
    # plt.bar(index + bar_width, all_scores_wls, bar_width, label='WLS')
    # plt.bar(index + 2 * bar_width, all_scores_kf, bar_width, label='KF')
    # plt.bar(index + 3 * bar_width, all_scores_rtk, bar_width, label='RTK')

    # plt.xticks(index + 1.5 * bar_width, path_names, rotation=45)
    # plt.legend()
    # plt.grid()
    # plt.ylim([0, 30])
    # plt.show()

if __name__ == "__main__":
    main()