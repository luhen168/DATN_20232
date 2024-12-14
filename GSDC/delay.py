import numpy as np  
import pandas as pd 
import os, csv, sys
import glob
import subprocess
import pymap3d as pm
import pymap3d.vincenty as pmv
from tqdm.notebook import tqdm
from dataclasses import dataclass
from scipy.interpolate import InterpolatedUnivariateSpline
from datetime import datetime, timezone, timedelta
import scipy.optimize
from tqdm.auto import tqdm
from time import time 


# Constants
CLIGHT = 299_792_458   # speed of light (m/s)
RE_WGS84 = 6_378_137   # earth semimajor axis (WGS84) (m)
OMGE = 7.2921151467E-5  # earth angular velocity (IS-GPS) (rad/s)


# path = 'device_gnss_2020_6.csv'
# gnss = pd.read_csv(path)
# selected_col = ['utcTimeMillis','SvElevationDegrees','SvAzimuthDegrees','RawPseudorangeMeters']
# measurement = gnss[selected_col]


path = '2020-06-25-00-34-us-ca-mtv-sb-101'
gnss_all = pd.read_csv('%s/device_gnss.csv' % path, dtype={'SignalType': str})
gt = pd.read_csv('%s/ground_truth.csv' % path, dtype={'SignalType': str})



def jac_prr_residuals(v, vsat, prr, x, xsat, W):
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
    u, _ = los_vector(x[:3], xsat)
    J = np.hstack([-u, np.ones([len(prr), 1])])

    return W @ J


# Compute pseudorange rate residuals
def prr_residuals(v, vsat, prr, x, xsat, W):
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
    u, rng = los_vector(x[:3], xsat)
    rate = np.sum((vsat-v[:3])*u, axis=1) \
          + OMGE / CLIGHT * (vsat[:, 1] * x[0] + xsat[:, 1] * v[0]
                           - vsat[:, 0] * x[1] - xsat[:, 0] * v[1])

    residuals = rate - (prr - v[3])

    return residuals @ W

def los_vector(xusr, xsat):
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

def jac_pr_residuals(x, xsat, pr, W):
    """
    Args:
        x : current position in ECEF (m)
        xsat : satellite position in ECEF (m)
        pr : pseudorange (m)
        W : weight matrix
    Returns:
        W*J : Jacobian matrix
    """
    u, _ = los_vector(x[:3], xsat)
    J = np.hstack([-u, np.ones([len(pr), 1])])  # J = [-ux -uy -uz 1]

    return W @ J

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

    return df[idx]

def f_wls(x_rcv,x_sat,pr_obs,w):
    """
    Compute error for guess y

    x_rcv (x1, x2, x3, b):
      x_rcv: receiver position at receiving time
      b: receiver clock bias in meters
    """
    b = x_rcv[3]
    r = pr_obs - b  # distance to each satellite [m]
    tau = r / CLIGHT  # signal flight time

    # Rotate satellite positions at emission for present ECEF coordinate
    x = np.empty_like(x_sat)
    cosO = np.cos(OMGE * tau)
    sinO = np.sin(OMGE * tau)
    x[:, 0] =  cosO * x_sat[:, 0] + sinO * x_sat[:, 1]
    x[:, 1] = -sinO * x_sat[:, 0] + cosO * x_sat[:, 1]
    x[:, 2] = x_sat[:, 2]

    return w @ (np.sqrt(np.sum((x - x_rcv[:3])**2, axis=1)) - r)

start_time = time()
gnss = gnss_all[gnss_all['utcTimeMillis'] == 1593045252440]
# Add standard Frequency column
frequency_median = gnss.groupby('SignalType')['CarrierFrequencyHz'].median()
gnss = gnss.merge(frequency_median, how='left', on='SignalType', suffixes=('', 'Ref'))
carrier_error = abs(gnss['CarrierFrequencyHz'] - gnss['CarrierFrequencyHzRef'])
gnss['CarrierErrorHz'] = carrier_error
gnss = satellite_selection(gnss)


x0 = np.zeros(4)  # [x,y,z,tGPSL1]
v0 = np.zeros(4)  # [vx,vy,vz,dtGPSL1]
x_wls = np.full(3, np.nan)  # For saving position
v_wls = np.full(3, np.nan)  # For saving velocity
cov_x = np.full([3, 3], np.nan) # For saving position covariance
cov_v = np.full([3, 3], np.nan) # For saving velocity covariance
# Corrected pseudorange/pseudorange rate
pr = (gnss['RawPseudorangeMeters'] + gnss['SvClockBiasMeters'] - gnss['IsrbMeters'] -
        gnss['IonosphericDelayMeters'] - gnss['TroposphericDelayMeters']).to_numpy()
prr = (gnss['PseudorangeRateMetersPerSecond'] +
        gnss['SvClockDriftMetersPerSecond']).to_numpy()

# Satellite position/velocity
xsat_pr = gnss[['SvPositionXEcefMeters', 'SvPositionYEcefMeters',
                    'SvPositionZEcefMeters']].to_numpy()
xsat_prr = gnss[['SvPositionXEcefMeters', 'SvPositionYEcefMeters',
                    'SvPositionZEcefMeters']].to_numpy()
vsat = gnss[['SvVelocityXEcefMetersPerSecond', 'SvVelocityYEcefMetersPerSecond',
                'SvVelocityZEcefMetersPerSecond']].to_numpy()

# Weight matrix for peseudorange/pseudorange rate
Wx = np.diag(1 / gnss['RawPseudorangeUncertaintyMeters'].to_numpy())
Wv = np.diag(1 / gnss['PseudorangeRateUncertaintyMetersPerSecond'].to_numpy())

# Robust WLS requires accurate initial values for convergence,
# so perform normal WLS for the first time
if len(gnss) >= 4:
    # Normal WLS
    if np.all(x0 == 0):
        opt = scipy.optimize.least_squares(
            f_wls, x0, jac_pr_residuals, args=(xsat_pr, pr, Wx))
        x0 = opt.x 
    # Robust WLS for position estimation
    opt = scipy.optimize.least_squares(
            f_wls, x0,  jac_pr_residuals, args=(xsat_pr, pr, Wx), loss='soft_l1')
    if opt.status < 1 or opt.status == 2:
        print(f'position lsq status = {opt.status}')
    else:
        # Covariance estimation
        cov = np.linalg.inv(opt.jac.T @ Wx @ opt.jac)
        cov_x[:, :] = cov[:3, :3]
        x_wls[:] = opt.x[:3]
        x0 = opt.x

# Velocity estimation
if len(gnss) >= 4:
    if np.all(v0 == 0): # Normal WLS
        opt = scipy.optimize.least_squares(
            prr_residuals, v0, jac_prr_residuals, args=(vsat, prr, x0, xsat_prr, Wv))
        v0 = opt.x
    # Robust WLS for velocity estimation
    opt = scipy.optimize.least_squares(
        prr_residuals, v0, jac_prr_residuals, args=(vsat, prr, x0, xsat_prr, Wv), loss='soft_l1')
    if opt.status < 1:
        print(f'velocity lsq status = {opt.status}')
    else:
        # Covariance estimation
        cov = np.linalg.inv(opt.jac.T @ Wv @ opt.jac)
        cov_v[:, :] = cov[:3, :3]
        v_wls[:] = opt.x[:3]
        v0 = opt.x

llh_wls = np.array(pm.ecef2geodetic(x_wls[0], x_wls[1], x_wls[2])).T
# gt = gt[gt['UnixTimeMillis'] == 1593045252440]
# print(gt)
# llh_gt = gt[['LatitudeDegrees', 'LongitudeDegrees']].to_numpy()[0]
print(llh_wls)


