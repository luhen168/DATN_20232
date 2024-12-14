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


folder_path = os.path.abspath('gnss_analysis')
sys.path.append(folder_path)
from gnssutils import EphemerisManager,WLS,Troposphere

file_path = 'C:/Users/ductm/Desktop/Code_Python/GSDC/2020-06-25-00-34-us-ca-mtv-sb-101'
gnss_df_all = pd.read_csv('%s/device_gnss.csv' % file_path, dtype={'SignalType': str})


# Add standard Frequency column
frequency_median = gnss_df_all.groupby('SignalType')['CarrierFrequencyHz'].median()
gnss_df_all = gnss_df_all.merge(frequency_median, how='left', on='SignalType', suffixes=('', 'Ref'))
carrier_error = abs(gnss_df_all['CarrierFrequencyHz'] - gnss_df_all['CarrierFrequencyHzRef'])
gnss_df_all['CarrierErrorHz'] = carrier_error

gnss_epoch = gnss_df_all[gnss_df_all['utcTimeMillis'] == 1593045252440]
# Tạo instance của class WLS
wls = WLS()

# Sử dụng phương thức WLS_onePosition để tính toán vị trí
ecef = wls.WLS_onePosition(gnss_epoch)

llh_wls = np.array(pm.ecef2geodetic(ecef[0], ecef[1], ecef[2])).T
# print(gnss_all['SvPositionXEcefMeters'][0],gnss_all['SvPositionYEcefMeters'][0],gnss_all['SvPositionZEcefMeters'][0])

print(llh_wls)

def my_ecef2lla(pos):
  # x, y and z are scalars or vectors in meters
  x = pos[0]
  y = pos[1]
  z = pos[2]
  a=6378137
  a_sq=a**2
  e = 8.181919084261345e-2
  e_sq = 6.69437999014e-3

  f = 1/298.257223563
  b = a*(1-f)

  # calculations:
  r = np.sqrt(x**2 + y**2)
  ep_sq  = (a**2-b**2)/b**2
  ee = (a**2-b**2)
  f = (54*b**2)*(z**2)
  g = r**2 + (1 - e_sq)*(z**2) - e_sq*ee*2
  c = (e_sq**2)*f*r**2/(g**3)
  s = (1 + c + np.sqrt(c**2 + 2*c))**(1/3.)
  p = f/(3.*(g**2)*(s + (1./s) + 1)**2)
  q = np.sqrt(1 + 2*p*e_sq**2)
  r_0 = -(p*e_sq*r)/(1+q) + np.sqrt(0.5*(a**2)*(1+(1./q)) - p*(z**2)*(1-e_sq)/(q*(1+q)) - 0.5*p*(r**2))
  u = np.sqrt((r - e_sq*r_0)**2 + z**2)
  v = np.sqrt((r - e_sq*r_0)**2 + (1 - e_sq)*z**2)
  z_0 = (b**2)*z/(a*v)
  h = u*(1 - b**2/(a*v))
  phi = np.arctan((z + ep_sq*z_0)/r)
  lambd = np.arctan2(y, x)

  return phi*180/np.pi, lambd*180/np.pi, h

posi = gnss_df_all[['SvPositionXEcefMeters','SvPositionYEcefMeters','SvPositionZEcefMeters']]
res = my_ecef2lla(posi.iloc[0])

print(res)

R_EARTH = 6371.0

def geodetic_to_ecef(lat, lon, alt):
    lat = np.radians(lat)
    lon = np.radians(lon)

    x = (R_EARTH + alt) * np.cos(lat) * np.cos(lon)
    y = (R_EARTH + alt) * np.cos(lat) * np.sin(lon)
    z = (R_EARTH + alt) * np.sin(lat)

    return np.array([x, y, z])

def relative_position(observer_ecef, satellite_ecef):
    return satellite_ecef - observer_ecef

def ecef_to_topocentric(observer_lat, observer_lon, rel_pos):
    observer_lat = np.radians(observer_lat)
    observer_lon = np.radians(observer_lon)

    # Transformation matrix
    T = np.array([
        [-np.sin(observer_lon),np.cos(observer_lon),0],
        [-np.sin(observer_lat) * np.cos(observer_lon), -np.sin(observer_lat) * np.sin(observer_lon), np.cos(observer_lat)],
        [ np.cos(observer_lat) * np.cos(observer_lon),  np.cos(observer_lat) * np.sin(observer_lon), np.sin(observer_lat)]
    ])

    topocentric = T @ rel_pos
    return topocentric


def calculate_elevation_azimuth(observer_lat, observer_lon, observer_alt, satellite_lat, satellite_lon, satellite_alt):
    # Convert geodetic to ECEF coordinates
    observer_ecef = geodetic_to_ecef(observer_lat, observer_lon, observer_alt)
    satellite_ecef = geodetic_to_ecef(satellite_lat, satellite_lon, satellite_alt)

    # Calculate relative position vector
    rel_pos = relative_position(observer_ecef, satellite_ecef)

    # Convert to topocentric coordinates
    topocentric = ecef_to_topocentric(observer_lat, observer_lon, rel_pos)

    # Calculate elevation and azimuth
    east, north, up = topocentric
    horizontal_distance = np.sqrt(east**2 + north**2)

    elevation = np.degrees(np.arctan2(up, horizontal_distance))
    azimuth = np.degrees(np.arctan2(east, north))
    if azimuth < 0:
        azimuth += 360

    return elevation, azimuth


observer_lat = llh_wls[0]  # Latitude in degrees
observer_lon = llh_wls[1]  # Longitude in degrees
observer_alt = llh_wls[2] / 1000  # Altitude in kilometers

# Satellite's location
satellite_lat = res[0]  # Latitude in degrees
satellite_lon = res[1]  # Longitude in degrees
satellite_alt = res[2] / 1000  # Altitude in kilometers (geostationary orbit)

elevation, azimuth = calculate_elevation_azimuth(observer_lat, observer_lon, observer_alt, satellite_lat, satellite_lon, satellite_alt)
print(f"Elevation: {elevation} degrees")
print(f"Azimuth: {azimuth} degrees")