import numpy as np  
import math
import pandas as pd 
import matplotlib.pyplot as plt
import os, csv, sys
import glob
import subprocess
from tqdm.notebook import tqdm
from dataclasses import dataclass
from scipy.interpolate import InterpolatedUnivariateSpline
from datetime import datetime, timezone, timedelta
from sklearn.metrics import mean_squared_error 
from decimal import Decimal,getcontext
from calculate_score import calc_score, vincenty_distance

import math

def haversine(lat1, lon1, lat2, lon2):
    # Bán kính trái đất
    R = 6371.0

    # Chuyển đổi độ sang radian
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    # Công thức Haversine
    a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Khoảng cách
    distance = R * c

    return distance


sat = 'C:/Users/ductm/Downloads/position_data_2024_06_08_13_33_sat.csv'
truth = 'C:/Users/ductm/Downloads/ground_truth_2024_06_08_13_33.csv'
gnss_sat = pd.read_csv(sat, index_col=False)
gnss_truth = pd.read_csv(truth,index_col=False)

time = gnss_sat['utcTimeMillis'].isin(gnss_truth['utcTimeMillis'])

measurement = gnss_sat[time]
gr_truth = gnss_truth[gnss_truth['utcTimeMillis'].isin(gnss_sat['utcTimeMillis'])]

score_bl = []

measurement = measurement[['Latitude','Longitude']]
gr_truth = gr_truth[['Latitude','Longitude']]
measurement = measurement.reset_index(drop=True)
gr_truth = gr_truth.reset_index(drop=True)
# score_bl.append(vincenty_distance(measurement, gr_truth))
# score_all_bl = np.mean([np.quantile(score_bl, 0.50), np.quantile(score_bl, 0.95)])
# print("Baseline score: ", score_all_bl)


# RMSE_lat = np.square(np.subtract(measurement['Latitude'] , gr_truth['Latitude'])).mean() 
# print("Lat: ",RMSE_lat)
# RMSE_long = np.square(np.subtract(measurement['Longitude'] , gr_truth['Longitude'])).mean() 
# print("Long: ",RMSE_long)
for i in range(len(measurement)):
    lat1, lon1 = measurement.loc[i, 'Latitude'], measurement.loc[i, 'Longitude']
    lat2, lon2 = gr_truth.loc[i, 'Latitude'], gr_truth.loc[i, 'Longitude']
    distance = haversine(lat1, lon1, lat2, lon2)
    score_bl.append(distance)
RMSE = np.square(score_bl).mean()
print(RMSE)
