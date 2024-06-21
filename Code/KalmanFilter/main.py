import pandas as pd
import numpy as np
from EKF_IMU import ESKF
from quaternion import Quaternion
from utilities import deg_to_rad, g_to_ms2

def read_data(imu_file, gnss_file):
    imu_data = pd.read_csv(imu_file)
    gnss_data = pd.read_csv(gnss_file)
    return imu_data, gnss_data

def process_data(imu_data, gnss_data):
    eskf = ESKF()
    
    # Initialize state with the first IMU data point
    initial_acc = imu_data.iloc[0][['MeasurementX', 'MeasurementY', 'MeasurementZ']]
    eskf.init_with_acc(initial_acc[0], initial_acc[1], initial_acc[2])
    
    # Variables to store results
    states = []
    timestamps = []

    for index, row in imu_data.iterrows():
        dt = 0.01  # Assume a constant time step of 10ms, modify if needed
        eskf.predict(dt)
        
        if row['MessageType'] == 'UncalGyro':
            eskf.correct_gyr(row['MeasurementX'], row['MeasurementY'], row['MeasurementZ'])
        elif row['MessageType'] == 'UncalAccel':
            eskf.correct_acc(row['MeasurementX'], row['MeasurementY'], row['MeasurementZ'])
        elif row['MessageType'] == 'UncalMag':
            # Use appropriate values for incl, B, W, V
            eskf.correct_mag(row['MeasurementX'], row['MeasurementY'], row['MeasurementZ'], incl=0, B=1, W=np.eye(3), V=np.zeros(3))

        states.append(eskf.getState())
        timestamps.append(row['utcTimeMillis'])

    return states, timestamps

def main():
    # imu_file = 'D:\DATN_20232\Code\KalmanFilter\imu_gnss_averaged_data.csv'
    imu_file = 'D:\DATN_20232\Code\GNSS-Positioning\data\2020-06-25-00-34-us-ca-mtv-sb-101\pixel4xl\device_imu.csv'
    gnss_file = 'D:\DATN_20232\Code\GNSS-Positioning\data\2020-06-25-00-34-us-ca-mtv-sb-101\pixel4xl\device_gnss.csv'
    
    imu_data, gnss_data = read_data(imu_file, gnss_file)
    
    states, timestamps = process_data(imu_data, gnss_data)
    
    # Save the results
    results = pd.DataFrame(states, columns=['x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'ex', 'ey', 'ez', 'wx', 'wy', 'wz'])
    results['utcTimeMillis'] = timestamps
    results.to_csv('eskf_results.csv', index=False)
    
    print(results.head())

if __name__ == "__main__":
    main()
