from pathlib import Path
from time import time
import numpy as np
import pandas as pd
from libWLS import WLS
from Outlier_Detection_final01 import Outlier

def main():
    # Load your GNSS data into a DataFrame
    current_directory = Path.cwd()
    path = 'moduleWLS/2020-06-25-00-34-us-ca-mtv-sb-101/pixel4xl'
    file_path = current_directory / path
    gnss_df_all = pd.read_csv('%s/device_gnss.csv' % file_path, dtype={'SignalType': str})

    
    # Add standard Frequency column
    frequency_median = gnss_df_all.groupby('SignalType')['CarrierFrequencyHz'].median()
    gnss_df_all = gnss_df_all.merge(frequency_median, how='left', on='SignalType', suffixes=('', 'Ref'))
    carrier_error = abs(gnss_df_all['CarrierFrequencyHz'] - gnss_df_all['CarrierFrequencyHzRef'])
    gnss_df_all['CarrierErrorHz'] = carrier_error

    gnss_epoch = gnss_df_all[gnss_df_all['utcTimeMillis'] == 1593045251447]
    # Tạo instance của class WLS
    wls = WLS()

    # Sử dụng phương thức WLS_onePosition để tính toán vị trí
    estimated_pos = wls.WLS_onePosition_origin(gnss_epoch)
    
    # # In ra kết quả vị trí đã tính toán
    # print('here')
    print(f"Estimated Position: {estimated_pos}")

if __name__ == "__main__":
    main()
