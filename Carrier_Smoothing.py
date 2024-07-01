import numpy as np
import pandas as pd

class CarrierSmoothing:
    def __init__(self, gnss_df):
        self.gnss_df = gnss_df
        self.carr_th = 1.5  # carrier phase jump threshold [m]
        self.pr_th = 20.0  # pseudorange jump threshold [m]
        
    def smooth(self):
        prsmooth = np.full_like(self.gnss_df['RawPseudorangeMeters'], np.nan)
        for (i, (svid_sigtype, df)) in enumerate(self.gnss_df.groupby(['Svid', 'SignalType'])):
            df = df.replace({'AccumulatedDeltaRangeMeters': {0: np.nan}})  # 0 to NaN

            # Compare time difference between pseudorange/carrier with Doppler
            drng1 = df['AccumulatedDeltaRangeMeters'].diff() - df['PseudorangeRateMetersPerSecond']
            drng2 = df['RawPseudorangeMeters'].diff() - df['PseudorangeRateMetersPerSecond']

            # Check cycle-slip
            slip1 = (df['AccumulatedDeltaRangeState'].to_numpy() & 2**1) != 0  # reset flag
            slip2 = (df['AccumulatedDeltaRangeState'].to_numpy() & 2**2) != 0  # cycle-slip flag
            slip3 = np.fabs(drng1.to_numpy()) > self.carr_th  # Carrier phase jump
            slip4 = np.fabs(drng2.to_numpy()) > self.pr_th  # Pseudorange jump

            idx_slip = slip1 | slip2 | slip3 | slip4
            idx_slip[0] = True

            # groups with continuous carrier phase tracking
            df['group_slip'] = np.cumsum(idx_slip)

            # Psudorange - carrier phase
            df['dpc'] = df['RawPseudorangeMeters'] - df['AccumulatedDeltaRangeMeters']

            # Absolute distance bias of carrier phase
            meandpc = df.groupby('group_slip')['dpc'].mean()
            df = df.merge(meandpc, on='group_slip', suffixes=('', '_Mean'))

            # Index of original gnss_df
            idx = (self.gnss_df['Svid'] == svid_sigtype[0]) & (self.gnss_df['SignalType'] == svid_sigtype[1])

            # Carrier phase + bias
            prsmooth[idx] = df['AccumulatedDeltaRangeMeters'] + df['dpc_Mean']

        # If carrier smoothing is not possible, use original pseudorange
        idx_nan = np.isnan(prsmooth)
        prsmooth[idx_nan] = self.gnss_df['RawPseudorangeMeters'][idx_nan]
        self.gnss_df['pr_smooth'] = prsmooth

        return self.gnss_df