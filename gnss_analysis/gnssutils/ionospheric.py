import numpy as np
import math
# Constants for the Klobuchar model


def geocentric_to_geodetic(geocentric_latitude):
    # Constants for WGS84
    a = 6378137.0  # Semi-major axis in meters
    b = 6356752.3142  # Semi-minor axis in meters
    e2 = 1 - (b ** 2 / a ** 2)  # Square of eccentricity

    geocentric_latitude_rad = math.radians(geocentric_latitude)
    
    # Use iterative approach to convert geocentric latitude to geodetic latitude
    geodetic_latitude_rad = geocentric_latitude_rad
    for _ in range(5):  # Iterate to converge to the correct value
        geodetic_latitude_rad = math.atan(
            (1 - e2) * math.tan(geocentric_latitude_rad)
        )
    
    return math.degrees(geodetic_latitude_rad)


class Ionosphere():

    def klobuchar_ionospheric_delay(self,_GPS_time, latitude, longitude, azimuth, elevation,alpha,beta):
        """
        Estimate the ionospheric delay in meters using the Klobuchar model.
        
        Parameters:
            GPS_time (float): GPS time in seconds.
            latitude (float): User's geodetic latitude in radians.
            longitude (float): User's geodetic longitude in radians.
            azimuth (float): Satellite azimuth angle in radians.
            elevation (float): Satellite elevation angle in radians.
            
        Returns:
            float: Estimated ionospheric delay in meters.
        """
        # Convert latitude and longitude to semi-circle units
        GPS_time = _GPS_time * 1e-3
        latitude = geocentric_to_geodetic(latitude)
        longitude = np.deg2rad(longitude)
        azimuth = np.deg2rad(azimuth)
        elevation = np.deg2rad(elevation)
        phi_u = latitude / np.pi #latitude semicycle
        lambda_u = longitude / np.pi #longitude semicycle
        
        # Earth's central angle between the user position and the earth projection of ionospheric intersection point
        psi = 0.0137 / (elevation / np.pi + 0.11) - 0.022
        
        # Subionospheric latitude
        phi_i = phi_u + psi * np.cos(azimuth)
        if phi_i > 0.416:
            phi_i = 0.416
        elif phi_i < -0.416:
            phi_i = -0.416
        
        # Subionospheric longitude
        lambda_i = lambda_u + (psi * np.sin(azimuth)) / np.cos(latitude)
        
        # Geomagnetic latitude
        phi_m = phi_i + 0.064 * np.cos((lambda_i - 1.617) * np.pi)
        
        # Local time
        t = 43200 * lambda_i + GPS_time
        t = t % 86400  # Ensure time is within a day
        if t < 0:
            t += 86400
        
        # Obliquity factor
        F = 1.0 + 16.0 * ((0.53 - elevation / np.pi) ** 3)
        
        # Amplitude of ionospheric delay
        A = alpha[0] + alpha[1] * phi_m + alpha[2] * phi_m**2 + alpha[3] * phi_m**3
        if A < 0:
            A = 0
        
        # Period of ionospheric delay
        P = beta[0] + beta[1] * phi_m + beta[2] * phi_m**2 + beta[3] * phi_m**3
        if P < 72000:
            P = 72000
        
        # Phase of ionospheric delay
        X = 2.0 * np.pi * (t - 50400) / P
        
        # Ionospheric time delay
        if abs(X) < 1.57:
            T_iono = F * (5e-9 + A * (1 - X**2 / 2 + X**4 / 24))
        else:
            T_iono = F * 5e-9
        
        # Convert time delay to meters
        ionospheric_delay = T_iono * 3e8  # Speed of light in meters per second
        
        return ionospheric_delay

# Example usage

# alpha = [0.4657e-08,  0.1490e-07, -0.5960e-07, -0.1192e-06]
# beta  = [0.8192e+05,  0.9830e+05, -0.6554e+05, -0.5243e+06]
# GPS_time = [1593045252440]  # GPS time in seconds
# latitude = (37.4280397000)  # User's latitude in degrees
# longitude = (-122.0699122000)  # User's longitude in degrees
# azimuth = (242.995566527484)  # Satellite azimuth angle in degrees
# elevation = (48.5623965942834)  # Satellite elevation angle in degrees

# delay = Ionosphere().klobuchar_ionospheric_delay(GPS_time, latitude, longitude, azimuth, elevation,alpha,beta)
# print(f"Ionospheric Delay: {delay:.10f} meters")
