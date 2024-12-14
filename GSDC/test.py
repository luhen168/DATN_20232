import math
import numpy as np

# Constants for the Klobuchar model
alpha = [0.4657E-08,  0.1490E-07, -0.5960E-07, -0.1192E-06 ]
beta = [0.8192E+05,  0.9830E+05, -0.6554E+05, -0.5243E+06 ]

def klobuchar_ionospheric_delay(GPS_time, latitude, longitude, azimuth, elevation):
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
    phi_u = latitude / math.pi
    lambda_u = longitude / math.pi
    
    # Earth's central angle between the user position and the earth projection of ionospheric intersection point
    psi = 0.0137 / (elevation / math.pi + 0.11) - 0.022
    
    # Subionospheric latitude
    phi_i = phi_u + psi * math.cos(azimuth)
    if phi_i > 0.416:
        phi_i = 0.416
    elif phi_i < -0.416:
        phi_i = -0.416
    
    # Subionospheric longitude
    lambda_i = lambda_u + (psi * math.sin(azimuth)) / math.cos(phi_i * math.pi)
    
    # Geomagnetic latitude
    phi_m = phi_i + 0.064 * math.cos((lambda_i - 1.617) * math.pi)
    
    # Local time
    t = 43200 * lambda_i + GPS_time
    t = t % 86400  # Ensure time is within a day
    if t < 0:
        t += 86400
    
    # Obliquity factor
    F = 1.0 + 16.0 * ((0.53 - elevation / math.pi) ** 3)
    
    # Amplitude of ionospheric delay
    A = alpha[0] + alpha[1] * phi_m + alpha[2] * phi_m**2 + alpha[3] * phi_m**3
    if A < 0:
        A = 0
    
    # Period of ionospheric delay
    P = beta[0] + beta[1] * phi_m + beta[2] * phi_m**2 + beta[3] * phi_m**3
    if P < 72000:
        P = 72000
    
    # Phase of ionospheric delay
    X = 2.0 * math.pi * (t - 50400) / P
    
    # Ionospheric time delay
    if abs(X) < 1.57:
        T_iono = F * (5e-9 + A * (1 - X**2 / 2 + X**4 / 24))
    else:
        T_iono = F * 5e-9
    
    # Convert time delay to meters
    ionospheric_delay = T_iono * 3e8  # Speed of light in meters per second
    
    return ionospheric_delay

# Example usage
GPS_time = 1593045252440*1e-3  # GPS time in seconds
latitude = math.radians(37.4280397000)  # User's latitude in degrees
longitude = math.radians(-122.0699122000)  # User's longitude in degrees
azimuth = math.radians(42.6860842805267)  # Satellite azimuth angle in degrees
elevation = math.radians(14.8933096135287)  # Satellite elevation angle in degrees

delay = klobuchar_ionospheric_delay(GPS_time, latitude, longitude, azimuth, elevation)
print(f"Ionospheric Delay: {delay:.2f} meters")
