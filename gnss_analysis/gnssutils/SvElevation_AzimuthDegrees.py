import numpy as np

# Observer's geodetic coordinates
# latitude = np.deg2rad(37.42842435538492)  # Latitude in radians
# longitude = np.deg2rad(-122.07260706994416)  # Longitude in radians
# altitude = -11.857622861477367  # Altitude in meters
# vehicle_ecef = [13603249.8762636,-8033462.84966784, 21217893.3183791]

class ElevationAzimuthAngle:
    def geodetic_to_ecef(self,lat, lon, alt):
        a = 6378137.0  # Earth's semi-major axis in meters
        f = 1 / 298.257223563  # Earth's flattening
        e2 = 2*f - f**2  # Square of eccentricity
        
        N = a / np.sqrt(1 - e2 * np.sin(lat)**2)  # Prime vertical radius of curvature
        
        X = (N + alt) * np.cos(lat) * np.cos(lon)
        Y = (N + alt) * np.cos(lat) * np.sin(lon)
        Z = (N * (1 - e2) + alt) * np.sin(lat)
        
        return X, Y, Z

    def ecef_to_enu(self,observer_ecef, vehicle_ecef, lat, lon):
        # Translation vector from observer to vehicle
        dX = vehicle_ecef['SvPositionXEcefMeters'] - observer_ecef[0]
        dY = vehicle_ecef['SvPositionYEcefMeters'] - observer_ecef[1]
        dZ = vehicle_ecef['SvPositionZEcefMeters'] - observer_ecef[2]
        
        # Rotation matrix to convert ECEF to ENU coordinates
        R = np.array([
            [-np.sin(lon), np.cos(lon), 0],
            [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
            [np.cos(lat)*np.cos(lon), np.cos(lat)*np.sin(lon), np.sin(lat)]
        ])
        
        # Topocentric-horizon coordinates (ENU)
        enu = np.dot(R, np.array([dX, dY, dZ]))
        
        return enu

    def calculate_elevation(self,enu):
        # enu[2] is the Up component in ENU coordinates
        horizontal_distance = np.sqrt(enu[0]**2 + enu[1]**2)
        elevation = np.arctan2(enu[2], horizontal_distance)
        
        return np.rad2deg(elevation)  # Return elevation in degrees

    def calculate_azimuth(self,enu):
        # enu[0] is the East component in ENU coordinates
        # enu[1] is the North component in ENU coordinates
        azimuth = np.arctan2(enu[0], enu[1])  # Azimuth angle in radians
        
        # Convert from radians to degrees
        azimuth = np.rad2deg(azimuth)
        
        # Normalize to 0-360 degrees
        if azimuth < 0:
            azimuth += 360
        
        return azimuth
    
    def CalculateAngle(self,SvPosition_ecef,UserPosition_lla):
        latitude = np.deg2rad(UserPosition_lla[0])
        longitude = np.deg2rad(UserPosition_lla[1])
        altitude = UserPosition_lla[2]
        observer_ecef = self.geodetic_to_ecef(latitude, longitude, altitude)
        enu = self.ecef_to_enu(observer_ecef, SvPosition_ecef, latitude, longitude)
        azimuth = self.calculate_azimuth(enu)
        elevation = self.calculate_elevation(enu)
        
        return elevation,azimuth



