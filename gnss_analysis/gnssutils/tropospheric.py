#tropospheric by egnos model

import pandas as pd
import matplotlib.pyplot as plt
import math

# define const 
# parameter of EGNOS models
INDEX_15_DEGREES = 0
INDEX_75_DEGREES = 4
LATITUDE_15_DEGREES = 15
LATITUDE_75_DEGREES = 75

#troposphere averange pressure mbar
latDegreeToPressureMbarAvgMap = [1013.25,1017.25,1015.75,1011.75,1013.0]
#troposphere averange temperature Kelvin
latDegreeToTempKelvinAvgMap = [299.65,294.15,283.15,272.15,263.65]
#troposphere averange water vapor pressure
latDegreeToWVPressureMbarAvgMap = [26.31,21.79,11.66,6.78,4.11]
#troposphere averange temperature lapse rate K/m
latDegreeToBetaAvgMapKPM = [6.3e-3,6.05e-3,5.58e-3,5.39e-3,4.53e-3]
#troposphere averange water vapor lapse rate (dimentionless)
latDegreeToLambdaAvgMap = [2.77,3.15,2.57,1.81,1.55]


#trophosphere amplitude pressure mbar
latDegreeToPressureMbarAmpMap = [0.0,-3.75,-2.25,-1.75,-0.5]
#troposphere amplitude temperature Kelvin
latDegreeToTempKelvinAmpMap = [0.0,7.0,11.0,15.0,14.5]
#tropesphere amplitude water vapor pressure
latDegreeToWVPressureMbarAmpMap = [0.0,8.85,7.24,5.36,3.39]
#troposphere amplitude temperature lapse rate K/m
latDegreeToBetaAmpMapKPM = [0.0,0.25e-3,0.32e-3,0.81e-3,0.62e-3]
#troposphere amplitudwater vapor lapse rate (dimentionless)
latDegreeToLambdaAmpMap = [0.0,0.33,0.46,0.74,0.3]

#zenith delay dry constant K/mbar
K1 = 77.604
#zenith delay wet constant K^2.mbar
K2 = 382000.0
#gas constant for dry air J/kg/K
RD = 287.054
#acceleration of gravity at the atmospheric colum centroid m/s^-2
GM = 9.784

#gravity m/s^2
GRAVITY_MPS2 = 9.80665

MINIMUN_INTERPOLATION_THRESHOLD = 1e-25
B_HYDROSTATIC = 0.0035716
C_HYDROSTATIC = 0.082456
B_NON_HYDROSTATIC = 0.0018576
C_NON_HYDROSTATIC = 0.062741

SOUTHERN_HEMISPHERE_DMIN = 211.0
NORTHERN_HEMISPHERE_DMIN = 28.0

DAY_PER_YEAR = 365.25


def utctimemilis_to_day_of_year(utctimemilis):
    # Chuyển đổi utctimemilis sang đối tượng datetime
    timestamp = pd.to_datetime(utctimemilis, unit='ms', utc=True)
    
    # Trích xuất số ngày trong năm từ đối tượng datetime
    day_of_year = timestamp.dayofyear
    
    return day_of_year

class Troposphere():
    class DryAndWetMappingValues:
        def __init__(self,dryMappingValues,wetMappingValues):
            self.dryMappingValue = dryMappingValues
            self.wetMappingValue = wetMappingValues
    class DryAndWetZenithDelays:
        def __init__(self,dryZenithDelays,wetZenithDelays):
            self.dryZenithDelaySec = dryZenithDelays
            self.wetZenithDelaySec = wetZenithDelays
        def __repr__(self):
            return f"DryAndWetZenithDelays(dry_delay={self.dry_delay}, wet_delay={self.wet_delay})"

    def interpolate(self,point1X,point1Y,point2X,point2Y,xOutput):
        if ((point1X < point2X and (xOutput < point1X or xOutput > point2X))
        or (point2X < point1X and (xOutput < point2X and xOutput > point1X))):
            raise ValueError("Interpolated value is outside the interpolated region")
        deltaX = point2X - point1X
        if abs(deltaX) > MINIMUN_INTERPOLATION_THRESHOLD:
            yOutput = point1Y + (xOutput - point1X) / deltaX * (point2Y - point1Y)
        else:
            yOutput = point1Y
        return yOutput            
    
    def calculateZenithDryAndWetDelaysSec(self,userLatitudeRadians,heightMetersAboveSeaLevel,dayOfYear1To366):
        absLatitudeDeg = math.degrees(abs(userLatitudeRadians))
        if userLatitudeRadians < 0 :
            dmin = SOUTHERN_HEMISPHERE_DMIN
        else:
            dmin = NORTHERN_HEMISPHERE_DMIN

        amplitudeScaleFactor = math.cos((2 * math.pi * (dayOfYear1To366 - dmin)) / DAY_PER_YEAR)
        
        if absLatitudeDeg <= LATITUDE_15_DEGREES :
            pressureMbar = latDegreeToPressureMbarAvgMap[INDEX_15_DEGREES] - latDegreeToPressureMbarAmpMap[INDEX_15_DEGREES] * amplitudeScaleFactor
            
            tempKelvin = latDegreeToTempKelvinAvgMap[INDEX_15_DEGREES] - latDegreeToTempKelvinAmpMap[INDEX_15_DEGREES] * amplitudeScaleFactor
            
            waterVaporPressureMbar = latDegreeToWVPressureMbarAvgMap[INDEX_15_DEGREES] - latDegreeToWVPressureMbarAmpMap[INDEX_15_DEGREES] * amplitudeScaleFactor
            
            beta = latDegreeToBetaAvgMapKPM[INDEX_15_DEGREES]  - latDegreeToBetaAmpMapKPM[INDEX_15_DEGREES] * amplitudeScaleFactor
            
            Lambda = latDegreeToLambdaAmpMap[INDEX_15_DEGREES] - latDegreeToLambdaAmpMap[INDEX_15_DEGREES] * amplitudeScaleFactor
            
        elif absLatitudeDeg > LATITUDE_15_DEGREES and absLatitudeDeg < LATITUDE_75_DEGREES:
            key = int(absLatitudeDeg / LATITUDE_15_DEGREES)
            
            #calculate pressure (Mbar)
            averagePressureMbar = self.interpolate(
                key*LATITUDE_15_DEGREES,
                latDegreeToPressureMbarAvgMap[key - 1],
                (key + 1) * LATITUDE_15_DEGREES,
                latDegreeToPressureMbarAvgMap[key],
                absLatitudeDeg)
            
            amplitudePressureMbar = self.interpolate(
                key * LATITUDE_15_DEGREES,
                latDegreeToPressureMbarAmpMap[key - 1],
                (key + 1)*LATITUDE_15_DEGREES,
                latDegreeToPressureMbarAmpMap[key],
                absLatitudeDeg)
            
            pressureMbar = averagePressureMbar - amplitudePressureMbar * amplitudeScaleFactor
            
            #calculate tempKelvin (K)
            averageTempKelvin = self.interpolate(
                key * LATITUDE_15_DEGREES,
                latDegreeToTempKelvinAvgMap[key - 1],
                (key + 1) * LATITUDE_15_DEGREES,
                latDegreeToTempKelvinAvgMap[key],
                absLatitudeDeg)
            
            amplitudeTempKelvin = self.interpolate(
                key * LATITUDE_15_DEGREES,
                latDegreeToTempKelvinAmpMap[key - 1],
                (key + 1) * LATITUDE_15_DEGREES,
                latDegreeToTempKelvinAmpMap[key],
                absLatitudeDeg)
            
            tempKelvin = averageTempKelvin - amplitudeTempKelvin * amplitudeScaleFactor
            
            #calculate vapor water pressure 
            averageWaterVaporPressureMbar = self.interpolate(
                key * LATITUDE_15_DEGREES,
                latDegreeToWVPressureMbarAvgMap[key - 1],
                (key + 1) * LATITUDE_15_DEGREES,
                latDegreeToWVPressureMbarAvgMap[key],
                absLatitudeDeg)
            
            amplitudeWaterVaporPressureMbar = self.interpolate(
                key * LATITUDE_15_DEGREES,
                latDegreeToWVPressureMbarAmpMap[key - 1],
                (key + 1) * LATITUDE_15_DEGREES,
                latDegreeToWVPressureMbarAmpMap[key],
                absLatitudeDeg)
            
            waterVaporPressureMbar = averageWaterVaporPressureMbar - amplitudeWaterVaporPressureMbar * amplitudeScaleFactor
            
            #calculate Beta
            averageBeta = self.interpolate(
                key * LATITUDE_15_DEGREES,
                latDegreeToBetaAvgMapKPM[key - 1],
                (key + 1) * LATITUDE_15_DEGREES,
                latDegreeToBetaAvgMapKPM[key],
                absLatitudeDeg)
            
            amplitudeBeta = self.interpolate(
                key * LATITUDE_15_DEGREES,
                latDegreeToBetaAmpMapKPM[key - 1],
                (key + 1) * LATITUDE_15_DEGREES,
                latDegreeToBetaAmpMapKPM[key],
                absLatitudeDeg)
            
            beta = averageBeta - amplitudeBeta * amplitudeScaleFactor

            #calculate lambda
            averageLambda = self.interpolate(
                key * LATITUDE_15_DEGREES,
                latDegreeToLambdaAvgMap[key - 1],
                (key + 1) * LATITUDE_15_DEGREES,
                latDegreeToLambdaAvgMap[key],
                absLatitudeDeg)
            
            amplitudeLambda = self.interpolate(
                key * LATITUDE_15_DEGREES,
                latDegreeToLambdaAmpMap[key - 1],
                (key + 1) * LATITUDE_15_DEGREES,
                latDegreeToLambdaAmpMap[key],
                absLatitudeDeg)
            Lambda = averageLambda - amplitudeLambda * amplitudeScaleFactor
            
        else:
            pressureMbar = latDegreeToPressureMbarAvgMap[INDEX_75_DEGREES] - latDegreeToPressureMbarAmpMap[INDEX_75_DEGREES] * amplitudeScaleFactor
            
            tempKelvin = latDegreeToTempKelvinAvgMap[INDEX_75_DEGREES] - latDegreeToTempKelvinAmpMap[INDEX_75_DEGREES] * amplitudeScaleFactor
            
            waterVaporPressureMbar = latDegreeToWVPressureMbarAvgMap[INDEX_75_DEGREES] - latDegreeToWVPressureMbarAmpMap[INDEX_75_DEGREES] * amplitudeScaleFactor
            
            beta = latDegreeToBetaAvgMapKPM[INDEX_75_DEGREES] - latDegreeToBetaAmpMapKPM[INDEX_75_DEGREES] * amplitudeScaleFactor

            Lambda = latDegreeToLambdaAmpMap[INDEX_75_DEGREES] - latDegreeToLambdaAmpMap[INDEX_75_DEGREES] * amplitudeScaleFactor

        zenithDryDelayAtSeaLeverSeconds = (1.0e-6 * K1 * RD * pressureMbar) / GM
        zenithWetDelayAtSeaLeverSeconds =  (((1.0e-6 * K2 * RD) / (GM * (Lambda + 1.0) - beta * RD))
            * (waterVaporPressureMbar / tempKelvin))
        
        commonBase = 1.0 - ((beta * heightMetersAboveSeaLevel) / tempKelvin)
        
        powerDry = (GRAVITY_MPS2 / (RD * beta))
        powerWet = (((Lambda + 1.0) * GRAVITY_MPS2) / (RD * beta)) - 1.0
        zenithDryDelaySeconds = zenithDryDelayAtSeaLeverSeconds * (commonBase**powerDry)
        zenithWetDelaySeconds = zenithWetDelayAtSeaLeverSeconds * (commonBase**powerWet)
        
        return self.DryAndWetZenithDelays(zenithDryDelaySeconds,zenithWetDelaySeconds)
        
        
    def computeDryAndWetMappingValuesUsingUNBabcMappingFunction(self,satElevationRadians,userLatitudeRadians,heighMeterAboveSeaLevel):
        if satElevationRadians > math.pi / 2:
            satElevationRadians = math.pi / 2
        elif satElevationRadians < 2.0 * math.pi / 180.0 :
            satElevationRadians = math.radians(2.0)
        
        aHydrostatic = (1.18972 - 0.026855 * heighMeterAboveSeaLevel / 1000.0 + 0.10664 * math.cos(userLatitudeRadians))/1000.0
        numeratorDry = 1.0 + (aHydrostatic / (1.0 + (B_HYDROSTATIC / (1.0 + C_HYDROSTATIC))))
        denominatorDry = math.sin(satElevationRadians) 
        + (aHydrostatic 
        / (math.sin(satElevationRadians) 
            + (B_HYDROSTATIC 
            / (math.sin(satElevationRadians) 
                + C_HYDROSTATIC))))
        
        drymap = numeratorDry / denominatorDry
        
        aNonHydrostatic = (0.61120
                        -0.035348 * heighMeterAboveSeaLevel / 1000.0
                        -0.01526 * math.cos(userLatitudeRadians)) / 1000.0
        numeratorWet = 1.0 + (aNonHydrostatic / (1.0 + (B_HYDROSTATIC / (1.0 + C_HYDROSTATIC))))
        
        denominatorWet = math.sin(satElevationRadians)
        + (aNonHydrostatic
        / (math.sin(satElevationRadians)
            + (B_HYDROSTATIC / (math.sin(satElevationRadians)+ C_HYDROSTATIC))))
        
        wetmap = numeratorWet / denominatorWet
        
        return self.DryAndWetMappingValues(drymap,wetmap)
    
    def calculateTropoCorrectionMeters(self,satElevationRadians,userLatitudeRadian,heighMeterAboveSeaLevel,utcTimeMilies):
        dryAndWetMappingValues = self.computeDryAndWetMappingValuesUsingUNBabcMappingFunction(satElevationRadians=satElevationRadians,userLatitudeRadians=userLatitudeRadian,heighMeterAboveSeaLevel=heighMeterAboveSeaLevel)
        
        dayOfYear_1To366 = utctimemilis_to_day_of_year(utcTimeMilies)
        
        dryAndWetZenithDelays = self.calculateZenithDryAndWetDelaysSec(userLatitudeRadians=userLatitudeRadian,heightMetersAboveSeaLevel=heighMeterAboveSeaLevel,dayOfYear1To366=dayOfYear_1To366)
        
        drydelaySeconds = dryAndWetZenithDelays.dryZenithDelaySec * dryAndWetMappingValues.dryMappingValue
        wetdelaySeconds = dryAndWetZenithDelays.wetZenithDelaySec * dryAndWetMappingValues.wetMappingValue
        
        return drydelaySeconds + wetdelaySeconds



truth = pd.read_csv('C:/Users/ductm/Desktop/Code_Python/GSDC/2020-06-25-00-34-us-ca-mtv-sb-101/ground_truth.csv')
gnss = pd.read_csv('C:/Users/ductm/Desktop/Code_Python/GSDC/2020-06-25-00-34-us-ca-mtv-sb-101/device_gnss.csv')

# Chọn các cột từ 'truth' và 'gnss'
selected_cols_truth = ['LatitudeDegrees', 'LongitudeDegrees', 'AltitudeMeters']
selected_cols_gnss = ['SvElevationDegrees','utcTimeMillis']

# Gộp các DataFrame vào DataFrame mới
measurement = truth[selected_cols_truth].merge(gnss[selected_cols_gnss], left_index=True, right_index=True)



for index in range(30):
    satEle = math.radians(measurement['SvElevationDegrees'][index])
    userLat = math.radians(measurement['LatitudeDegrees'][index])
    heigh = measurement['AltitudeMeters'][index]
    print(Troposphere().calculateTropoCorrectionMeters(satElevationRadians=satEle,userLatitudeRadian=userLat,heighMeterAboveSeaLevel=heigh,utcTimeMilies=measurement['utcTimeMillis'][index]))
