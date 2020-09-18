''' These classes model the change of air properties (Pressure, Density, etc... with altitude) '''

import abc
from bisect import bisect
from math import exp
from typing import Sequence

import numpy as np

from MAPLEAF.IO import defaultConfigValues, getAbsoluteFilePath
from MAPLEAF.Motion import linInterp

__all__ = [ "atmosphericModelFactory", "ConstantAtmosphere", "TabulatedAtmosphere", "USStandardAtmosphere" ]

class AtmosphericModel(abc.ABC):
    ''' Interface for all atmosphere models '''
    
    @abc.abstractmethod
    def getAirProperties(self, ASLElevation: float, time: float) -> Sequence[float]:
        ''' 
            Should return an iterable containing: 
                temp(K), 
                static pressure (Pa), 
                density (kg/m^3), 
                dynamic viscosity (Pa*s),
            in that order
        '''
        return

def atmosphericModelFactory(atmosphericModel=None, envDictReader=None) -> AtmosphericModel:
    ''' 
        Provide either an atmosphericModel name ('USStandardAtmosphere' is only option right now that doesn't require additional info,
            or provide an envDictReader (`MAPLEAF.IO.SubDictReader`)
    '''
    if atmosphericModel == None:
        atmosphericModel = envDictReader.getString("AtmosphericPropertiesModel")

    if atmosphericModel == "Constant":
        if envDictReader == None:
            raise ValueError("envDictReader required to initialize Constant atm properties model")

        # Get values from ConstantAtmosphere subDictionary
        envDictReader.simDefDictPathToReadFrom = "Environment.ConstantAtmosphere"

        constTemp = envDictReader.getFloat("temp") + 273.15 # Convert to Kelvin (Expecting Celsius input)
        constPressure = envDictReader.getFloat("pressure")
        constDensity = envDictReader.getFloat("density")
        constViscosity = envDictReader.getFloat("viscosity")

        # Return to reading from Environment for any subsequent parsing
        envDictReader.simDefDictPathToReadFrom = "Environment"

        return ConstantAtmosphere(constTemp, constPressure, constDensity, constViscosity)
    
    elif atmosphericModel == "TabulatedAtmosphere":
        try:
            tableFilePath = envDictReader.getString("TabulatedAtmosphere.filePath")
        except AttributeError:
            tableFilePath = defaultConfigValues["Environment.TabulatedAtmosphere.filePath"]

        tableFilePath = getAbsoluteFilePath(tableFilePath)
        return TabulatedAtmosphere(tableFilePath)

    elif atmosphericModel == "USStandardAtmosphere":
        return USStandardAtmosphere()

    else:
        raise ValueError("Atmospheric model: {} not implemented, try using 'USStandardAtmosphere'".format(atmosphericModel))
        
class ConstantAtmosphere(AtmosphericModel):
    def __init__(self, temp, pressure, density, viscosity):
        self.airProperties = [ temp, pressure, density, viscosity ]

    def getAirProperties(self, _, _2=None):
        return self.airProperties

class TabulatedAtmosphere(AtmosphericModel):
    '''
        Provides linearly-interpolated atmospheric properties from a table in a file
        Table columns are expected are:
            h(m ASL)	T(K)	P(Pa)	rho(kg/m^3)	    mu(10^-5 Pa*s)
    '''
    def __init__(self, filePath):
        inputTable = np.loadtxt(filePath, skiprows=1)

        # Convert viscosity to Pa-s
        inputTable[:, 4] *= 1E-5 
        
        # Set up keys, values for interpolation of all properties at once
        self.keys = inputTable[:, 0] # Altitude (m ASL) is interpolation key
        self.values = inputTable[:, 1:] # Contains: T(K), P(Pa), rho(kg/m^3), dynamicViscosity(Pa*s), all to be interpolated over

    def getAirProperties(self, ASLElevation, _=None):
        return linInterp(self.keys, self.values, ASLElevation)

class USStandardAtmosphere(AtmosphericModel):
    '''
        Provides atmospheric properties calculated according to the model here: 
        https://nebula.wsimg.com/ab321c1edd4fa69eaa94b5e8e769b113?AccessKeyId=AF1D67CEBF3A194F66A3&disposition=0&alloworigin=1

    '''
    T0 = 288.15 # K (15 Celsius) @ ASL == 0
    p0 = 101325 # Pa @ ASL == 0
    M = 28.9644 # Molecular weight of air
    R = 8.31432 # J/molK
    earthRadius = 6356766 # m - used to convert to geopotential altitude
    G = 9.80665 # m/s^2 (sea level, 45 degree latitude)

    baseHeights = [ -2000, 11000, 20000, 32000, 47000, 51000, 71000, 84852 ] # m (Geopotential)
    dt_dh = [ -6.5e-3, 0, 1.0e-3, 2.8e-3, 0, -2.8e-3, -2.0e-3, 0 ] # K/m
    baseTemps = [] # Gets filled out in self.__init__
    basePressures = [] # Gets filled out in self.__init__

    def __init__(self):
        # Pre-Calculate base temperatures for each interval
        dt_dh = self.dt_dh[0]
        baseTemp1 = self.T0 + dt_dh*self.baseHeights[0]
        self.baseTemps.append(baseTemp1)

        Pb_over_P0 = ( (self.T0 + dt_dh*self.baseHeights[0]) / \
            self.T0) ** (-self.G*self.M/(self.R * dt_dh * 1000))
        self.basePressures.append(Pb_over_P0 * self.p0)

        # Loop over each layer, calculate temp & pressure at base
        for i in range(1, len(self.baseHeights)):
            # Calculate temperatures at base of each layer
            Tb, Pb, dt_dh = self.baseTemps[-1], self.basePressures[i-1], self.dt_dh[i-1]
            dh = float(self.baseHeights[i] - self.baseHeights[i-1])
            
            nextTemp = Tb + dt_dh*dh

            # Calculate pressures at the base of each layer
            if dt_dh == 0.0:
                P_over_Pb = exp(-self.G*self.M*dh / (self.R * Tb * 1000))
                nextPressure = P_over_Pb * Pb

            else:
                body = (Tb + dt_dh*dh) / Tb
                exponent = -self.G*self.M / (self.R*dt_dh*1000)
                nextPressure = body**exponent * Pb

            self.baseTemps.append(nextTemp)
            self.basePressures.append(nextPressure)

    def getAirProperties(self, ASLElevation, time):

        # Calculate geopotential altitude (H)
        H = (self.earthRadius * ASLElevation) / (self.earthRadius + ASLElevation)
        
        # Set density to zero above 86 km
        if H >= 86000:
            H = 86000

        # Figure out which interval we're in 
        baseIndex = bisect(self.baseHeights, H) - 1
        altitudeAboveBase = H - self.baseHeights[baseIndex] # Geopotential altitude above base
        dt_dh = self.dt_dh[baseIndex]
        
        # Calc temp
        Tb = self.baseTemps[baseIndex]
        temp = Tb + dt_dh * altitudeAboveBase

        # Calc pressure
        Pb = self.basePressures[baseIndex]
        if dt_dh == 0:
            P_over_Pb = exp(-self.G*self.M * altitudeAboveBase / (self.R * Tb * 1000))
            pressure = P_over_Pb * Pb
        else:
            body = (Tb + dt_dh*altitudeAboveBase) / Tb
            exponent = -self.G*self.M / (self.R*dt_dh*1000)
            pressure = body**exponent * Pb

        # Calculate density and viscosity from temperature and pressure   
        rho = self.M * pressure / (self.R * temp * 1000) # Ideal gas law
        viscosity = 1.457e-6 * temp**(1.5) / (temp + 110.4) # Sutherland's law

        if H >= 86000:
            rho = 0.0

        return [ temp, pressure, rho, viscosity ]
