from timeit import timeit

nTests=10000
print("Each operation performed {} times".format(nTests))






print("")
print("interp1D * 5")
print("")

setup = '''
import pandas as pd
from scipy.interpolate import interp1d
from collections import namedtuple

getAirProperties = namedtuple("getAirProperties", [ "Temp", "Pressure", "Density", "GravityAccel", "DynamicViscosity" ])

class StandardAtmosphere():
    """
    Interface:
    Pass elevation to:
    .temp (K)
    .pressure (Pa)
    .density (kg/m3)
    .g (m/s^2)
    .dynamicViscosity (Pa*s)
    .getAirProperties (for all of the above in a named tuple)
    """
    
    #TODO: Wind
    #TODO: Initial Altitude
    def __init__(self):
        # Import US Standard Atmosphere data from table
        df = pd.read_csv("MAPLEAF/ENV/US_STANDARD_ATMOSPHERE.txt", sep="\t")
        
        # Parse data into separate arrays and alter to get better units
        elevationData = df['h'].values # m, above sea level
        temperature = df['T'].values + 273.15 # K
        gravity = df['g'].values # m/(s^2)
        pressure = df['P'].values * 1E4 # Pa
        density = df['rho'].values # kg/m3
        dynamic_viscosity = df['mu'].values * 1E-5 # Pa-s 
        
        # Create interpolation functions
        self.temperature_function = interp1d(elevationData, temperature, assume_sorted=True)
        self.pressure_function = interp1d(elevationData, pressure, assume_sorted=True)
        self.density_function = interp1d (elevationData, density, assume_sorted=True)
        self.gravity_function = interp1d(elevationData, gravity, assume_sorted=True)
        self.dynamic_viscosity_function = interp1d(elevationData, dynamic_viscosity, assume_sorted=True)

    def temp(self, elevation):
        return self.temperature_function(elevation)

    def pressure(self, elevation):
        return self.pressure_function(elevation)

    def density(self, elevation):
        return self.density_function(elevation)

    def g(self, elevation):
        return self.gravity_function(elevation)

    def dynamicViscosity(self, elevation):
        return self.dynamic_viscosity_function(elevation)

    def getAirProperties(self, elevation):
        return getAirProperties(self.temp(elevation), self.pressure(elevation), self.density(elevation), self.g(elevation), self.dynamicViscosity(elevation))

SA = StandardAtmosphere()
print("Result: {}".format(SA.getAirProperties(150)))
'''
print("Interpolating Atmosphere at 150m:")
print(timeit("a = SA.getAirProperties(150)", setup=setup, number=nTests))








print("")
print("interp1d * 1")
print("")

setup = '''
import numpy as np
from scipy.interpolate import interp1d
from collections import namedtuple

getAirProperties = namedtuple("getAirProperties", [ "Temp", "Pressure", "Density", "GravityAccel", "DynamicViscosity" ])

class StandardAtmosphere():
    """
    Interface:
    Pass elevation to:
    .temp (K)
    .pressure (Pa)
    .density (kg/m3)
    .g (m/s^2)
    .dynamicViscosity (Pa*s)
    .getAirProperties (for all of the above in a named tuple)
    """
    
    #TODO: Wind
    #TODO: Initial Altitude
    def __init__(self):
        # Import US Standard Atmosphere data from table
        df = np.loadtxt("MAPLEAF/ENV/US_STANDARD_ATMOSPHERE.txt", skiprows=1)

        # Parse data into separate arrays and alter to get better units
        df[:,1] += 273.15 # K
        df[:, 3] *= 1E4 # Pa
        df[:, 5] *= 1E-5 # Pa-s
        keys = df[:,0]
        values = df[:,1:]
        self.interpolate = interp1d(keys, values, axis=0, assume_sorted=True)

    def getAirProperties(self, elevation):
        return getAirProperties(*self.interpolate(elevation))

SA = StandardAtmosphere()
print("Result: {}".format(SA.getAirProperties(150)))
'''

print("Interpolating Atmosphere at 150m:")
print(timeit("a = SA.getAirProperties(150)", setup=setup, number=nTests))








print("")
print("Custom interp: np.searchsorted")
print("")

setup = '''
import numpy as np
from collections import namedtuple

getAirProperties = namedtuple("getAirProperties", [ "Temp", "Pressure", "Density", "GravityAccel", "DynamicViscosity" ])

def linInterp(X, Y, desiredX):
    #Interp
    interpPt = np.searchsorted(X, desiredX)
    lgX = X[interpPt]
    smX = X[interpPt-1]
    lgY = Y[interpPt]
    smY = Y[interpPt-1]

    return (lgY - smY)*(desiredX - smX)/(lgX - smX) + smY

class StandardAtmosphere():
    """
    Interface:
    Pass elevation to:
    .temp (K)
    .pressure (Pa)
    .density (kg/m3)
    .g (m/s^2)
    .dynamicViscosity (Pa*s)
    .getAirProperties (for all of the above in a named tuple)
    """
    
    #TODO: Wind
    #TODO: Initial Altitude
    def __init__(self):
        # Import US Standard Atmosphere data from table
        df = np.loadtxt("MAPLEAF/ENV/US_STANDARD_ATMOSPHERE.txt", skiprows=1)

        # Parse data into separate arrays and alter to get better units
        df[:,1] += 273.15 # K
        df[:, 3] *= 1E4 # Pa
        df[:, 5] *= 1E-5 # Pa-s
        self.keys = df[:,0]
        self.values = df[:,1:]

    def getAirProperties(self, elevation):
        return getAirProperties(*linInterp(self.keys, self.values, elevation))

SA = StandardAtmosphere()
print("Result: {}".format(SA.getAirProperties(150)))
'''

print("Interpolating Atmosphere at 150m:")
print(timeit("a = SA.getAirProperties(150)", setup=setup, number=nTests))








print("")
print("Custom interp: bisect")
print("")

setup = '''
import numpy as np
from bisect import bisect
from collections import namedtuple

getAirProperties = namedtuple("getAirProperties", [ "Temp", "Pressure", "Density", "GravityAccel", "DynamicViscosity" ])

def linInterp(X, Y, desiredX):
    interpPt = bisect(X, desiredX)
    lgX = X[interpPt]
    smX = X[interpPt-1]
    lgY = Y[interpPt]
    smY = Y[interpPt-1]

    return (lgY - smY)*(desiredX - smX)/(lgX - smX) + smY

class StandardAtmosphere():
    """
    Interface:
    Pass elevation to:
    .temp (K)
    .pressure (Pa)
    .density (kg/m3)
    .g (m/s^2)
    .dynamicViscosity (Pa*s)
    .getAirProperties (for all of the above in a named tuple)
    """
    
    def __init__(self):
        # Import US Standard Atmosphere data from table
        df = np.loadtxt("MAPLEAF/ENV/US_STANDARD_ATMOSPHERE.txt", skiprows=1)

        # Parse data into separate arrays and alter to get better units
        df[:,1] += 273.15 # K
        df[:, 3] *= 1E4 # Pa
        df[:, 5] *= 1E-5 # Pa-s
        self.keys = df[:,0]
        self.values = df[:,1:]

    def getAirProperties(self, elevation):
        return getAirProperties(*linInterp(self.keys, self.values, elevation))

class StandardAtmosphereLg():
    """
    Interface:
    Pass elevation to:
    .temp (K)
    .pressure (Pa)
    .density (kg/m3)
    .g (m/s^2)
    .dynamicViscosity (Pa*s)
    .getAirProperties (for all of the above in a named tuple)
    """
    
    def __init__(self):
        # Import US Standard Atmosphere data from table
        df = np.loadtxt("test/speedTests/largerTestInterpData.txt", skiprows=1)

        # Parse data into separate arrays and alter to get better units
        df[:,1] += 273.15 # K
        df[:, 3] *= 1E4 # Pa
        df[:, 5] *= 1E-5 # Pa-s
        self.keys = df[:,0]
        self.values = df[:,1:]

    def getAirProperties(self, elevation):
        return getAirProperties(*linInterp(self.keys, self.values, elevation))

SA = StandardAtmosphere()
SALg = StandardAtmosphereLg()
print("Result: {}".format(SA.getAirProperties(150)))
'''

print("Interpolating Atmosphere at 150m:")
print(timeit("a = SA.getAirProperties(150)", setup=setup, number=nTests))
print("Interpolating Larger array at 150.234m:")
print(timeit("a = SALg.getAirProperties(150.234)", setup=setup, number=nTests))