'''
Defines functions to calculate the Mach and Re numbers, angles of attack, etc...
Several have a standardized interface, and are used for control gain interpolations in `MAPLEAF.GNC`
'''
import math

from MAPLEAF.Motion import Vector
from MAPLEAF.Utilities import cacheLastResult


def getAltitude(state, environment, _=None):
    return environment.ASLAltitude

@cacheLastResult
def getMachNumber(state, environment, _=None):
    # Otherwise compute and cache the result
    airVel = state.velocity - environment.Wind
    Ma = airVel.length() / math.sqrt(1.4 * 287 * environment.Temp)    #TODO: Get gamma and R from default dict / config file
    return Ma

@cacheLastResult
def getReynoldsNumber(state, environment, length):
    unitRe = getLocalFrameAirVel(state, environment).length() * environment.Density / environment.DynamicViscosity
    return unitRe * length

@cacheLastResult
def getTotalAOA(state, environment, _=None):
    ''' Returns the **magnitude** of the angle between the local-frame wind vector and longitudinal axis, in degrees '''
    localFrameAirVel = getLocalFrameAirVel(state, environment)

    if localFrameAirVel.length() == 0:
        AOA = 0
    else:
        AOA = localFrameAirVel.angle(Vector(0,0,-1))

    return AOA

def getAOA(state, environment, _=None):
    ''' 
        Returns angle of attack (between the local Z-X plane and the airspeed vector)
        Measures rotations about the local +y axis (compared to wind-aligned state)
    '''
    localFrameAirVel = getLocalFrameAirVel(state, environment)

    if localFrameAirVel.length() == 0:
        return 0

    # Zero out the Y-component (Sideslip component)
    localFrameAirVel.Y = 0
    
    # Compute angle with longitudinal axis
    if localFrameAirVel.X > 0:
        return localFrameAirVel.angle(Vector(0,0,-1))
    else:
        return -localFrameAirVel.angle(Vector(0,0,-1))

def getAOSS(state, environment, _=None):
    ''' 
        Returns angle of sideslip (between the local Z-Y plane and the airspeed vector)
        Measures rotations about the local +z axis (compared to wind-aligned state)
    '''
    localFrameAirVel = getLocalFrameAirVel(state, environment)

    if localFrameAirVel.length() == 0:
        return 0

    # Zero out the X-component (AOA component)
    localFrameAirVel.X = 0

    # Compute angle with longitudinal axis
    if localFrameAirVel.Y < 0:
        return localFrameAirVel.angle(Vector(0,0,-1))
    else:
        return -localFrameAirVel.angle(Vector(0,0,-1))

@cacheLastResult
def getRollAngle(state, environment, _=None):
    ''' 
        Returns the angular distance between axis about which the rocket rotates to create an angle of attack, and the X-axis, in degrees
        If the normal force direction is aligned with the y-axis, roll angle = 0
        If the normal force direction is aligned with the x-axis, roll angle = 90 degrees
    '''

    # TODO: Convert this to using radians
    normalAeroForceDirection = getNormalAeroForceDirection(state, environment)
    y = normalAeroForceDirection*(-1)
    if(y.length() < 0.01): #Check for when there is a zero angle of attack (Numerical issue)
        rollAngle = 0
    else:
        rollAngle = math.degrees(math.atan2(-y.Y, y.X))
        rollAngle = (-1)*rollAngle
        rollAngle %= 360

    return rollAngle

def getAeroPropertiesList(aeroFunctionList, state, environment):
    '''
        Returns a vector of parameters specified by strings - used for interpolating by user-specified parameters

        aeroFunctionVector:     vector of references to standardized-interface functions below
        state:                  rocket rigid body state
        environment:            environmentalConditions named tuple
    '''
    aeroProperties = []
    for fn in aeroFunctionList:
        aeroProperties.append(fn(state, environment, 1))

    return aeroProperties

stringToAeroFunctionMap = {
    # This defines the relationship between strings obtained from sim config files,
        # To functions that obtain some parameter, often used for interpolation of tabulated properties
        # Use by control systems, TabulatedAeroForce, and TabulatedInertia objects
    "Mach":         getMachNumber,
    "Altitude":     getAltitude,
    "UnitReynolds": getReynoldsNumber,
    "TotalAOA":     getTotalAOA,
    "RollAngle":    getRollAngle,
    "AOA":          getAOA,
    "AOSS":         getAOSS
}



#### Other Functions ####
@cacheLastResult
def getLocalFrameAirVel(state, environment):
    ''' Returns the vector representing the motion of freestream air relative to the rocket, in the rocket frame '''
    return state.orientation.conjugate().rotate(state.velocity - environment.Wind)*-1

def getAirVelRelativeToVehicle(state, environment):
    ''' 3DoF version of getLocalFrameAirVel '''
    return (state.velocity - environment.Wind) * -1

@cacheLastResult
def getNormalAeroForceDirection(state, environment):
    localFrameAirVel = getLocalFrameAirVel(state, environment)
    if localFrameAirVel[0] == 0.0 and localFrameAirVel[1] == 0.0:
        # Return random vector if AOA == 0 - force will be zero anyhow
        return Vector(1,0,0)
    else:
        return Vector(localFrameAirVel[0], localFrameAirVel[1], 0).normalize()

@cacheLastResult
def getDynamicPressure(state, environment):
    try:
        velMag = getLocalFrameAirVel(state, environment).length()
    except AttributeError:
        #3DoF mode
        velMag = getAirVelRelativeToVehicle(state, environment).length()

    return velMag*velMag * environment.Density / 2

@cacheLastResult
def getBeta(Mach):
    #Define the compressibility factor depending on if we are subsonic or supersonic, if we are at M = 1 we set
    #the compressibility factor to a hardcoded literal close to zero (equations break if not)
    if Mach <= 0.9: 
        return math.sqrt(1 - Mach**2) #Compressibility correction factor for subsonic flow
    elif Mach >= 1.1:
        return math.sqrt(Mach**2 - 1) #For supersonic flow
    else:
        raise NotImplementedError("Beta not valid in the range 0.9 < M < 1.1 - goes to infinity")
