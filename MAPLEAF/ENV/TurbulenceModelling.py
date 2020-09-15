''' Modeling of turbulent, fluctuating component of wind velocity '''

import abc
import random
from math import cos, pi

from MAPLEAF.IO import SubDictReader
from MAPLEAF.Motion import Vector

__all__ = [ "turbulenceModelFactory", "PinkNoiseGenerator" ]

#TODO: Implement vonKarman or Dryden model with std deviation / length scale that varies with altitude as per NASA HDBK-1001

class TurbulenceModel(abc.ABC):
    ''' Interface for all turbulence models '''
    @abc.abstractmethod
    def getTurbVelocity(self, altitude, meanWindVelocity, time):
        pass

def turbulenceModelFactory(simDefinition, silent=False):
    ''' Reads data from simDefinition, initializes and returns the appropriate TurbulenceModel '''

    envReader = SubDictReader("Environment", simDefinition)
    turbModelType = envReader.getString("Environment.TurbulenceModel")
    
    if not silent:
        print("Turbulence/Gust Model: {}".format(turbModelType))

    if turbModelType== "None":
        turbulenceModel = NoTurb()
        
    elif "PinkNoise" in turbModelType:
        def tryGetValue(key):
            try:
                return int(simDefinition.getValue(key))
            except KeyError:
                return None

        measuredPinkNoiseStdDev = 2.26 # For a 2-pole PinkNoiseGenerator - re-measure if the number of poles is changed

        turbulenceIntensity = envReader.tryGetInt("PinkNoiseModel.turbulenceIntensity")
        if turbulenceIntensity != None:
            turbulenceIntensity /= (measuredPinkNoiseStdDev * 100)

        velocityStDev = envReader.tryGetInt("PinkNoiseModel.velocityStdDeviation")
        if velocityStDev != None:
            velocityStDev /= measuredPinkNoiseStdDev

        pinkNoiseRandomSeed1 = envReader.tryGetInt("PinkNoiseModel.randomSeed1")
        pinkNoiseRandomSeed2 = envReader.tryGetInt("PinkNoiseModel.randomSeed2")
        pinkNoiseRandomSeed3 = envReader.tryGetInt("PinkNoiseModel.randomSeed3")
            
        if turbModelType == "PinkNoise1D":
            turbulenceModel = PinkNoise1D(turbulenceIntensity, velocityStDev, pinkNoiseRandomSeed1)
            if not silent:
                print("Random seed 1: {}".format(turbulenceModel.png1.seed))

        elif turbModelType == "PinkNoise2D":
            turbulenceModel = PinkNoise2D(turbulenceIntensity, velocityStDev, pinkNoiseRandomSeed1, pinkNoiseRandomSeed2)
            if not silent:
                print("Random seed 1: {}".format(turbulenceModel.png1.seed))
                print("Random seed 2: {}".format(turbulenceModel.png2.seed))

        elif turbModelType == "PinkNoise3D":
            turbulenceModel = PinkNoise3D(turbulenceIntensity, velocityStDev, pinkNoiseRandomSeed1, pinkNoiseRandomSeed2, pinkNoiseRandomSeed3)
            if not silent:
                print("Random seed 1: {}".format(turbulenceModel.png1.seed))
                print("Random seed 2: {}".format(turbulenceModel.png2.seed))
                print("Random seed 2: {}".format(turbulenceModel.png3.seed))
            
    elif turbModelType == "customSineGust":
        GustStartAltitude = envReader.getFloat("CustomSineGust.startAltitude")
        GustMagnitude = envReader.getFloat("CustomSineGust.magnitude")
        GustSineBlendDistance = envReader.getFloat("CustomSineGust.sineBlendDistance")
        GustLayerThickness = envReader.getFloat("CustomSineGust.thickness")
        GustDirection = envReader.getVector("CustomSineGust.direction").normalize()

        turbulenceModel = CustomSineGust(GustStartAltitude, GustSineBlendDistance, GustLayerThickness, GustMagnitude, GustDirection)

        if not silent:
            print("Gust Altitude: {}".format(GustStartAltitude))
            print("Gust Vector: {}".format(GustMagnitude * GustDirection))

    else:
        raise ValueError("Turbulence Model {} not found. Please choose from: None, PinkNoise1/2/3D or customSineGust")

    return turbulenceModel

class NoTurb(TurbulenceModel):
    def getTurbVelocity(self, _, __, ___):
        return Vector(0,0,0)

class _PinkNoiseTurbModel(TurbulenceModel):
    def __init__(self, turbulenceIntensity, velocityStdDev, randomSeed):
        self.velStdDev = velocityStdDev
        self.turbulenceIntensity = turbulenceIntensity
        self.png1 = PinkNoiseGenerator(seed=randomSeed)

    # Dummy function, will be replaced by subclasses
    def getTurbVelocity(self, altitude, meanWindVelocity, time):
        return Vector(0,0,0)

    def _getDesiredTurbulenceStdDev(self, altitude, meanWindVelocity):
        if self.turbulenceIntensity != None:
            return meanWindVelocity.length() * self.turbulenceIntensity
        else:
            return self.velStdDev

class PinkNoise1D(_PinkNoiseTurbModel):
    def __init__(self, turbulenceIntensity, velocityStdDev, randomSeed):
        super().__init__(turbulenceIntensity, velocityStdDev, randomSeed)

    def getTurbVelocity(self, altitude, meanWindVelocity, time):
        self.velStdDev = super()._getDesiredTurbulenceStdDev(altitude, meanWindVelocity)
        newTurbMagnitude = self.png1.getValue(time) * self.velStdDev
        return meanWindVelocity.normalize() * newTurbMagnitude

class PinkNoise2D(_PinkNoiseTurbModel):
    def __init__(self, turbulenceIntensity, velocityStdDev, randomSeed, randomSeed2):
        super().__init__(turbulenceIntensity, velocityStdDev, randomSeed)
        self.png2 = PinkNoiseGenerator(seed=randomSeed2)

    def getTurbVelocity(self, altitude, meanWindVelocity, time):
        velStdDev = self._getDesiredTurbulenceStdDev(altitude, meanWindVelocity)
        xTurbVel = self.png1.getValue(time) * velStdDev
        yTurbVel = self.png2.getValue(time) * velStdDev
        return Vector(xTurbVel, yTurbVel, 0)

class PinkNoise3D(TurbulenceModel):
    def __init__(self, turbulenceIntensity, velocityStdDev, randomSeed, randomSeed2, randomSeed3):
        super().__init__(turbulenceIntensity, velocityStdDev, randomSeed)
        self.png2 = PinkNoiseGenerator(seed=randomSeed2)
        self.png3 = PinkNoiseGenerator(seed=randomSeed3)

    def getTurbVelocity(self, altitude, meanWindVelocity, time):
        velStdDev = self._getDesiredTurbulenceStdDev(altitude, meanWindVelocity)      
        newXTurbVel = self.png1.getValue(time) * velStdDev
        newYTurbVel = self.png2.getValue(time) * velStdDev
        newZTurbVel = self.png2.getValue(time) * velStdDev
        return Vector(newXTurbVel, newYTurbVel, newZTurbVel)

class CustomSineGust(TurbulenceModel):
    def __init__(self, GustStartAltitude, GustSineBlendDistance, GustLayerThickness, GustMagnitude, GustDirection):
        self.GustStartAltitude = GustStartAltitude
        self.GustSineBlendDistance = GustSineBlendDistance
        self.GustMagnitude = GustMagnitude
        self.GustDirection = GustDirection
        
        self.GustZoneBoundaries = [
            GustStartAltitude,
            GustStartAltitude + GustSineBlendDistance, # End of lower blend zone
            GustStartAltitude + GustSineBlendDistance + GustLayerThickness, # Start of upper blending zone
            GustStartAltitude + GustSineBlendDistance + GustLayerThickness + GustSineBlendDistance # End of upper blending zone
        ]

    def getTurbVelocity(self, altitude, meanWindVelocity, time):
        ''' # From NASA HDBK-1001, pg. 2-84, equation 2.69 '''
        # Determine whether the current altitude is in the blending region or plateau region
        if altitude > self.GustStartAltitude:
            if altitude < self.GustZoneBoundaries[1]:
                # In the lower blending region
                gustVel = self.GustMagnitude/2 * ( 1 - cos(pi*(altitude - self.GustStartAltitude)/self.GustSineBlendDistance))
            elif altitude < self.GustZoneBoundaries[2]:
                # In the plateau (gust) region
                gustVel = self.GustMagnitude
            elif altitude < self.GustZoneBoundaries[3]:
                # In the upper blending region
                gustThickness = self.GustZoneBoundaries[-1] - self.GustZoneBoundaries[0]
                gustVel = self.GustMagnitude/2 * ( 1 - cos(pi*(altitude - self.GustStartAltitude - gustThickness)/self.GustSineBlendDistance))
            else:
                return Vector(0,0,0)

            return self.GustDirection * gustVel
        else:
            return Vector(0,0,0)
        
class PinkNoiseGenerator():
    '''
        Inputs:
            
            * alpha - pink noise generated has a power spectrum which has a log-log slope of -alpha.  
                default value of 5/3 matches power spectrum slope of turbulent integral scales  
            * seed - pink noise generated based on applying a filter to gaussian white noise specifying  
                the seed for the input random noise allows for repeatable sequences of values  
            * nPoles - more poles include more low frequency content in the spectrum  
                parameter could be interpreted as similar to the turbulence length scale  
                can be varied in more sophisticated models  
            * simulatedSamplingFreqHz - controls "sampling frequency" that the pink noise is interpreted as  
                also modulated the effective length scale of turbulence, together with nPoles  
        Methods:
            
            * .getValue(time)  
                If the value is requested at time values that fall between sampling intervals, linearly 
                interpolated values (from adjacent samples) are returned.
                If no time is provided, the sampling frequency is ignored and a new value is computed  
    '''

    def __init__(self, alpha=5/3, seed=None, nPoles=2, simulatedSamplingFreqHz=20):
        self.alpha = alpha
        if seed != None:
            self.seed = seed
        else:
            self.seed = random.randrange(1000000)
        self.rng = random.Random(self.seed)
        # self.rng.seed(self.seed)
        self.nPoles = nPoles

        # Calculate coefficient values - openrocket documentation pg. 59/60 (Section 4.1.2 Wind Modeling)
        # Based on method from Kasdin (1995) "Discrete Simulation of Colored Noise and Stochastic Processes and 1/f^{alpha} Power Law Noise Generation"
        self.coeffs = [ 1, ]
        for i in range(nPoles):
            self.coeffs.append((i - alpha/2) * self.coeffs[i]/(i+1))
        self.coeffs.pop(0)

        # Overwriting will wrap around the array to avoid constantly shifting values
        self.nextIndexToOverwrite = 0

        # Fixed sampling frequency allows for achieving "mesh convergence" as the timestep is decreased
        self.lastValTime = 0
        self.timeStep = 1 / simulatedSamplingFreqHz # seconds

        # Fill lastValues
        self.lastValues = [ self.rng.gauss(0, 1) for i in range(nPoles) ]
        for i in range(10*nPoles):
            self.getValue()

    def getValue(self, time=None):
        '''
            Calculate coefficient values - openrocket documentation pg. 59/60 (Section 4.1.2 Wind Modeling)
            Based on method from Kasdin (1995) "Discrete Simulation of Colored Noise and Stochastic Processes and 1/f^{alpha} Power Law Noise Generation"
        '''
        # Compute any required new values
        while time == None or time > self.lastValTime:
            whiteNoise = self.rng.gauss(0, 1)

            for i in range(self.nPoles):
                whiteNoise -= self.coeffs[i]*self.lastValues[(self.nextIndexToOverwrite - i - 1) % self.nPoles]
            pinkNoiseSample = whiteNoise

            self.lastValues[self.nextIndexToOverwrite] = pinkNoiseSample
            self.nextIndexToOverwrite = (self.nextIndexToOverwrite + 1) % self.nPoles

            if time == None:
                return pinkNoiseSample

            self.lastValTime += self.timeStep
            if self.lastValTime == time:
                return pinkNoiseSample

        # Linearly interpolate value at desired time
        lastVal = self.lastValues[self.nextIndexToOverwrite - 1]
        secondLastVal = self.lastValues[self.nextIndexToOverwrite - 2]
        secondLastValTime = self.lastValTime - self.timeStep
        
        dValdt = (lastVal - secondLastVal) / self.timeStep
        return dValdt*(time-secondLastValTime) + secondLastVal
