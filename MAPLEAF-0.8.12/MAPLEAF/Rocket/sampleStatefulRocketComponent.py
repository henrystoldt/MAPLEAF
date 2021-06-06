from MAPLEAF.Motion import ForceMomentSystem, Inertia, Vector
from MAPLEAF.Rocket import RocketComponent

__all__ = [ "SampleStatefulComponent" ]

class SampleStatefulComponent(RocketComponent):
    def __init__(self, componentDictReader, rocket, stage):
        self.rocket = rocket
        self.stage = stage
        self.name = componentDictReader.getDictName()

    def getExtraParametersToIntegrate(self):
        # Examples below for a single parameter to be integrated, can put as many as required in these lists
        paramNames = [ "tankLevel" ]
        initValues = [ 1.0 ]
        derivativeFunctions = [ self.getTankLevelDerivative ]
        
        return paramNames, initValues, derivativeFunctions

    def getTankLevelDerivative(self, time, rocketState):
        return -2*rocketState.tankLevel # tankLevel will asymptotically approach 0

    def getAppliedForce(self, rocketState, time, envConditions, rocketCG):
        mag = -2000*self.getTankLevelDerivative(time, rocketState) # Force magnitude proportional to flow rate out of the tank
        forceVector = Vector(0, 0, mag)

        self.rocket.appendToForceLogLine(" {:>6.4f}".format(mag)) # This will end up in the log file, in the SampleZForce column
        
        return ForceMomentSystem(forceVector)

    def getInertia(self, time, rocketState):
        mass = 5 + rocketState.tankLevel*4.56 # Fixed Mass + fluid mass
        MOI = Vector(mass, mass, mass*0.05) # Related to current mass

        CGz = -3 + rocketState.tankLevel # Moves depending on current tank level
        CG = Vector(0, 0, CGz)
        
        return Inertia(MOI, CG, mass)