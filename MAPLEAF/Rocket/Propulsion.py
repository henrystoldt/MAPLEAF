import re

from MAPLEAF.Motion import linInterp
from MAPLEAF.IO import SubDictReader
from MAPLEAF.Motion import Vector
from MAPLEAF.Motion import ForceMomentSystem
from MAPLEAF.Motion import Inertia
from MAPLEAF.Rocket import RocketComponent

__all__ = [ "TabulatedMotor" ]

class TabulatedMotor(RocketComponent, SubDictReader):
    '''
    Interface:
        Initialization:
            In rocket text file, attribute: "path", pointing at a motor definition text file
            Format:"test/testMotorDefintion.txt"
        Functions:
            .Thrust(time) returns current thrust level
            .OxWeight(time) returns current oxidizer weight
            .FuelWeight(time) returns current fuel weight
        Attributes:
            .initialOxidizerWeight
            .initialFuelWeight

        All in same units as in the motor definition file (linearly interpolated)
        The motor is assumed to apply zero moment to the rocket, thrust is applied in z-direction
    '''

    #### Init Functions ####
    def __init__(self, componentDictReader, rocket, stage):
        #TODO: Oxidizer and Fuel CG Locations should be defined relative to the motor location
        self.rocket = rocket
        self.stage = stage
        self.componentDictReader = componentDictReader
        self.name = componentDictReader.getDictName()

        stage.motor = self
        self.classType = componentDictReader.getString("class")
        self.ignitionTime = 0 # Modified by Rocket._initializeStaging and Rocket._stageSeparation

        # Impulse adjustments (mostly for Monte Carlo sims)
        self.impulseAdjustFactor = componentDictReader.getFloat("impulseAdjustFactor")
        self.burnTimeAdjustFactor = componentDictReader.getFloat("burnTimeAdjustFactor")

        motorFilePath = componentDictReader.getString("path")
        self._parseMotorDefinitionFile(motorFilePath)

        # Set the position to the initial CG location
        initInertia = self.getInertia(0, "fakeState")
        self.position = initInertia.CG

    #TODO: Build converter/parser for standard engine format like rasp/.eng or something like that

    def _parseMotorDefinitionFile(self, motorFilePath):
        ''' Parses a motor definition text file. See MAPLEAF/Examples/Motors for examples '''
      
        # Get motor definition text
        with open(motorFilePath, "r") as motorFile:
            motorFileText = motorFile.read()

        # Remove all comment rows
        comment = re.compile("#.*") 
        motorFileText = re.sub(comment, "", motorFileText)
        
        #Remove blank lines
        motorFileText = [line for line in motorFileText.split('\n') if line.strip() != '']
        
        # Parse CG locations
        # TODO: Future motors should be able to exist off the rocket's center axis
        self.initOxCG_Z = float(motorFileText[0].split()[1]) + self.stage.position.Z
        self.finalOxCG_Z = float(motorFileText[1].split()[1]) + self.stage.position.Z
        self.initFuelCG_Z = float(motorFileText[2].split()[1]) + self.stage.position.Z
        self.finalFuelCG_Z = float(motorFileText[3].split()[1]) + self.stage.position.Z
        motorFileText = motorFileText[4:]

        # Parse data; Columns defined in MAPLEAF/Examples/Motors/test.txt
        # Gets defined values for: Time, thrust, oxFlowRate, fuelFlowRate, oxMOI, fuelMOI
        self.times = []
        self.thrustLevels = []
        oxFlowRate = []
        fuelFlowRate = []
        self.oxMOIs = []
        self.fuelMOIs = []
        for dataLine in motorFileText:
            # Splits line at each white space
            info = dataLine.split()

            self.times.append(float(info[0]))
            self.thrustLevels.append(float(info[1]))
            oxFlowRate.append(float(info[2]))
            fuelFlowRate.append(float(info[3]))
            
            oxVecStartIndex = dataLine.index('(')
            oxVecEndIndex = dataLine.index(')', oxVecStartIndex)+1
            oxVecString  = dataLine[oxVecStartIndex:oxVecEndIndex]
            oxMOIVec = Vector(oxVecString)
            self.oxMOIs.append(oxMOIVec)

            fuelVecStartIndex = dataLine.index('(', oxVecEndIndex)
            fuelVecEndIndex = dataLine.index(')', fuelVecStartIndex)+1
            fuelVecString  = dataLine[fuelVecStartIndex:fuelVecEndIndex]
            fuelMOIVec = Vector(fuelVecString)
            self.fuelMOIs.append(fuelMOIVec)

        # Tell the rocket and stage when their engines shut off -> used for flight animations
        self.stage.engineShutOffTime = self.times[-1]
        if self.rocket.engineShutOffTime == None:
            self.rocket.engineShutOffTime = self.times[-1]
        else:
            self.rocket.engineShutOffTime = max(self.rocket.engineShutOffTime, self.times[-1])

        # Apply adjustment factors for monte carlo sims
        self.thrustLevels = [ thrust*self.impulseAdjustFactor/self.burnTimeAdjustFactor for thrust in self.thrustLevels ]
        self.times = [ t*self.burnTimeAdjustFactor for t in self.times ]

        # Calculate initial fuel and oxidizer masses through trapezoid rule
        # Trapezoid rule matches the linear interpolation used to find thrust values
        self.initialOxidizerWeight = 0
        self.initialFuelWeight = 0
        self.oxWeights = [ 0 ]
        self.fuelWeights = [ 0 ]
        for i in range(len(self.times)-1, 0, -1):
            deltaT = self.times[i] - self.times[i-1]
            def integrateVal(value, sum, timeSeries):
                sum += deltaT * (value[i-1] + value[i]) / 2
                timeSeries.insert(0, sum)
                return sum

            self.initialOxidizerWeight = integrateVal(oxFlowRate, self.initialOxidizerWeight, self.oxWeights)
            self.initialFuelWeight = integrateVal(fuelFlowRate, self.initialFuelWeight, self.fuelWeights)

    #### Operational Functions ####
    def getInertia(self, time, state):
        timeSinceIgnition = max(0, time - self.ignitionTime)

        oxInertia = self._getOxInertia(timeSinceIgnition)
        fuelInertia = self._getFuelInertia(timeSinceIgnition)
        
        return oxInertia + fuelInertia

    def getAeroForce(self, state, time, environment, CG):
        #TODO: Model "thrust damping" - where gases moving quickly in the engine act to damp out rotation about the x and y axes
        #TODO: Thrust vs altitude compensation
        timeSinceIgnition = max(0, time - self.ignitionTime)
        
        # Determine thrust magnitude
        if timeSinceIgnition < 0 or timeSinceIgnition > self.times[-1]:
            thrustMagnitude = 0
        else:
            thrustMagnitude = linInterp(self.times, self.thrustLevels, timeSinceIgnition)
        
        # Create Vector
        thrust = Vector(0,0, thrustMagnitude)

        # Log and return
        self.rocket.appendToForceLogLine(" {:>10.4f}".format(thrust.Z))
        return ForceMomentSystem(thrust)

    def updateIgnitionTime(self, ignitionTime, fakeValue=False):
        self.ignitionTime = ignitionTime
        if not fakeValue:
            self.rocket.engineShutOffTime = max(self.rocket.engineShutOffTime, self.ignitionTime + self.times[-1])
            self.stage.engineShutOffTime = self.ignitionTime + self.times[-1]

    def getLogHeader(self):
        return " {}Thrust(N)".format(self.name)

    def getTotalImpulse(self):
        # Integrate the thrust - assume linear interpolations between points given -> midpoint rule integrates this perfectly
        totalImpulse = 0
        for i in range(1, len(self.times)):
            deltaT = self.times[i] - self.times[i-1]
            totalImpulse += deltaT * (self.thrustLevels[i-1] + self.thrustLevels[i]) / 2
        
        return totalImpulse

    def _getMass(self, timeSinceIgnition):
        return self.OxWeight(timeSinceIgnition) + self.FuelWeight(timeSinceIgnition)

    def _getOxInertia(self, timeSinceIgnition):
        if self.initialOxidizerWeight == 0:
            return Inertia(Vector(0,0,0), Vector(0,0,0), 0)

        oxWeight = linInterp(self.times, self.oxWeights, timeSinceIgnition)
        
        # Find fraction of oxidizer burned
        oxBurnedFrac = 1 - (oxWeight/self.initialOxidizerWeight)
        
        #Linearly interpolate CG location based on fraction of oxidizer burned
        oxCG_Z = self.initOxCG_Z*(1 - oxBurnedFrac) + self.finalOxCG_Z*oxBurnedFrac
        #TODO: Allow motor(s) to be defined off-axis
        oxCG = Vector(0,0,oxCG_Z)

        # Get MOI
        oxMOI = linInterp(self.times, self.oxMOIs, timeSinceIgnition)
        
        return Inertia(oxMOI, oxCG, oxWeight)

    def _getFuelInertia(self, timeSinceIgnition):
        if self.initialFuelWeight == 0:
            return Inertia(Vector(0,0,0), Vector(0,0,0), 0)

        #See comments in _getOxInertia()
        fuelWeight = linInterp(self.times, self.fuelWeights, timeSinceIgnition)

        fuelBurnedFrac = 1 - (fuelWeight / self.initialFuelWeight)

        fuelCG_Z = self.initFuelCG_Z*(1 - fuelBurnedFrac) + self.finalFuelCG_Z*fuelBurnedFrac
        fuelCG = Vector(0,0,fuelCG_Z)

        fuelMOI = linInterp(self.times, self.fuelMOIs, timeSinceIgnition)

        return Inertia(fuelMOI, fuelCG, fuelWeight)
