import re

from MAPLEAF.Motion import ForceMomentSystem, Inertia, Vector, linInterp
from MAPLEAF.Rocket import RocketComponent
from MAPLEAF.IO import SubDictReader
import math

__all__ = [ "TabulatedMotor" , "DefinedMotor"]

class DefinedMotor(RocketComponent, SubDictReader):
    '''
    Interface:
        Initialization:
            In rocket text file, attribute: "type", is used to specify the motor used in the sim
            Format:"test/testMotorDefintion.txt"

            In rocket text file, attribute: "path", pointing at motor consolidated data
            Format:"test/testMotorDefintion.txt"

            In rocket text file, attribute: "number", indicates number of motors needed for the stage
        Functions:
            .Thrust(time) returns current thrust level, from engine data
            .OxWeight(time) returns current oxidizer weight
            .FuelWeight(time) returns current fuel weight
        Attributes:
            .initialOxidizerWeight
            .initialFuelWeight
            
        The motor is assumed to apply zero moment to the rocket, thrust is applied in z-direction
    '''
    def __init__(self, componentDictReader, rocket, stage):
        self.rocket = rocket
        self.stage = stage
        self.componentDictReader = componentDictReader
        self.name = componentDictReader.getDictName()
        self.diameterRef =  self.stage.bodyTubeDiameter
        stage.motor = self
        self.classType = componentDictReader.getString("class")
        self.ThrustAfterBurnout = componentDictReader.tryGetBool("ThrustAfterBurnout", defaultValue = False)
        
        self.timeExtraStart = 0
        self.timeExtraEnd = 0
        self.timeExtraFlag = False
        self.extraMassProp = 0
        
        self.ignitionTime = 0 # Default ignition time, if the engine gets turned off, this value gets delayed

        # Impulse adjustments (mostly for Monte Carlo sims)
        # self.impulseAdjustFactor = componentDictReader.getFloat("impulseAdjustFactor")
        # self.burnTimeAdjustFactor = componentDictReader.getFloat("burnTimeAdjustFactor")

        motorType = componentDictReader.getString("type") # Extracts the motor type from the consolidated engine data
        self.numMotors = int(float(componentDictReader.getString("number"))) # Extracts number of motors used in the stage
        motorFilePath = componentDictReader.getString("path") # Reads path to where the engine data is stored
        self._parseMotorDefinitionFile(motorFilePath, motorType) # Calls the parse function to separate the data into columns

        # Set the position to the initial CG location
        initInertia = self.getInertia(0, "fakeState")
        self.position = initInertia.CG

    def _parseMotorDefinitionFile(self, motorFilePath, motorType):
        ''' Parses a motor definition text file. See MAPLEAF/Examples/Motors for examples '''
        
        # Get motor definition text
        with open(motorFilePath, "r") as motorFile:
            motorFileText = motorFile.read()

        # Remove all comment rows
        comment = re.compile("#.*") 
        motorFileText = re.sub(comment, "", motorFileText)
        
        #Remove blank lines
        motorFileText = [line for line in motorFileText.split('\n') if line.strip() != '']

        # Parse data; Columns defined in MAPLEAF\Examples\Motors\Engine_list_test.txt
        # Gets defined values for: motorType, fuelDensity, oxyDensity, oxyFuelRatio, ISP and maxThrust

        self.motorSelection   =    []
        self.isp              =    []
        self.fuelDensity      =    []
        self.oxyDensity       =    []
        self.oxyFuelRatio     =    []
        self.engineThrust     =    []
        # self.massPropTotal  =    []
        self.stageDiameter    =    []
        self.motorMass        =    []
        self.motorDiameter    =    []

        # Each column in the data spreadsheet is parsed out into individual columns
        for dataLine in motorFileText:
            info = dataLine.split()
            self.motorSelection.append((info[0]))
            self.fuelDensity.append(float(info[1]))
            self.oxyDensity.append(float(info[2]))
            self.oxyFuelRatio.append(float(info[3]))
            self.isp.append(float(info[4]))
            self.engineThrust.append(float(info[5]))
            self.motorMass.append(float(info[6]))
            self.motorDiameter.append(float(info[7]))

        # From the defined motor in the sim def, the data is pulled for that specific motor
            
        place = self.motorSelection.index(re.sub('\.\d+', '', motorType)) #TODO: make re check if there is letters in the string. If so, do not extract decimal or numbers. Potential bug for motor names containing "V1.1" etc.
        self.motorISP          = self.isp[place]
        self.motorFuelDensity  = self.fuelDensity[place]
        self.motorOxyDensity   = self.oxyDensity[place]
        self.motorOxyFuelRatio = self.oxyFuelRatio[place]
        self.motorEngineThrust = self.engineThrust[place]*self.numMotors
        self.motorEnginemass   = self.motorMass[place]*self.numMotors
        self.motorEngineDiameter = self.motorDiameter[place]

        self.motorStageDiameter = self.diameterRef # Diameter is pulled from bodyTube within stage

        self.updateProp = False # Flag indicator used to update the propellant total mass when motor is turned off and on

        # Computes the total available volume in the body tube
        volumeTotal =  (math.pi/4)*(self.stage.bodyTubeDiameter**2)*self.stage.bodyTubeLength
        # Computes the amount of volume taken up by other components (Engine, Tanks, etc.)
        volumeComponent = 0.05*volumeTotal #NOTE: Random Compoenet Volume, fraction needs to be standardised
        # Computes the fraction of components that are taken up in body tube
        volComponentFrac = volumeComponent/volumeTotal
        # The rest of the space is taken up by fuel (SHould account for tanks, etc.)
        volPropFrac = 1 - volComponentFrac
        # Total Volume of propellants that is in the Body Tube
        volumeProp = volPropFrac*volumeTotal
        # Compute the total mass of the propellants from total volume taken up by fuel and oxydiser
        self.motorMassPropTotal = self.motorFuelDensity*volumeProp + ((volumeProp*self.motorFuelDensity*self.motorOxyFuelRatio)/(self.motorOxyDensity+self.motorFuelDensity*self.motorOxyFuelRatio))*(self.motorOxyDensity - self.motorFuelDensity)

        # Sets the initial CG of the Oxydiser (Stacked Above Fuel)
        # TODO: Right now, doesnt account for NoseConeLength, need to obtain position of the bodyTube  
        self.initMassOxy = self.motorMassPropTotal/(1+(1/self.motorOxyFuelRatio))
        self.initVolumeOxy = self.initMassOxy/self.motorOxyDensity
        self.initLengthOxy = self.initVolumeOxy/((math.pi/4)*self.motorStageDiameter**2) #Assumes cylindrical oxy tank with same diameter as bodytube
        self.initOxCG_Z = ((self.stage.bodyTubePosition.Z) - self.initLengthOxy/2) # Formally .position.Z
        self.finalOxCG_Z = (-self.initLengthOxy + self.stage.bodyTubePosition.Z) # Formally .position.Z

        # Sets the Inital and Final CG Locations for the Fuel (Stacked Below Oxidizer)
        self.initMassFuel = self.motorMassPropTotal/(self.motorOxyFuelRatio+1)
        self.initVolumeFuel = self.initMassFuel/self.motorFuelDensity
        initLengthFuel = self.initVolumeFuel/((math.pi/4)*self.motorStageDiameter**2) #Assumes cylindrical oxy tank with same diameter as bodytube
        self.initFuelCG_Z =  (self.finalOxCG_Z - initLengthFuel/2)
        self.finalFuelCG_Z = (-initLengthFuel + self.finalOxCG_Z)

        gravity = 9.81
        massFlowProp = (self.motorEngineThrust/(gravity*self.motorISP))
        burnTime = self.motorMassPropTotal/massFlowProp

        # Engine Shuts off when the time exceeds burntime from when engines turn on.
        self.rocket.engineShutOffTime = self.ignitionTime + burnTime
        self.stage.engineShutOffTime = self.ignitionTime + burnTime

        self.stageList = None

    #### Operational Functions ####
    def getInertia(self, time, state):

        # Checks the powered state of the engine
        if self.rocket.engineShutOff == True:
        
        # Set ignition time to the current time so that flowrates resume when engines are turned on again
            self.ignitionTime = time
        else:
        # If the Engine is turned on, flip this switch to be able to update total prop mass when engines turn off again
            self.updateProp = False

        # Updates the total amount of propellant left when engines get shut off
        if self.rocket.engineShutOff == True and self.updateProp == False:
            self.updateAmountPropellant(time)

        # Flips switch so that propellent does not get updated as engine remains off
            self.updateProp = True

        # Set time since ignition when engines are turned on
        timeSinceIgnition = max(0, time - self.ignitionTime)
        gravity = 9.81
        massFlowProp = (self.motorEngineThrust/(gravity*self.motorISP)) 
        burnTime = self.motorMassPropTotal/massFlowProp

        # Calls to obtain oxydiser and fuel inertia as propellant gets depleted.
        oxInertia = self._getOxInertia(timeSinceIgnition)
        fuelInertia = self._getFuelInertia(timeSinceIgnition)
        engineInertia = self._getEngineInertia(timeSinceIgnition)
        
        return oxInertia + fuelInertia + engineInertia

    # Function to update global propellant mass when engines are turned off
    def updateAmountPropellant(self, time):
        gravity = 9.81
        massFlowProp = (self.motorEngineThrust/(gravity*self.motorISP)) 
        massPropBurned = massFlowProp*time
        self.motorMassPropTotal = self.motorMassPropTotal - massPropBurned

    def extraPropellantAmount(self, time):
        gravity = 9.81
        massFlowProp = (self.motorEngineThrust/(gravity*self.motorISP))
        self.extraMassProp = massFlowProp*(self.timeExtraEnd-self.timeExtraStart)
        extraMassOx = self.extraMassProp/(1+(1/self.motorOxyFuelRatio))
        extraMassFuel = self.extraMassProp/(self.motorOxyFuelRatio+1)
        
        return(extraMassOx, extraMassFuel)

    def getAeroForce(self, state, time, environment, CG):

        # Checks to see the powered state of the engine
        timeSinceIgnition = max(0,time - self.ignitionTime)

        if self.rocket.engineShutOff == True:
            # Sets the time to the current time so that mass flow of propellants goes to zero
            self.ignitionTime = time

        gravity = 9.81

        #TODO: Account for Variable ISP --> Mass flow of the propellants is computed asuming constant ISP

        massFlowProp = (self.motorEngineThrust/(gravity*self.motorISP)) 
        massPropBurned = massFlowProp*timeSinceIgnition
        burnTime = self.motorMassPropTotal/massFlowProp
        thrustMagnitude = 0

        if timeSinceIgnition >= burnTime  and self.timeExtraFlag == False and self.rocket.orbitalVelocityReached == False and len(self.rocket.stages) == 1 and self.ThrustAfterBurnout == True:
            self.timeExtraStart = time
            self.timeExtraFlag = True

        elif timeSinceIgnition >= burnTime  and self.timeExtraFlag == True and self.rocket.orbitalVelocityReached == True and len(self.rocket.stages) == 1 and self.ThrustAfterBurnout == True:
            self.timeExtraEnd = time
            self.timeExtraFlag = False

        #Determine the magnitude of Thrust from Specified Motor
        if (timeSinceIgnition <= 0 or timeSinceIgnition > burnTime) and len(self.rocket.stages) > 1: # Checks to see if Engine is powered on
            thrustMagnitude = 0

        # elif massPropBurned >= self.motorMassPropTotal and len(self.rocket.stages) > 1: # Checks to see if propellent mass is used up
        #     thrustMagnitude = 0

        elif self.ThrustAfterBurnout == False and len(self.rocket.stages) == 1 and timeSinceIgnition > burnTime:
            thrustMagnitude = 0

        elif self.rocket.engineShutOff == True: # Checks the powered state of the motor
            thrustMagnitude = 0

        elif timeSinceIgnition > burnTime and self.rocket.orbitalVelocityReached == False and len(self.rocket.stages) == 1 and self.ThrustAfterBurnout == True:
            thrustMagnitude = self.motorEngineThrust

        else:
            thrustMagnitude = self.motorEngineThrust # set thrust to engine maximum if there is prop. and engine is on

        #TODO: Generate variable thrust condition?
        thrust = Vector(0,0,thrustMagnitude)
        self.rocket.appendToForceLogLine(" {:>10.4f}".format(thrust.Z))
        return ForceMomentSystem(thrust)

    # Function is called to update the ignition time if multiple stages are used
    def updateIgnitionTime(self, ignitionTime, fakeValue=False):
        gravity = 9.81
        massFlowProp = (self.motorEngineThrust/(gravity*self.motorISP))
        burnTime = self.motorMassPropTotal/massFlowProp

        self.ignitionTime = ignitionTime

        if not fakeValue:
            self.rocket.engineShutOffTime = self.ignitionTime + burnTime
            self.stage.engineShutOffTime = self.ignitionTime + burnTime # Engine Shuts off when the time exceeds burntime from when engines turn on.

    # Gets the log Header for the trhust value as a func. of time
    def getLogHeader(self):
        return " {}Thrust(N)".format(self.name)

# NOT NEEDED?: LEFT OVER FUNCTION FROM OTHER MOTOR CLASS
    # def _getMass(self, timeSinceIgnition):
    #     return self.OxWeight(timeSinceIgnition) + self.FuelWeight(timeSinceIgnition)

    # Function used to model oxydiser as its depleted. Returns CG, MOI, and Weight of Oxydiser
    def _getOxInertia(self, timeSinceIgnition):
        # TODO: Account for variable gravity
        gravity = 9.81
        initMassOxy = self.motorMassPropTotal/(1+(1/self.motorOxyFuelRatio))
        massFlowProp = (self.motorEngineThrust/(gravity*self.motorISP))
        massFlowOxy  = massFlowProp/(1+(1/self.motorOxyFuelRatio))

        # Sets mass of the oxydiser as the propellent is burned
        massOxy = initMassOxy-massFlowOxy*timeSinceIgnition 
        
        # Conditional Statement if Oxydiser Tank is Empty
        if massOxy < 0:
            massOxy = 0

        # Computes CG of Oxydiser as its depleted
        volumeOxy = massOxy/self.motorOxyDensity
        lengthOxy = volumeOxy/((math.pi/4)*self.motorStageDiameter**2) #Assumes cylindrical oxy tank with same diameter as bodytube
        oxCG_Z = self.finalOxCG_Z + lengthOxy/2
        oxCG = Vector(0,0,oxCG_Z)

        #MOI Calculations Assume Cylindrical Fuel Tank
        MOI_X = (1/4)*massOxy*(self.motorStageDiameter/2)**2 + (1/12)*massOxy*lengthOxy**2
        MOI_Y = MOI_X
        MOI_Z = 0.5*massOxy*(self.motorStageDiameter/2)**2
        oxMOI = Vector(MOI_X,MOI_Y,MOI_Z)

        # Sets Oxydiser weight to calcuated amount
        # TODO: Should this Value in kg??
        oxWeight = massOxy

        # TODO: Do we need to return a negative CG?
        return Inertia(oxMOI, oxCG, oxWeight)

    # Function used to model Fuel as its depleted. Returns CG, MOI, and Weight of Fuel
    def _getFuelInertia(self, timeSinceIgnition):
        gravity = 9.81

        # Mass Flow of Propellant Assuming Max Thrust and Constant ISP
        massFlowProp = (self.motorEngineThrust/(gravity*self.motorISP))
        
        #Fuel Information
        initMassFuel = self.motorMassPropTotal/(self.motorOxyFuelRatio+1)
        massFlowFuel  = massFlowProp/(self.motorOxyFuelRatio+1)

        # Computes the mass of fuel that is left in the tank
        massFuel = initMassFuel-massFlowFuel*timeSinceIgnition #Obtain current amount of oxydizer
        
        # Conditional statement if the tank becomes empty
        if massFuel < 0:
            massFuel = 0

        # Computes the CG of the Fuel as it gets Burned
        volumeFuel = massFuel/self.motorFuelDensity
        lengthFuel = volumeFuel/((math.pi/4)*self.motorStageDiameter**2) #Assumes cylindrical oxy tank with same diameter as bodytube
        fuelCG_Z = self.finalFuelCG_Z + lengthFuel/2
        fuelCG = Vector(0,0,fuelCG_Z)

        #MOI Calculations Assume Cylindrical Fuel Tank
        MOI_X = (1/4)*massFuel*(self.motorStageDiameter/2)**2 + (1/12)*massFuel*lengthFuel**2
        MOI_Y = MOI_X
        MOI_Z = 0.5*massFuel*(self.motorStageDiameter/2)**2
        fuelMOI = Vector(MOI_X,MOI_Y,MOI_Z)

        fuelWeight = massFuel

        return Inertia(fuelMOI, fuelCG, fuelWeight)

    def _getEngineInertia(self, timeSinceIgnition):     
        factor = 1
        motorCG_Z = self.motorEngineDiameter*factor + self.stage.bodyTubePosition.Z - self.stage.bodyTubeLength
        motorCG = Vector(0,0,motorCG_Z)

        #MOI Calculations Assume Cylindrical Fuel Tank
        MOI_X = (1/4)*self.motorEnginemass*(self.motorEngineDiameter/2)**2 + (1/12)*self.motorEnginemass*self.motorEngineDiameter**2
        MOI_Y = MOI_X
        MOI_Z = 0.5*self.motorEnginemass*(self.motorEngineDiameter/2)**2
        motorMOI = Vector(MOI_X,MOI_Y,MOI_Z)

        engineWeight = self.motorEnginemass

        return Inertia(motorMOI, motorCG, engineWeight)

class TabulatedMotor(RocketComponent):
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

    def getAeroForce(self, state, time , environment, CG):
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
