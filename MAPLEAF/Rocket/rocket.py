"""
`Rocket ties together the code in `MAPLEAF.GNC`, `MAPLEAF.Rocket`, `MAPLEAF.Motion`, and `MAPLEAF.ENV` to 
simulate the flight of a single rocket or dropped rocket stage (`Rocket` always only represents a single rigid body at a time).
New instances of `Rocket` are created by `MAPLEAF.SimulationRunners.Simulation` to represent dropped stages.

.. image:: https://media.defense.gov/2020/Apr/01/2002273784/780/780/0/200326-F-KD758-1012.JPG
"""

import math
import os

import matplotlib.pyplot as plt
from MAPLEAF.ENV import Environment, EnvironmentalConditions
from MAPLEAF.GNC import RocketControlSystem
from MAPLEAF.IO import Log, Logging, SubDictReader, TimeStepLog
from MAPLEAF.IO.HIL import HILInterface
from MAPLEAF.Motion import (AeroParameters, AngularVelocity, Inertia,
                            Quaternion, RigidBody, RigidBody_3DoF,
                            RigidBodyState, RigidBodyState_3DoF,
                            StatefulRigidBody, Vector)
from MAPLEAF.Rocket import (AeroFunctions, BoatTail, BodyComponent,
                            PlanarInterface, SimEventDetector, Stage,
                            initializeForceLogging)
from MAPLEAF.Rocket.CompositeObject import CompositeObject

__all__ = [ "Rocket" ]

class Rocket(CompositeObject):
    '''
        Class used to represent a single flying rigid body composed of `MAPLEAF.Rocket.Stage` objects.
        New instances of this class are also created to model the flight of dropped stages.
    '''
    #### Initialization ####
    def __init__(self, rocketDictReader, silent=False, stageToInitialize=None, simRunner=None, environment=None):
        '''
            Initialization of Rocket(s) is most easily completed through an instance of Simulation
            To get a single Rocket object, initialize a Simulation and call `MAPLEAF.SimulationRunners.Simulation.createRocket()`.  
            This will return a Rocket initialized on the pad with all its stages, ready for flight.

            If initializing manually, can either provide fileName or simDefinition. If a simDefinition is provided, it will be used and fileName will be ignored.

            Inputs:

            * rocketDictReader:     (`MAPLEAF.IO.SubDictReader`) SubDictReader pointed at the "Rocket" dictionary of the desired simulation definition file.  
            * silent:               (bool) controls console output  
            * stageToInitialize:    (int or None) controls whether to initialize a complete Rocket or a single (usually dropped) stage. None = initialize complete rocket. n = initialize only stage n, where n >= 1.  
            * simRunner:            (`MAPLEAF.SimulationRunners.Simulation`) reference to the current simulation driver/runner
            * environment:          (`MAPLEAF.ENV.Environment`) environment model from which the rocket will retrieve atmospheric properties and wind speeds
        '''
        self.rocketDictReader = rocketDictReader
        self.simDefinition = rocketDictReader.simDefinition

        self.simRunner = simRunner
        ''' Parent instance of `MAPLEAF.SimulationRunners.Simulation` (or derivative sim runner). This is usually the object that has created the current instance of Rocket. '''
        
        self.environment = environment
        ''' Instance of `MAPLEAF.ENV.Environment` '''
        if self.environment == None:
            # If no environment is passed in, create one
            self.environment = Environment(self.simDefinition, silent=silent)

        self.name = rocketDictReader.getString("name")

        self.silent = silent
        ''' Controls output to console '''

        self.stage = stageToInitialize
        ''' If controls whether the whole rocket is initialized (if == None), or a single stage is initialized (Integer stage number) '''

        self.stages = []
        '''
            A list of `MAPLEAF.Rocket.Stage` objects that make up the rocket, ordered from top to bottom.  
            Populated by `_initializeStages`.
        '''

        self.recoverySystem = None
        '''
            Reference to the current Rocket's (which can represent a dropped stage) recovery system. Only a single recovery system is allowed per stage.  
            Set in `MAPLEAF.Rocket.RecoverySystem.__init__`
        '''

        self.rigidBody = None            
        ''' 
            (`MAPLEAF.Motion.RigidBody` or `MAPLEAF.Motion.RigidBody_3DoF`) Responsible for motion integration.  
            Set in `_initializeRigidBody()`.
        '''

        self.isUnderChute = False
        ''' (bool) Controlled by `MAPLEAF.Rocket.Recovery.RecoverySystem._deployNextStage()` '''

        self.mainChuteDeployTime = None
        ''' (float) Filled in during flight by `MAPLEAF.Rocket.Recovery.RecoverySystem._deployNextStage()`  '''
        
        self.engineShutOffTime = None
        ''' (float) Filled in by `MAPLEAF.Rocket.Propulsion.TabulatedMotor.__init__()` upon initialization '''

        self.turbulenceOffWhenUnderChute = rocketDictReader.getBool("Environment.turbulenceOffWhenUnderChute")
        ''' (bool) '''

        self.maxDiameter = self._getMaxBodyTubeDiameter()     
        ''' (float) Holds maximum constant-size body tube diameter, from bodytube components in stages '''

        self.Aref = math.pi * self.maxDiameter**2 / 4
        ''' 
            Reference area for force and moment coefficients.
            Maximum rocket cross-sectional area. Remains constant during flight to retain a 1:1 relationship b/w coefficients in different parts of flight.
            Always based on the maximum body tube diameter in the fully-assembled rocket.
        '''
        
        # TODO: Remove
        self.targetLocation = None

        self.simEventDetector = SimEventDetector(self) 
        ''' (`MAPLEAF.Rocket.SimEventDetector`) Used to trigger things like recovery systems and staging '''

        self.eventTimeStep = rocketDictReader.getFloat("SimControl.TimeStepAdaptation.eventTimingAccuracy")
        ''' If using an adaptive time stepping method, the time step will be overridden near non-time-deterministic discrete events, possibly all the way down to this minimum value '''

        self.addZeroLengthBoatTailsToAccountForBaseDrag = rocketDictReader.getBool("Aero.addZeroLengthBoatTailsToAccountForBaseDrag")
        ''' Controls whether zero-length boat tails are automatically added to the bottom of rockets without them, to make sure base drag is accounted for '''

        self.fullyTurbulentBL = rocketDictReader.getBool("Aero.fullyTurbulentBL")
        ''' Controls whether skin friction is solved assuming a fully turbulent BL or using laminar/transitional flow at lower Reynolds numbers '''

        self.surfaceRoughness = rocketDictReader.getFloat("Aero.surfaceRoughness")
        ''' Default surface roughness for all rocket components '''

        self.finenessRatio = None
        ''' Used in some aerodynamic functions. Updated after initializing subcomponents, and throughout the flight. None if no BodyComponent(s) are present in the rocket '''
        
        self.engineShutOff = False
        '''Used to shut off engines in MAPLEAF.Rocket.Propulsion.DefinedMotor class. Currently set in MAPLE_AF.GNC.Navigation'''

        #### Init Hardware in the loop ####
        subDicts = rocketDictReader.getImmediateSubDicts()
        if "Rocket.HIL" in subDicts:
            self.hardwareInTheLoopControl = "yes"
            quatUpdateRate = rocketDictReader.getInt("HIL.quatUpdateRate")
            posUpdateRate = rocketDictReader.getInt("HIL.posUpdateRate")
            velUpdateRate = rocketDictReader.getInt("HIL.velUpdateRate")
            teensyComPort = rocketDictReader.getString("HIL.teensyComPort")
            imuComPort = rocketDictReader.getString("HIL.imuComPort")
            teensyBaudrate = rocketDictReader.getInt("HIL.teensyBaudrate")
            imuBaudrate = rocketDictReader.getInt("HIL.imuBaudrate")
            self.hilInterface = HILInterface(quatUpdateRate,posUpdateRate,velUpdateRate, teensyComPort, imuComPort, teensyBaudrate, imuBaudrate)
        else:
            self.hardwareInTheLoopControl = "no"

        #### Initialize Logging ####
        self.timeStepLog = None
        ''' Log containing one entry per time step, logs rocket state. None if logging level == 0 '''
        self.derivativeEvaluationLog = None
        ''' Log containing one entry per rocket motion derivative evaluation, contains component forces. None if logging level < 2 '''

        loggingLevel = int(self.simDefinition.getValue("SimControl.loggingLevel"))
        self.loggingLevel = loggingLevel

        if loggingLevel > 0:
            # Create the time step log and add columns to track the rocket state between each time step
            self.timeStepLog = TimeStepLog()
            zeroVector = Vector(0,0,0)
            self.timeStepLog.addColumn("Position(m)", zeroVector)
            self.timeStepLog.addColumn("Velocity(m/s)", zeroVector)
            self.timeStepLog.addColumn("OrientationQuaternion", Quaternion(0,0,0,0))
            self.timeStepLog.addColumn("EulerAngle(rad)", zeroVector)
            self.timeStepLog.addColumn("AngularVelocity(rad/s)", zeroVector)

            if "Adapt" in self.simDefinition.getValue("SimControl.timeDiscretization"):
                self.timeStepLog.addColumn("EstimatedIntegrationError", 0)

        if loggingLevel > 1:
            self.derivativeEvaluationLog = Log()
            zeroVector = Vector(0,0,0)

            self.derivativeEvaluationLog.addColumn("Position(m)", zeroVector)
            self.derivativeEvaluationLog.addColumn("Velocity(m/s)", zeroVector)
            self.derivativeEvaluationLog.addColumn("OrientationQuaternion", Quaternion(0,0,0,0))
            self.derivativeEvaluationLog.addColumn("AngularVelocity(rad/s)", zeroVector)

            self.derivativeEvaluationLog.addColumn("CG(m)", zeroVector)
            self.derivativeEvaluationLog.addColumn("MOI(kg*m^2)", zeroVector)
            self.derivativeEvaluationLog.addColumn("Mass(kg)", 0)
            
            self.derivativeEvaluationLog.addColumn("Wind(m/s)", zeroVector)
            self.derivativeEvaluationLog.addColumn("AirDensity(kg/m^3)", 0)
            self.derivativeEvaluationLog.addColumn("Mach", 0)
            self.derivativeEvaluationLog.addColumn("UnitRe", 0)
            self.derivativeEvaluationLog.addColumn("AOA(deg)", 0)
            self.derivativeEvaluationLog.addColumn("RollAngle(deg)", 0)

            self.derivativeEvaluationLog.addColumn("CPZ(m)", 0)
            self.derivativeEvaluationLog.addColumn("AeroF(N)", zeroVector)
            self.derivativeEvaluationLog.addColumn("AeroM(Nm)", zeroVector)
            self.derivativeEvaluationLog.addColumn("GravityF(N)", zeroVector)
            self.derivativeEvaluationLog.addColumn("TotalF(N)", zeroVector)
        
        #### Init Components ####
        self._createStages()
        self._initializeRigidBody()
        self._switchToStatefulRigidBodyIfRequired()
        self._sortStagesAndComponents()
        self._initializeStagingTriggers()
        self._precomputeComponentProperties()

        #### Init Parent classes ####
        CompositeObject.__init__(self, self.stages)
        self._updateFinenessRatio()

        # Adjust rigid body state position to correspond to the rocket's CG instead of nose cone tip position
        self._moveStatePositionToCG()

        #### Init Guidance/Navigation/Control System (if required) ####
        self.controlSystem = None
        ''' None for uncontrolled rockets. `MAPLEAF.GNC.ControlSystems.RocketControlSystem` for controlled rockets '''
        if ( rocketDictReader.tryGetString("ControlSystem.controlledSystem") != None or rocketDictReader.tryGetString("ControlSystem.MomentController.Type") == "IdealMomentController") and stageToInitialize == None:
            # Only create a control system if this is NOT a dropped stage
            ControlSystemDictReader = SubDictReader("Rocket.ControlSystem", simDefinition=self.simDefinition)
            controlSystemLogging = loggingLevel > 3
            self.controlSystem = RocketControlSystem(ControlSystemDictReader, self, log=controlSystemLogging, silent=silent)

    def _getMaxBodyTubeDiameter(self):
        ''' Gets max body tube diameter directly from config file '''
        stageDicts = self._getStageSubDicts()

        maxDiameter = 0
        for stageDict in stageDicts:
            componentDicts = self.rocketDictReader.getImmediateSubDicts(stageDict)
            
            for componentDict in componentDicts:
                className = self.rocketDictReader.getString(componentDict + ".class")
                
                if className == "Bodytube":
                    diameter = self.rocketDictReader.getFloat(componentDict + ".outerDiameter")
                    maxDiameter = max(maxDiameter, diameter)
        
        return maxDiameter

    def _getStageSubDicts(self):
        # Get all immediate subdictionaries of 'Rocket'
        stageDicts = self.rocketDictReader.getImmediateSubDicts()

        # Assume all subdictionaries represent rocket stages except for these exceptions
        nonStageSubDicts = [ "Rocket.ControlSystem", "Rocket.HIL", "Rocket.Aero" ]
        # Remove them from the list if they're in it
        for dictName in nonStageSubDicts:
            if dictName in stageDicts:
                stageDicts.remove(dictName)

        return stageDicts

    def _initializeRigidBody(self):
        #### Get initial kinematic state (in launch tower frame) ####
        initPos = self.rocketDictReader.getVector("position")
        initVel = self.rocketDictReader.getVector("velocity")
        
        # Check whether precise initial orientation has been specified
        rotationAxis = self.rocketDictReader.tryGetVector("rotationAxis", defaultValue=None)
        if rotationAxis != None:
            rotationAngle = math.radians(self.rocketDictReader.getFloat("rotationAngle"))
            initOrientation = Quaternion(rotationAxis, rotationAngle)
        else:
            # Calculate initial orientation quaternion in launch tower frame
            initialDirection = self.rocketDictReader.getVector("initialDirection").normalize()
            angleFromVertical = Vector(0,0,1).angle(initialDirection)
            rotationAxis = Vector(0,0,1).crossProduct(initialDirection)
            initOrientation = Quaternion(rotationAxis, angleFromVertical)

        initAngVel = AngularVelocity(rotationVector=self.rocketDictReader.getVector("angularVelocity"))
     
        initState_launchTowerFrame = RigidBodyState(initPos, initVel, initOrientation, initAngVel)

        # Convert to the global inertial frame
        initState_globalInertialFrame = self.environment.convertInitialStateToGlobalFrame(initState_launchTowerFrame)

        # Get desired time discretization method
        timeDisc = self.rocketDictReader.getString("SimControl.timeDiscretization")

        #TODO: Check for additional parameters to integrate - if they exist create a StatefulRigidBody + RocketState instead!
            # Ask each rocket component whether it would like to add parameters to integrate after all the components have been initialized

        discardDtCallback = None if (self.simRunner == None) else self.simRunner.discardForceLogsForPreviousTimeStep

        #### Initialize the rigid body ####
        self.rigidBody = RigidBody(
            initState_globalInertialFrame, 
            self._getAppliedForce, 
            self.getInertia, 
            integrationMethod=timeDisc, 
            discardedTimeStepCallback=discardDtCallback,
            simDefinition=self.simDefinition
        )

    def _createStages(self):
        ''' Initialize each of the stages and all of their subcomponents. '''
        stageDicts = self._getStageSubDicts()

        # If we're initializing a dropped stage, figure out which one
        if self.stage != None:
            stageNumbers = []
            stageNumberSet = set()
            for stageDict in stageDicts:
                stageNumber = self.rocketDictReader.getFloat(stageDict + ".stageNumber")
                stageNumbers.append(stageNumber)
                stageNumberSet.add(stageNumber)
            
            if len(stageNumbers) != len(stageNumberSet):
                raise ValueError("For multi-stage rockets, each stage must have a unique stage number. Set the Rocket.StageName.stageNumber value for each stage. 0 for first stage, 1 for second, etc...")
            
            stageNumbers.sort()
            stageToInitialize = stageNumbers[self.stage]

        # Initialize Stage(s)
        initializingAllStages = (self.stage == None)
        for stageDictionary in stageDicts:
            stageDictReader = SubDictReader(stageDictionary, self.simDefinition)
            stageNumber = stageDictReader.getFloat("stageNumber")

            if initializingAllStages or stageNumber == stageToInitialize:
                newStage = Stage(stageDictReader, self)
                self.stages.append(newStage)

                if newStage.name not in self.__dict__: # Avoid clobbering existing info
                    setattr(self, newStage.name, newStage) # Make stage available as rocket.stageName

    def _sortStagesAndComponents(self):
        # Create Planar Interfaces b/w components inside stages
        for stage in self.stages:
            stage.components = PlanarInterface.sortByZLocation(stage.components, self.rigidBody.state)
            stage.componentInterfaces = PlanarInterface.createPlanarComponentInterfaces(stage.components) 

        # Create planar interfaces b/w stages
        self.stages = PlanarInterface.sortByZLocation(self.stages, self.rigidBody.state)
        self.stageInterfaces = PlanarInterface.createPlanarComponentInterfaces(self.stages)

        if self.maxDiameter > 0:
            # Only run this if we're running a real rocket with body tubes
            self._ensureBaseDragIsAccountedFor()

    def _initializeStagingTriggers(self):
        ''' Set up trigger conditions for staging '''       
        # Self.stage is not passed in if the current instance represents a rocket ready to launch - then we have to set up staging events
        if self.stage == None:
            for stageIndex in range(len(self.stages)):
                stage = self.stages[stageIndex]

                if stage.separationConditionType != 'None':
                    self.simEventDetector.subscribeToEvent(
                        stage.separationConditionType, 
                        self._stageSeparation, 
                        stage.separationConditionValue, 
                        triggerDelay=stage.separationDelay
                    )
                
                if stageIndex != len(self.stages)-1:
                    # Set all upper stage motors to ignite a very long time in the future    
                    stage.motor.updateIgnitionTime(1000000000, fakeValue=True)

        else:
            # Otherwise this rocket object is representing a single dropped stage, and no stage separations are necessary
            # Motor is burned out
            self.stages[0].motor.updateIgnitionTime(-1000000000, fakeValue=True)
    
    def _switchToStatefulRigidBodyIfRequired(self):
        '''
            Query all of the rocket components to see if any of them are stateful by attempting to call their getExtraParametersToIntegrate function
            If the rocket contains stateful components, the rocket state is converted to a StateList and all of these state variables requested by components are added to it
            Otherwise, nothing changes and the rocket state remains a RigidBodyState
        '''
        varNames = []
        initVals = []
        derivativeFuncs = []

        # Query all of the components
        for stage in self.stages:
            for component in stage.components:
                try:
                    paramNames, initValues, derivativeFunctions = component.getExtraParametersToIntegrate()
                    varNames += paramNames
                    initVals += initValues
                    derivativeFuncs += derivativeFunctions
                except AttributeError:
                    pass # No extra parameters to integrate

        # If any of the components are stateful, keep track of their states in the main rocket state
        if len(varNames) > 0:
            if len(varNames) != len(initVals) or len(initVals) != len(derivativeFuncs):
                raise ValueError("ERROR: Mismatch in number of extra parameters names ({}), init values({}), and derivative functions({}) to integrate".format(len(varNames), len(initVals), len(derivativeFuncs)))
            
            # Switch to a stateful rigid body
            initState = self.rigidBody.state
            forceFunc = self.rigidBody.forceFunc
            inertiaFunc = self.rigidBody.inertiaFunc
            integrationMethod = self.rocketDictReader.getString("SimControl.timeDiscretization")
            discardDtCallback = None if (self.simRunner == None) else self.simRunner.discardForceLogsForPreviousTimeStep
            self.rigidBody = StatefulRigidBody(
                initState,
                forceFunc,
                inertiaFunc,
                integrationMethod=integrationMethod,
                discardedTimeStepCallback=discardDtCallback,
                simDefinition=self.simDefinition
            )

            # Add the additional state variables
            for i in range(len(varNames)):
                self.rigidBody.addStateVariable(varNames[i], initVals[i], derivativeFuncs[i])

    def _precomputeComponentProperties(self):
        for stage in self.stages:
            for component in stage.components:
                try:
                    component.precomputeProperties()
                except AttributeError:
                    pass

    def _moveStatePositionToCG(self):
        ''' Moves self.rigidBody.state.position to have it represent the rocket's initial CG position, not the initial nose cone position '''
        initInertia = self.getInertia(0, self.rigidBody.state)
        CGPosition_wrtNoseCone_localFrame = initInertia.CG
        CGPosition__wrtNoseCone_globalFrame = self.rigidBody.state.orientation.rotate(CGPosition_wrtNoseCone_localFrame)
        CGPosition_globalFrame = self.rigidBody.state.position + CGPosition__wrtNoseCone_globalFrame
        self.rigidBody.state.position = CGPosition_globalFrame

    def getLength(self):
        totalLength = 0
        for stage in self.stages:
            try:
                totalLength += stage.getLength()
            except TypeError:
                pass # Stage Length was None - no body components in stage

        return totalLength

    def plotShape(self):
        plt.figure()
        plt.gca().set_aspect('equal')

        rocketInertia = self.getInertia(0, self.rigidBody.state)
        TotalCG = rocketInertia.CG.Z
        TotalMass = rocketInertia.mass
        
        TotalCGplt = plt.plot(TotalCG, 0, color='b', marker='d', label='Total CG', linestyle='None')
        CGsubZ = []
        CGsubY = []

        plt.title('Total Rocket CG: %10.4f m \n Total Rocket Mass: %10.4f Kg' % (TotalCG,TotalMass) )

        for stage in self.stages:
            zCGs, yCGs = stage.plotShape()

            # Add subcomponents CGs to arrays
            CGsubZ += zCGs
            CGsubY += yCGs

        SubCGplt = plt.plot(CGsubZ, CGsubY, color='g', marker='.', label='Subcomponent CG', linestyle='None')
        legendHeight = self.maxDiameter
        plt.legend(loc='upper center', bbox_to_anchor = (0.5,-1.05))
        plt.show()

    #### Stage Separation ####
    def _stageSeparation(self):
        print("Stage {} Separation".format(self.simRunner.stagingIndex + 1))
        # Initialize dropped stage as a new rocket
        self.simRunner.createNewDetachedStage()

        # Drop stage from current rocket
        self._dropStage()
        
        # Ignite next motor (set ignition time to the time of stage separation)
        currentTime = self.rigidBody.time
        self.stages[0].motor.updateIgnitionTime(currentTime)

        self._ensureBaseDragIsAccountedFor()
        self._updateFinenessRatio()

    def _dropStage(self, stageIndex=-1):
        droppedStage = self.stages.pop(stageIndex)
        delattr(self, droppedStage.name)
        self.recomputeFixedMassInertia()

        if self.controlSystem != None:
            # Check whether the controlled system was part of the dropped stage
            if self.controlSystem.controlledSystem in droppedStage.components:
                # If so delete the rocket's control system and remove any control system-induced time stepping modifications
                print("Rocket's controlled system was on the dropped stage. Deactivating control system.")
                self.controlSystem.restoreOriginalTimeStepping()
                self.controlSystem = None

    def _ensureBaseDragIsAccountedFor(self):
        ''' If no BoatTail exists at the bottom of the rocket, adds a zero-length boat tail. This is necessary b/c Boat Tail aero-functions are the ones that account for base drag '''
        boatTailComponentAtBottomOfRocket = False
        bottomStage = self.stages[-1]
        for comp in reversed(bottomStage.components):
            if isinstance(comp, BodyComponent):
                if isinstance(comp, BoatTail):
                    boatTailComponentAtBottomOfRocket = True
                break
        
        if not boatTailComponentAtBottomOfRocket and self.addZeroLengthBoatTailsToAccountForBaseDrag:
            if not self.silent:
                print("Adding zero-length BoatTail to the bottom of current bottom stage ({}) to account for base drag".format(bottomStage.name))
            # Create a zero-length, zero-mass boat tail to account for base drag
            zeroInertia = Inertia(Vector(0,0,0), Vector(0,0,0), 0)
            diameter = self.maxDiameter # TODO: Get the actual bottom-body-tube diameter from a future Stage.getRadius function
            length = 0
            position = bottomStage.getBottomInterfaceLocation()
            boatTail = BoatTail(
                diameter, 
                diameter, 
                length, 
                position, 
                zeroInertia, 
                self, 
                bottomStage, 
                "Auto-AddedZeroLengthBoatTail", 
                self.surfaceRoughness
            )
            initializeForceLogging(boatTail, "FakeRocketName.Auto-AddedZeroLengthBoatTail", self)
            bottomStage.components.append(boatTail)     

    def _updateFinenessRatio(self):
        ''' Updates self.finenessRatio based on current BodyComponents in rocket stages '''
        length = self.getLength()
        maxDiameter = max([ stage.getMaxDiameter() for stage in self.stages ])
        if maxDiameter == 0:
            self.finenessRatio = None
        else:
            self.finenessRatio = length/maxDiameter

    #### Component-buildup method for Force ####
    def _getEnvironmentalConditions(self, time, state):
        env = self.environment.getAirProperties(state.position, time)

        # Neglect turbulent component of wind if required
        if self.isUnderChute and self.turbulenceOffWhenUnderChute:
            env = EnvironmentalConditions(
                env.ASLAltitude,
                env.Temp,
                env.Pressure,
                env.Density,
                env.DynamicViscosity,
                env.MeanWind,
                env.MeanWind,
                Vector(0, 0, 0),
            )
        
        return env

    def _getAppliedForce(self, time, state):
        ''' Get the total force currently being experienced by the rocket, used by self.rigidBody to calculate the rocket's acceleration '''
        ### Precomputations and Logging ###
        environment = self._getEnvironmentalConditions(time, state)               # Get and log current air/wind properties
        rocketInertia = self.getInertia(time, state)                                            # Get and log current rocket inertia

        if self.derivativeEvaluationLog is not None:
            self.derivativeEvaluationLog.newLogRow(time)
            self.derivativeEvaluationLog.logValue("Position(m)", state.position)
            self.derivativeEvaluationLog.logValue("Velocity(m/s)", state.velocity)

        ### Component Forces ###
        if not self.isUnderChute:            
            # Precompute and log
            Mach = AeroParameters.getMachNumber(state, environment)
            unitRe = AeroParameters.getReynoldsNumber(state, environment, 1.0)
            AOA = AeroParameters.getTotalAOA(state, environment)
            rollAngle = AeroParameters.getRollAngle(state, environment)

            if self.derivativeEvaluationLog is not None:
                self.derivativeEvaluationLog.logValue("Mach", Mach)
                self.derivativeEvaluationLog.logValue("UnitRe", unitRe)
                self.derivativeEvaluationLog.logValue("AOA(deg)", math.degrees(AOA))
                self.derivativeEvaluationLog.logValue("RollAngle(deg)", rollAngle)
                self.derivativeEvaluationLog.logValue("OrientationQuaternion", state.orientation)
                self.derivativeEvaluationLog.logValue("AngularVelocity(rad/s)", state.angularVelocity)

            # This function will be the inherited function CompositeObject.getAppliedForce
            componentForces = self.getAppliedForce(state, time, environment, rocketInertia.CG) 

        else:
            # When under chute, neglect forces from other components
            componentForces = self.recoverySystem.getAppliedForce(state, time, environment, Vector(0,0,-1))
            
            # Log the recovery system's applied force (Normally handled in CompositeObject.getAppliedForce)
            if hasattr(self.recoverySystem, "forcesLog"):
                self.recoverySystem.forcesLog.append(componentForces.force)
                self.recoverySystem.momentsLog.append(componentForces.moment)
    
        # Move Force-Moment system to rocket CG
        componentForces = componentForces.getAt(rocketInertia.CG)
           
        ### Gravity ###
        gravityForce = self.environment.getGravityForce(rocketInertia, state)        
        totalForce = componentForces + gravityForce

        ### Launch Rail ###
        totalForce = self.environment.applyLaunchTowerForce(state, time, totalForce)        

        if self.derivativeEvaluationLog is not None:
            self.derivativeEvaluationLog.logValue("Wind(m/s)", environment.Wind)
            self.derivativeEvaluationLog.logValue("AirDensity(kg/m^3)", environment.Density)
            self.derivativeEvaluationLog.logValue("CG(m)", rocketInertia.CG)
            self.derivativeEvaluationLog.logValue("MOI(kg*m^2)", rocketInertia.MOI)
            self.derivativeEvaluationLog.logValue("Mass(kg)", rocketInertia.mass)

            CPZ = AeroFunctions._getCPZ(componentForces)            
            self.derivativeEvaluationLog.logValue("CPZ(m)", CPZ)
            self.derivativeEvaluationLog.logValue("AeroF(N)", componentForces.force)
            self.derivativeEvaluationLog.logValue("AeroM(Nm)", componentForces.moment)
            self.derivativeEvaluationLog.logValue("GravityF(N)", gravityForce.force)
            self.derivativeEvaluationLog.logValue("TotalF(N)", totalForce.force)

        return totalForce
    
    #### Driving / Controlling Simulation ####
    def _logState(self):
        '''
            Logs the initial state of the rocket to the time step log
        '''
        state = self.rigidBody.state
        time = self.rigidBody.time

        if self.timeStepLog is not None:
            self.timeStepLog.newLogRow(time)
            self.timeStepLog.logValue("Position(m)", state.position)
            self.timeStepLog.logValue("Velocity(m/s)", state.velocity)

            try: # 6DoF Mode
                self.timeStepLog.logValue("OrientationQuaternion", state.orientation)
                self.timeStepLog.logValue("AngularVelocity(rad/s)", state.angularVelocity)

                # Also log NED Tait-Bryan 3-2-1 z-y-x Euler Angles if in 6DoF mode
                globalOrientation = state.orientation
                orientationOfNEDFrameInGlobalFrame = self.environment.earthModel.getInertialToNEDFrameRotation(*state.position)
                orientationRelativeToNEDFrame = orientationOfNEDFrameInGlobalFrame.conjugate() * globalOrientation
                eulerAngles = orientationRelativeToNEDFrame.toEulerAngles()
                self.timeStepLog.logValue("EulerAngle(rad)", eulerAngles)

            except AttributeError:
                pass # 3DoF mode

        # Print the current time and altitude to the console
        altitude = self.environment.earthModel.getAltitude(*state.position)
        consoleOutput = "{:<8.4f} {:>6.5f}".format(time, altitude)
        print(consoleOutput)

    def _runControlSystem(self):
        '''
            Attempts to run the rocket control system (only runs if it's time to run again, based on its updated rate) (updating target positions for actuators) 
        '''
        if self.controlSystem != None and not self.isUnderChute:
            state = self.rigidBody.state
            time = self.rigidBody.time
            environment = self._getEnvironmentalConditions(time, state)

            ### Run Control Loop ###
            self.controlSystem.runControlLoopIfRequired(time, state, environment)        

    def timeStep(self, dt: float):
        '''
            Tells the simulation to take a time step of size dt.  

            Usually called by functions like `MAPLEAF.SimulationRunners.Simulation.run()`

            Returns:
                * timeStepAdaptationFactor: (float) indicates the factor by which the adaptive time stepping method would like to change the timestep for the next timestep (1.0 for non-adaptive methods)
                * dt:                       (float) actual size of time step taken. Adaptive methods will override the dt asked for if the predicted error for a time step is over their error threshold.
        '''
        # Stop the rocket from sliding off the bottom of the launch rail
        self.rigidBody.state = self.environment.applyLaunchRailMotionConstraints(self.rigidBody.state, self.rigidBody.time)
        
        # Trigger any events that occurred during the last time step
        estimatedTimeToNextEvent, accuratePrediction = self.simEventDetector.triggerEvents()

        # If required, override time step to accurately resolve upcoming discrete events
        if "Adapt" in self.rigidBody.integrate.method and estimatedTimeToNextEvent < dt:
            if accuratePrediction:
                # For time-deterministic events, just set the time step to ever so slightly past the event
                newDt = estimatedTimeToNextEvent + 1e-5
                print("Rocket + SimEventDetector overriding time step from {} to {} to accurately trigger resolve time-deterministic event.".format(dt, newDt))
                dt = newDt
            else:
                # For time-nondeterministic events, slowly approach the event
                newDt = max(estimatedTimeToNextEvent/1.5, self.eventTimeStep)
                estimatedOccurenceTime = self.rigidBody.time + estimatedTimeToNextEvent
                print("Rocket + SimEventDetector overriding time step from {} to {} to accurately resolve upcoming event. Estimated occurence at: {}".format(dt, newDt, estimatedOccurenceTime))
                dt = newDt
                
        self._runControlSystem()
        self._logState()

        # Take timestep
        integrationResult = self.rigidBody.timeStep(dt)

        # If required, log estimated integration error
        if "Adapt" in self.rigidBody.integrate.method and self.timeStepLog is not None:
            self.timeStepLog.logValue("EstimatedIntegrationError", integrationResult.errorMagEstimate)
            
        return integrationResult
    
    def _switchTo3DoF(self):
        ''' Switch to 3DoF simulation after recovery system is deployed '''
        print("Switching to 3DoF simulation")
        new3DoFState = RigidBodyState_3DoF(self.rigidBody.state.position, self.rigidBody.state.velocity)
        
        # Re-read time discretization in case an adaptive method has been selected while using a fixed-update rate control system - in that case, want to switch back to adaptive time stepping for the recovery (uncontrolled) portion of the flight
        originalTimeDiscretization = self.rocketDictReader.getString("SimControl.timeDiscretization")

        if self.simRunner != None:
            self.rigidBody = RigidBody_3DoF(
                new3DoFState, 
                self._getAppliedForce, 
                self.getMass, 
                startTime=self.rigidBody.time, 
                integrationMethod=originalTimeDiscretization, 
                discardedTimeStepCallback=self.simRunner.discardForceLogsForPreviousTimeStep, 
                simDefinition=self.simDefinition
            )
        else:
            self.rigidBody = RigidBody_3DoF(
                new3DoFState, 
                self._getAppliedForce, 
                self.getMass, 
                startTime=self.rigidBody.time, 
                integrationMethod=originalTimeDiscretization, 
                simDefinition=self.simDefinition
            )

    #### After Simulation ####
    def writeLogsToFile(self, directory="."):
        logfilePaths = []

        if self.timeStepLog is not None:
            rocketName = self.components[0].name # Rocket is named after its top stage
            path = os.path.join(directory, "{}_timeStepLog.csv".format(rocketName))
            
            # Time step log
            if self.timeStepLog.writeToCSV(path):
                logfilePaths.append(path)

            # Derivative evaluation log
            if self.derivativeEvaluationLog is not None:
                path = os.path.join(directory, "{}_derivativeEvaluationLog.csv".format(rocketName))  
                
                if self.derivativeEvaluationLog.writeToCSV(path):
                    logfilePaths.append(path)

                    # Calculate aerodynamic coefficients if desired
                    if self.loggingLevel > 2:
                        expandedLogPath = Logging.postProcessForceEvalLog(path, refArea=self.Aref, refLength=self.maxDiameter)
                        logfilePaths.append(expandedLogPath)

            # Control system log
            if self.controlSystem != None:
                if self.controlSystem.log != None:
                    controlSystemLogPath = self.controlSystem.writeLogsToFile(directory)
                    logfilePaths.append(controlSystemLogPath)

        return logfilePaths