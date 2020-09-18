"""
`Rocket ties together the code in `MAPLEAF.GNC`, `MAPLEAF.Rocket`, `MAPLEAF.Motion`, and `MAPLEAF.ENV` to 
simulate the flight of a single rocket or dropped rocket stage (`Rocket` always only represents a single rigid body at a time).
New instances of `Rocket` are created by `MAPLEAF.SimulationRunners.SingleSimRunner` to represent dropped stages.

.. image:: https://media.defense.gov/2020/Apr/01/2002273784/780/780/0/200326-F-KD758-1012.JPG
"""

import math

import matplotlib.pyplot as plt

from MAPLEAF.ENV import Environment, EnvironmentalConditions
from MAPLEAF.GNC import RocketControlSystem
from MAPLEAF.IO import SubDictReader
from MAPLEAF.IO.HIL import HILInterface
from MAPLEAF.Motion import (AeroParameters, AngularVelocity, Inertia,
                            Quaternion, RigidBody, RigidBody_3DoF,
                            RigidBodyState, RigidBodyState_3DoF, Vector)
from MAPLEAF.Rocket import (AeroFunctions, BoatTail, BodyComponent,
                            PlanarInterface, SimEventDetector, Stage)
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
            Initialization of Rocket(s) is most easily completed through an instance of SingleSimRunner
            To get a single Rocket object, initialize a SingleSimRunner and call `MAPLEAF.SimulationRunners.SingleSimRunner.prepRocketForSingleSimulation()`.  
            This will return a Rocket initialized on the pad with all its stages, ready for flight.

            If initializing manually, can either provide fileName or simDefinition. If a simDefinition is provided, it will be used and fileName will be ignored.

            Inputs:

            * rocketDictReader:     (`MAPLEAF.IO.SubDictReader`) SubDictReader pointed at the "Rocket" dictionary of the desired simulation definition file.  
            * silent:               (bool) controls console output  
            * stageToInitialize:    (int or None) controls whether to initialize a complete Rocket or a single (usually dropped) stage. None = initialize complete rocket. n = initialize only stage n, where n >= 1.  
            * simRunner:            (`MAPLEAF.SimulationRunners.SingleSimRunner`) reference to the current simulation driver/runner
            * environment:          (`MAPLEAF.ENV.Environment`) environment model from which the rocket will retrieve atmospheric properties and wind speeds
        '''
        self.rocketDictReader = rocketDictReader
        self.simDefinition = rocketDictReader.simDefinition

        self.simRunner = simRunner
        ''' Parent instance of `MAPLEAF.SimulationRunners.SingleSimRunner` (or derivative sim runner). This is usually the object that has created the current instance of Rocket. '''
        
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

        self.bodyTubeDiameter = self._getMaxBodyTubeDiameter()     
        ''' (float) Holds maximum constant-size body tube diameter, from bodytube components in stages '''

        self.Aref = self.bodyTubeDiameter**2 / 4
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

        #### Init Components ####
        self._initializeRigidBody()
        self._initializeStages()
        self._initializeStagingTriggers()

        #### Init Parent classes ####
        CompositeObject.__init__(self, self.stages)
        self._updateFinenessRatio()

        # Adjust rigid body state position to correspond to the rocket's CG instead of nose cone tip position
        self._moveStatePositionToCG()

        #### Init Guidance/Navigation/Control System (if required) ####
        self.controlSystem = None
        ''' None for uncontrolled rockets. `MAPLEAF.GNC.ControlSystems.RocketControlSystem` for controlled rockets '''
        if rocketDictReader.tryGetString("ControlSystem.controlledSystem") != None and stageToInitialize == None:
            # Only create a control system if this is NOT a dropped stage
            ControlSystemDictReader = SubDictReader("Rocket.ControlSystem", simDefinition=self.simDefinition)
            self.controlSystem = RocketControlSystem(ControlSystemDictReader, self)

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
                    if diameter > maxDiameter:
                        maxDiameter = diameter
        
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

        #### Initialize the rigid body ####
        if self.simRunner != None:
            self.rigidBody = RigidBody(
                initState_globalInertialFrame, 
                self._getAppliedForce, 
                self.getInertia, 
                integrationMethod=timeDisc, 
                discardedTimeStepCallback=self.simRunner.discardForceLogsForLastTimeStep, 
                simDefinition=self.simDefinition
            )
        else:
            self.rigidBody = RigidBody(
                initState_globalInertialFrame, 
                self._getAppliedForce, 
                self.getInertia, 
                integrationMethod=timeDisc, 
                simDefinition=self.simDefinition
            )

    def _initializeStages(self):
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

        self.stages = PlanarInterface.sortByZLocation(self.stages)
        self.stageInterfaces = PlanarInterface.createPlanarComponentInterfaces(self.stages)

        if self.bodyTubeDiameter > 0:
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
        legendHeight = self.bodyTubeDiameter
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
            diameter = self.bodyTubeDiameter # TODO: Get the actual bottom-body-tube diameter from a future Stage.getRadius function
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
            bottomStage.components.append(boatTail)     

    def _updateFinenessRatio(self):
        ''' Updates self.finenessRatio based on current BodyComponents in rocket stages '''
        length = self.getLength()
        maxDiameter = max([ stage.getMaxDiameter() for stage in self.stages ])
        if maxDiameter == 0:
            self.finenessRatio = None
        else:
            self.finenessRatio = length/maxDiameter

    #### Logging ####
    def appendToForceLogLine(self, txt: str):
        ''' Appends txt to the current line of the parent `MAPLEAF.SimulationRunners.SingleSimRunner`'s forceEvaluationLog '''
        try:
            self.simRunner.forceEvaluationLog[-1] += txt
        except AttributeError:
            pass # Force logging not set up for this sim

    #### Component-buildup method for Force ####
    def _getEnvironmentalConditions(self, time, state, logWind=True):
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

        # Log Wind
        if logWind:
            self.appendToForceLogLine(" {:>10.4f} {:>7.4}".format(env.Wind, env.Density))
        
        return env

    def _getAppliedForce(self, time, state):
        ''' Get the total force currently being experienced by the rocket, used by self.rigidBody to calculate the rocket's acceleration '''
        self.simRunner.newForcesLogLine("{:<7.3f} ".format(time) + str(state))                  # Start a new line in the force Evaluation log
        
        environment = self._getEnvironmentalConditions(time, state, logWind=True)   # Get and log current air/wind properties
        
        rocketInertia = self.getInertia(time, state)                                            # Get and log current rocket inertia
        self.appendToForceLogLine(" {:>10.4f} {:>10.8f} {:>10.8f}".format(rocketInertia.CG, rocketInertia.mass, rocketInertia.MOI))
        
        ### Component Forces ###
        if not self.isUnderChute:
            # Precompute CG, AOA, roll angle, normal force direction, localFrameAirVel, Ma, UnitRe
            Mach = AeroParameters.getMachNumber(state, environment)
            unitRe = AeroParameters.getReynoldsNumber(state, environment, 1.0)
            AOA = AeroParameters.getTotalAOA(state, environment)
            rollAngle = AeroParameters.getRollAngle(state, environment)

            # Log current rocket / flight conditions
            self.appendToForceLogLine(" {:>10.4f} {:>10.0f} {:>10.4f} {:>10.4f}".format(Mach, unitRe, math.degrees(AOA), rollAngle))

            # This function will be the inherited function CompositeObject.getAeroForce
            componentForces = self.getAeroForce(state, time, environment, rocketInertia.CG) 

        else:
            # When under chute, neglect forces from other components
            componentForces = self.recoverySystem.getAeroForce(state, time, environment, Vector(0,0,-1))

        componentForces = componentForces.getAt(rocketInertia.CG) # Move Force-Moment system to rocket CG

        # Compute and log center of pressure z-location
        self.appendToForceLogLine(" {:>10.5f}".format(AeroFunctions._getCPZ(componentForces)))
           
        ### Gravity ###
        gravityForce = self.environment.getGravityForce(rocketInertia, state)        
        totalForce = componentForces + gravityForce

        ### Launch Rail ###
        totalForce = self.environment.applyLaunchTowerForce(state, time, totalForce)

        self.appendToForceLogLine(" {:>8.5f} {:>8.6f} {:>8.4f} {:>8.4f}".format(
            componentForces.force, componentForces.moment, gravityForce.force, totalForce.force)
        )

        return totalForce
    
    #### Driving / Controlling Simulation ####
    def _runControlSystemAndLogStartingState(self, dt):
        startState = self.rigidBody.state
        startTime = self.rigidBody.time       

        # Control loop (if applicable)
        controlLogString = "" # Empty string if not overridden by running the control loop
        if self.controlSystem != None and not self.isUnderChute:
            environment = self._getEnvironmentalConditions(startTime, startState, logWind=False)

            ### Run Control Loop ###
            newTargetActuatorDeflections = self.controlSystem.runControlLoopIfRequired(startTime, startState, environment)
            
            if newTargetActuatorDeflections != False:
                # runControlLoopIfRequired returns False if the control doesn't run
                # Add target deflections to the main log string
                deflStrings = [ " {:>7.3f}".format(defl) for defl in newTargetActuatorDeflections ]
                controlLogString = "".join(deflStrings)

        # Log NED Tait-Bryan 3-2-1 z-y-x Euler Angles if in 6DoF mode
        try: # 6DoF Mode
            globalOrientation = startState.orientation
            orientationOfNEDFrameInGlobalFrame = self.environment.earthModel.getInertialToNEDFrameRotation(*startState.position)
            orientationRelativeToNEDFrame = orientationOfNEDFrameInGlobalFrame.conjugate() * globalOrientation
            eulerAngles = orientationRelativeToNEDFrame.toEulerAngles()
            eulerAnglesString =  " {:>10.6f}".format(eulerAngles)
        except AttributeError:
            eulerAnglesString = "" # 3DoF mode

        # Log -> Main Sim Log
        mainLogString = "{:<8.4f} {:>6.5f}".format(startTime, dt) + str(startState) + eulerAnglesString + controlLogString
        print(mainLogString)

    def timeStep(self, dt: float):
        '''
            Tells the simulation to take a time step of size dt.  

            Usually called by functions like `MAPLEAF.SimulationRunners.SingleSimRunner.runSingleSimulation()`

            Returns:
                * timeStepAdaptationFactor: (float) indicates the factor by which the adaptive time stepping method would like to change the timestep for the next timestep (1.0 for non-adaptive methods)
                * dt:                       (float) actual size of time step taken. Adaptive methods will override the dt asked for if the predicted error for a time step is over their error threshold.
        '''
        # Stop the rocket from sliding off the bottom of the launch rail
        self.rigidBody.state = self.environment.applyLaunchRailMotionConstraints(self.rigidBody.state, self.rigidBody.time)
        
        # Trigger any events that occurred during the last time step
        estimatedTimeToNextEvent, accuratePrediction = self.simEventDetector.triggerEvents()

        if "Adapt" in self.rigidBody.integrate.method and estimatedTimeToNextEvent < dt:
            # Override time step to accurately resolve discrete events
            if accuratePrediction:
                # For time-deterministic events, just set the time step to ever so slightly past the event
                dt = estimatedTimeToNextEvent + 1e-5
            else:
                # For time-nondeterministic events, slowly approach the event
                dt = max(estimatedTimeToNextEvent/2, self.eventTimeStep)
                
        # Logs line to mainSimLog
        self._runControlSystemAndLogStartingState(dt)

        # Take timestep
        timeStepAdjustmentFactor, dt = self.rigidBody.timeStep(dt)

        # Return time step adaptation factor - will be 1 for constant time stepping
        return timeStepAdjustmentFactor, dt
    
    def _switchTo3DoF(self):
        ''' Switch to 3DoF simulation after recovery system is deployed '''
        print("Switching to 3DoF model for descent")
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
                discardedTimeStepCallback=self.simRunner.discardForceLogsForLastTimeStep, 
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
