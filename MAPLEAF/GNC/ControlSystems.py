''' 
Classes that implement air vehicle control systems by tying together a`MAPLEAF.GNC.Navigation.Navigator`, a `MAPLEAF.GNC.MomentControllers.MomentController`, and an `MAPLEAF.GNC.Actuators.ActuatorController`.  
Control systems run a simulated control loops between simulation time steps, and then provide new target actuator deflections to the `MAPLEAF.GNC.ActuatedSystem` they control.
'''

import abc
import numpy as np

from MAPLEAF.GNC import ConstantGainPIDRocketMomentController, ScheduledGainPIDRocketMomentController, Stabilizer
from MAPLEAF.IO import SubDictReader
from MAPLEAF.Motion import integratorFactory

__all__ = [ "RocketControlSystem", "ControlSystem" ]

class ControlSystem(abc.ABC):
    ''' Interface for control systems '''

    @abc.abstractmethod
    def runControlLoopIfRequired(self, currentTime, rocketState, environment):
        ''' 
            Should check if the control loop needs to be run and run it if so. 
            Should return a list of actuator deflections (for logging)

            Reference to controlled system expected to be self.controlledSystem
            Controlled system expected to inherit from ActuatedSystem

            If the control system ran:
                Should return an iterable of new actuator position targets (in order consistent with log headers)
            If the control system didn't run:
                Should return False
        '''

    @abc.abstractmethod
    def getLogHeader(self):
        ''' Should return the headers (in order) for each of the actuator deflections in the list returned by self.runControlLoopIfRequired '''

    def calculateAngularError(self, currentOrientation, targetOrientation):

        errors = np.array((targetOrientation / currentOrientation).toRotationVector())
        pitchError = errors[0]
        yawError = errors[1]
        rollError = errors[2]

        return pitchError, yawError, rollError

class RocketControlSystem(ControlSystem, SubDictReader):
    ''' Simplest possible control system for a rocket '''

    def __init__(self, controlSystemDictReader, rocket, initTime=0):
        self.rocket = rocket
        self.controlSystemDictReader = controlSystemDictReader
        self.lastControlLoopRunTime = initTime
        self.timeSteppingModified = False # set to True if the sim's time stepping has been constrained by the control system update rate (in self._checkSimTimeStepping)

        ### Create Navigator ###
        desiredFlightDirection = controlSystemDictReader.getVector("desiredFlightDirection")
        self.navigator = Stabilizer(desiredFlightDirection)

        ### Create Moment Controller ###
        momentControllerType = controlSystemDictReader.getString("MomentController.Type")
        if momentControllerType == "ConstantGainPIDRocket":
            Pxy = controlSystemDictReader.getFloat("MomentController.Pxy")
            Ixy = controlSystemDictReader.getFloat("MomentController.Ixy")
            Dxy = controlSystemDictReader.getFloat("MomentController.Dxy")
            Pz = controlSystemDictReader.getFloat("MomentController.Pz")
            Iz = controlSystemDictReader.getFloat("MomentController.Iz")
            Dz = controlSystemDictReader.getFloat("MomentController.Dz")
            self.momentController = ConstantGainPIDRocketMomentController(Pxy,Ixy,Dxy,Pz,Iz,Dz)
        elif momentControllerType == "ScheduledGainPIDRocket":
            gainTableFilePath = controlSystemDictReader.getString("MomentController.gainTableFilePath")
            keyColumnNames = controlSystemDictReader.getString("MomentController.scheduledBy").split()
            self.momentController = ScheduledGainPIDRocketMomentController(gainTableFilePath, keyColumnNames)
        else:
            raise ValueError("Moment Controller Type: {} not implemented. Try ScheduledGainPIDRocket".format(momentControllerType))

        ### Set update rate ###
        self.updateRate = controlSystemDictReader.getFloat("updateRate")

        ### Get reference to the controlled system ###
        self.controlledSystems = {}
        stageNumbers = controlSystemDictReader.getImmediateSubKeys("controlledSystems")
        for stageNumberKey in stageNumbers:
            controlledSystemPath = controlSystemDictReader.getString(stageNumberKey)
            stageNumber = int(stageNumberKey[stageNumberKey.rfind(".")+1:])
            
            # Identify the controlled system
            for stage in rocket.stages:
                for rocketComponent in stage.components:
                    if rocketComponent.componentDictReader.simDefDictPathToReadFrom == controlledSystemPath:
                        self.controlledSystems[stageNumber] = rocketComponent
                        break

            # Output an error message if we couldn't find it
            if stageNumber not in self.controlledSystems:
                simDefinitionFile = controlSystemDictReader.simDefinition.fileName
                raise ValueError("Rocket Component: {} not found in {}".format(controlledSystemPath, simDefinitionFile))

            # Create the actuators
            self.controlledSystems[stageNumber].initializeActuators(self)

        currentStageNumber = min(self.controlledSystems.keys()) # Smallest number will be the first stage number
        self.controlledSystem = self.controlledSystems[currentStageNumber]

        self._checkSimTimeStepping()

    def _checkSimTimeStepping(self):
        # If the update rate is zero, control system is simply run once per time step
        if self.updateRate != 0:
            # Since a discrete update rate has been specified, we need to ensure that the simulation time step is an integer divisor of the control system time step
            
            # Disable adaptive time stepping during the ascent portion of the flight (if it's enabled)
            timeDiscretization = self.controlSystemDictReader.getString("SimControl.timeDiscretization")
            if "Adaptive" in timeDiscretization:
                print("Time stepping conflict between adaptive-time-stepping runge-kutta method and fixed control system update rate")
                print("Disabling adaptive time stepping for ascent portion of flight. Will re-enable if/when recovery system deploys.")
                print("Switching to RK4 time stepping")
                self.rocket.rigidBody.integrate = integratorFactory(integrationMethod='RK4')
                self.timeSteppingModified = True

            # Adjust the sim time step to be an even divisor of the control system time step
            controlTimeStep = 1/self.updateRate
            self.controlTimeStep = controlTimeStep
            originalSimTimeStep = self.controlSystemDictReader.getFloat("SimControl.timeStep")
            self.originalSimTimeStep = originalSimTimeStep
            if controlTimeStep / originalSimTimeStep != round(controlTimeStep / originalSimTimeStep):
                print("WARNING: Selected time step: {} does not divide the control system time step: {} Into an integer number of time steps.".format(originalSimTimeStep, controlTimeStep))
                recommendedSimTimeStep = controlTimeStep / max(1, round(controlTimeStep / originalSimTimeStep))
                
                print("Adjusting time step to: {}".format(recommendedSimTimeStep))
                print("")
                # This relies on the SimRunner reading the time step size from the sim definition after intializing the rocket
                    # To make this more robust, have the rocket perform a timestep size check every single time step
                self.controlSystemDictReader.simDefinition.setValue("SimControl.timeStep", str(recommendedSimTimeStep))
                self.timeSteppingModified = True
            
        else:
            self.controlTimeStep = 0

    def restoreOriginalTimeStepping(self):
        '''
            Should get called whenever the control system is disabled:
                1. If a stage containing the controlled system is dropped
                2. If the rocket recovery system is deployed
        '''
        if self.timeSteppingModified:
            # This currently won't do anything because the time step is saved in an array by the Simulation - might be useful in the future
            self.controlSystemDictReader.simDefinition.setValue("SimControl.timeStep", str(self.originalSimTimeStep))

            # This will re-initialize the integrator to match that originally selected in the sim definition file
            originalTimeIntegrationMethod = self.controlSystemDictReader.getString("SimControl.timeDiscretization")
            print("Restoring original time discretization method ({})".format(originalTimeIntegrationMethod))
            self.rocket.rigidBody.integrate = integratorFactory(integrationMethod=originalTimeIntegrationMethod, simDefinition=self.controlSystemDictReader.simDefinition)

    def controlNextStageAfterSeparation(self):
        # Delete reference to controlled system from previous stage
        droppedStageNumber = min(self.controlledSystems.keys()) # Smallest number will be current bottom stage number
        del self.controlledSystems[droppedStageNumber]

        # Update stage number to the next one
        if len(self.controlledSystems.keys()) > 0:
            # Update to next one
            nextStageNumber = min(self.controlledSystems.keys()) # Smallest number will be current bottom stage number
            self.controlledSystem = self.controlledSystems[nextStageNumber]
            for actuator in self.controlledSystem.actuatorList:
                actuator.lastTime = self.rocket.rigidBody.time
        else:
            # If no control system on next stage, delete itself
            self.deactivate()

    def deactivate(self):
        self.restoreOriginalTimeStepping()
        self.rocket.controlSystem = None

    def runControlLoopIfRequired(self, currentTime, rocketState, environment):
        # Run control loop if enough time has passed

        if (currentTime - self.lastControlLoopRunTime) + 0.0000000001 >= self.controlTimeStep:
            dt = currentTime - self.lastControlLoopRunTime

            # Figure out where to go/point
            targetOrientation = self.navigator.getTargetOrientation(rocketState, "notRequired", currentTime)

            # Figure out what moment needs to be applied
            desiredMoments = self.momentController.getDesiredMoments(rocketState, environment, targetOrientation, currentTime, dt)
            
            # Apply it by actuating the controlled system
            newActuatorPositionTargets = self.controlledSystem.actuatorController.setTargetActuatorDeflections(desiredMoments, rocketState, environment, currentTime)

            self.lastControlLoopRunTime = currentTime

            pitchError, yawError, rollError = self.calculateAngularError(rocketState.orientation,targetOrientation)

            if self.rocket.simRunner.loggingLevel >= 4:
                self.rocket.simRunner.newControlSystemLogLine("{:<7.3f} ".format(currentTime))      # Start a new line in the control system evaluation log 
                self.appendToControlSystemLogLine(" {:>10.4f} {:>10.8f} {:>10.8f}".format(pitchError, yawError, rollError))

            return newActuatorPositionTargets
        else:
            return False

    def getLogHeader(self):
        headerStrings = [ " Actuator_{}_TargetDeflection".format(i) for i in range(len(self.controlledSystem.actuatorList)) ]
        return "".join(headerStrings)

    def appendToControlSystemLogLine(self, txt: str):
        ''' Appends txt to the current line of the parent `MAPLEAF.SimulationRunners.SingleSimRunner`'s controlSystemEvaluationLog '''
        try:
            self.rocket.simRunner.controlSystemEvaluationLog[-1] += txt
        except AttributeError:
            pass # Force logging not set up for this sim
