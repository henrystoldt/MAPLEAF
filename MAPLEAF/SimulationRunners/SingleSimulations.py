import math
import sys
from copy import deepcopy
from distutils.util import strtobool

from tqdm import tqdm

from MAPLEAF.ENV import Environment
from MAPLEAF.IO import (Logging, Plotting, RocketFlight, SimDefinition,
                        SubDictReader)
from MAPLEAF.Motion import Vector
from MAPLEAF.Rocket import Rocket

__all__ = [ "Simulation", "RemoteSimulation", "WindTunnelSimulation", "loadSimDefinition" ]

def loadSimDefinition(simDefinitionFilePath=None, simDefinition=None, silent=False):
    ''' Loads a simulation definition file into a `MAPLEAF.IO.SimDefinition` object - accepts either a file path or a `MAPLEAF.IO.SimDefinition` object as input '''
    if simDefinition == None and simDefinitionFilePath != None:
        return SimDefinition(simDefinitionFilePath, silent=silent) # Parse simulation definition file

    elif simDefinition != None:
        return simDefinition # Use the SimDefinition that was passed in

    else:
        raise ValueError(""" Insufficient information to initialize a Simulation.
            Please provide either simDefinitionFilePath (string) or fW (SimDefinition), which has been created from the desired Sim Definition file.
            If both are provided, the SimDefinition is used.""")

class Simulation():

    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False):
        '''
            Inputs:
                
                * simDefinitionFilePath:  (string) path to simulation definition file  
                * fW:                     (`MAPLEAF.IO.SimDefinition`) object that's already loaded and parsed the desired sim definition file  
                * silent:                 (bool) toggles optional outputs to the console  
        '''
        self.simDefinition = loadSimDefinition(simDefinitionFilePath, simDefinition, silent)
        ''' Instance of `MAPLEAF.IO.SimDefinition`. Defines the current simulation '''

        self.environment = Environment(self.simDefinition, silent=silent)
        ''' Instance of `MAPLEAF.ENV.Environment`. Will be shared by all Rockets created by this sim runner '''

        self.stagingIndex = None # Set in self.createRocket
        ''' (int) Set in `Simulation.createRocket`. Tracks how many stages have been dropped '''

        self.silent = silent
        ''' (bool) '''

        self.computeStageDropPaths = strtobool(self.simDefinition.getValue("SimControl.StageDropPaths.compute"))

    def run(self, rocket=None):
        ''' 
            Runs simulation defined by self.simDefinition (which has parsed a simulation definition file)

            Returns:
                * stageFlightsPaths: (list[`MAPLEAF.IO.RocketFlight.RocketFlight`]) Each RocketFlight object represents the flight path of a single stage
                * logFilePaths: (list[string]) list of paths to all log files created by this simulation
        '''
        simDefinition = self.simDefinition

        # Initialize the rocket + environment models and simulation logging
        if rocket == None:
            rocket = self.createRocket() # Initialize rocket on launch pad, with all stages attached
        self.rocketStages = [ rocket ] # In this array, 'stage' means independent rigid bodies. Stages are initialized as new rocket objects and added once they are dropped from the main rocket

        # Create progress bar if appropriate
        if simDefinition.getValue("SimControl.EndCondition") == "Time":
            endTime = float(simDefinition.getValue("SimControl.EndConditionValue"))
            progressBar = tqdm(total=endTime+0.01)
            
            try:
                self.logger.continueWritingToTerminal = False
            except AttributeError:
                pass # Logging not set up for this sim
        else:
            progressBar = None

        #### Main Loop Setup #### 
        self.dts = [ float(simDefinition.getValue("SimControl.timeStep")) ]    # (Initial) time step size
        self.endDetectors = [ self._getEndDetectorFunction(rocket, simDefinition) ] # List contains a boolean function that controls sim termination for each stage
        self.stageFlightPaths = [ self._setUpCachingForFlightAnimation(rocket) ] # List will contain resulting flight paths for each stage

        if(rocket.hardwareInTheLoopControl == "yes"):
            print("Setting up hardware in the loop interface")
            rocket.hilInterface.setupHIL(self.rocketStages[0].rigidBody.state)

        #### Main Loop ####
        s = 0 # Stage Index
        while s < len(self.rocketStages):

            if s > 0:
                print("Computing stage {} drop path".format(s))

            rocket = self.rocketStages[s]
            endDetector = self.endDetectors[s]
            flight = self.stageFlightPaths[s]

            endSimulation, FinalTimeStepDt = endDetector(self.dts[s])
                
            while not endSimulation:
                # Take a time step
                try:
                    if FinalTimeStepDt != None:
                        self.dts[s] = FinalTimeStepDt

                    timeStepAdjustmentFactor, self.dts[s] = rocket.timeStep(self.dts[s])

                    if s == 0: # Currently, progress bar only works for bottom stage
                        try:
                            progressBar.update(self.dts[s])
                        except AttributeError:
                            pass
                except:
                    self._handleSimulationCrash()

                # Adjust time step
                self.dts[s] *= timeStepAdjustmentFactor

                # HIL
                if(rocket.hardwareInTheLoopControl == "yes"):
                    rocket.hilInterface.performHIL(rocket.rigidBody.state,rocket.rigidBody.time)

                # Cache states for flight animation
                self.cacheState(rocket, flight)

                # Check whether we should end the simulation, or take a modified-size final time step    
                endSimulation, FinalTimeStepDt = endDetector(self.dts[s])
            
            # Log last state (would be the starting state of the next time step)
            rocket._runControlSystemAndLogStartingState(0.0)

            # Move on to next (dropped) stage
            s += 1

            try:
                progressBar.close()
                
                if not self.silent:
                    sys.stdout.continueWritingToTerminal = True # Actually editing a MAPLEAF.IO.Logging.Logger object here
            except AttributeError:
                pass

        print("Simulation Complete")

        logFilePaths = self._postProcess(simDefinition)

        return self.stageFlightPaths, logFilePaths

    #### Pre-sim ####
    def createRocket(self, stage=None):
        ''' 
            Initializes a rocket, complete with an Environment object and logs, both owned by the instance of this class
            Returns an instance of Rocket with it's Environment/Logs initialized. Can be called by external classes to obtain a prepped rocket (used a lot this way in test cases).
        '''
        # Initialize Rocket
        rocketDictReader = SubDictReader("Rocket", self.simDefinition)  
        rocket = Rocket(rocketDictReader, silent=self.silent, stageToInitialize=stage, simRunner=self, environment=self.environment)       # Initialize Rocket

        if self.simDefinition.getValue('SimControl.RocketPlot') in [ 'On', 'on' ]:
            rocket.plotShape()  # Reference to this simRunner used to add to logs

        if stage == None:
            self._setUpLogging()
            self._createLogDataTableHeaders(rocket)

            self.stagingIndex = 0 # Initially zero, after dropping first stage: 1, after dropping second stage: 2, etc...
            
        return rocket

    def _setUpLogging(self):
        self.loggingLevel = int(self.simDefinition.getValue("SimControl.loggingLevel"))

        if  self.loggingLevel > 0:
            # Set up logging so that the output of any print calls after this point is captured in mainSimulationLog
            self.mainSimulationLog = []
            if self.silent:
                self.logger = Logging.Logger(self.mainSimulationLog, continueWritingToTerminal=False)
            else:
                self.logger = Logging.Logger(self.mainSimulationLog)
            sys.stdout = self.logger
            
            # Output system info to console and to log
            Logging.getSystemInfo(printToConsole=True)
            # Output sim definition file and default value dict to the log only
            self.mainSimulationLog += Logging.getSimDefinitionAndDefaultValueDictsForOutput(simDefinition=self.simDefinition, printToConsole=False)

            # Start force evaluation log if required
            if self.loggingLevel >= 2:
                self.forceEvaluationLog = []

            # Start control system evaluation log if required
            if self.loggingLevel >=4:
                self.controlSystemEvaluationLog = []

        elif self.silent:
            # No intention of writing things to a log file, just prevent them from being printed to the terminal
            _ = []
            logger = Logging.Logger(_, continueWritingToTerminal=False)
            sys.stdout = logger

    def _createLogDataTableHeaders(self, rocket):
        print("Starting Simulation:")

        if self.loggingLevel > 0 or not self.silent:
            # Create main sim log header (written to once per time step)
            mainSimLogHeader = "Time(s) TimeStep(s)" 
            mainSimLogHeader += rocket.rigidBody.state.getLogHeader() + " EulerAngleX(rad) EulerAngleY(rad) EulerAngleZ(rad)"
            if rocket.controlSystem != None:
                mainSimLogHeader += rocket.controlSystem.getLogHeader()

            # Actually print/log the main sim log header
            print(mainSimLogHeader)

            if self.loggingLevel >= 2:
                # Create force evaluation log header (written to once per force evaluation (several time per time step for higher-order time discretizations))
                # Columns always included
                header = "Time(s)" + rocket.rigidBody.state.getLogHeader() + \
                " WindX(m/s) WindY(m/s) WindZ(m/s) AirDensity(kg/m^3)" + \
                " CGX(m), CGY(m), CGZ(m) Mass(kg) MOIx(kg*m^2) MOIy(kg*m^2) MOIz(kg*m^2)" + \
                " Mach UnitRe AOA(deg) RollAngle(deg)"

                # Columns for each rocket component
                for stage in rocket.stages:
                    for component in stage.components:
                        try:
                            header += component.getLogHeader()
                        except AttributeError:
                            pass

                # Total force columns
                header += " CPZ(m) AeroFX(N) AeroFY(N) AeroFZ(N) AeroMX(Nm)" + \
                " AeroMY(Nm) AeroMZ(Nm) GravityFX(N) GravityFY(N) GravityFZ(N)" + \
                " TotalFX(N) TotalFY(N) TotalFZ(N)"
                
                self.forceEvaluationLog.append(header)
            
            if self.loggingLevel >= 4:
                #Create control system evaluation log header (written once per time step)
                # Columns always included
                header = "Time(s)" + \
                " PitchAngularError(degrees) YawAngularError(degrees) RollAngularError(degrees)"

                self.controlSystemEvaluationLog.append(header)

    def _getEndDetectorFunction(self, rocket, simConfig, droppedStage=False):
        ''' 
            Returns a function, which returns a boolean value and Union[None, float], indicating whether the 
                simulation endpoint has been reached. When close to the end of a sim, the second value returned is the recommended
                time step to take to hit the end criteria.
                Simulation end criteria defined in sim definition file. 
            Rocket object must be passed in because the end condition functions require a reference to the rocket, 
                so they can access its current altitude/velocity/time attributes
        '''
        # Read desired end criteria from simulation definition file
        if not droppedStage:
            # Get end condition for main stage
            endCondition = simConfig.getValue("SimControl.EndCondition")
            conditionValue = float(simConfig.getValue("SimControl.EndConditionValue"))
        else:
            # Get end condition for dropped stages
            endCondition = simConfig.getValue("SimControl.StageDropPaths.endCondition")
            conditionValue = float(simConfig.getValue("SimControl.StageDropPaths.endConditionValue"))

        # Define all possible end-detector functions
        def isAfterApogee(dt):
            return rocket.rigidBody.state.velocity.Z <= 0 and rocket.rigidBody.time > 1.0, None
        def isAboveAltitude(dt):
            return rocket.rigidBody.state.position.Z >= conditionValue, None
        def isBelowAltitude(dt):
            return rocket.environment.earthModel.getAltitude(*rocket.rigidBody.state.position) <= conditionValue, None
        def EndTimeReached(dt):
            currTime = rocket.rigidBody.time
            if currTime < conditionValue and currTime + dt >= conditionValue:
                return False, conditionValue+1e-14-currTime
            elif currTime >= conditionValue:
                return True, None
            else:
                return False, None

        # Return the desired function
        if endCondition == "Apogee":
            return isAfterApogee
        elif endCondition == "Altitude" and rocket.rigidBody.state.position.Z < conditionValue:
            return isAboveAltitude
        elif endCondition == "Altitude":
            return isBelowAltitude
        else:
            return EndTimeReached

    def _setUpCachingForFlightAnimation(self, rocket):
        flight = RocketFlight()
        flight.times.append(rocket.rigidBody.time)
        flight.rigidBodyStates.append(rocket.rigidBody.state)
        if rocket.controlSystem != None:  
            # If rocket has moving fins, record their angles for plotting
            nActuators = len(rocket.controlSystem.controlledSystem.actuatorList)
            flight.actuatorDefls = [ [0] for i in range(nActuators) ]
            flight.actuatorTargetDefls = [ [0] for i in range(nActuators) ]
        else:
            flight.actuatorDefls = None
            flight.actuatorTargetDefls = None

        return flight

    #### During sim ####
    def cacheState(self, rocket: Rocket, flight: RocketFlight):
        ''' Adds the rocket's current state to the flight object '''
        time = rocket.rigidBody.time
        flight.times.append(time)
        flight.rigidBodyStates.append(rocket.rigidBody.state)
        if rocket.controlSystem != None:
            try:
                for a in range(len(rocket.controlSystem.controlledSystem.actuatorList)):
                    flight.actuatorDefls[a].append(rocket.controlSystem.controlledSystem.actuatorList[a].getDeflection(time))
                    flight.actuatorTargetDefls[a].append(rocket.controlSystem.controlledSystem.actuatorList[a].targetDeflection)
            except AttributeError:
                # Expecting to arrive here when timestepping a dropped stage of a controlled rocket, which doesn't have canards
                pass
                
    def createNewDetachedStage(self):
        ''' Called by Rocket._stageSeparation '''
        if self.computeStageDropPaths:
            newDetachedStage = self.createRocket(stage=self.stagingIndex)
            # Set kinematic properties to match those of the current top-most stage
            topStage = self.rocketStages[0]
            newDetachedStage.rigidBody.state = deepcopy(topStage.rigidBody.state)
            newDetachedStage.rigidBody.time = topStage.rigidBody.time
            self.rocketStages.append(newDetachedStage)

            # New sim termination condition function
            self.endDetectors.append(self._getEndDetectorFunction(newDetachedStage, self.simDefinition, droppedStage=True))
            self.dts.append(self.dts[0])
            
            # Duplicate existing flight object
            newFlightObject = deepcopy(self.stageFlightPaths[0]) # Will have had the same flight path as the top stage until the moment of separation
            newFlightObject.actuatorDefls = None # Dropped stage shouldn't have any canard deflections
            self.stageFlightPaths.append(newFlightObject)

            self.stagingIndex += 1

    def newForcesLogLine(self, txt):
        try:
            if len(self.forceEvaluationLog) > 0 and self.forceEvaluationLog[-1][-1:] != '\n':
                self.forceEvaluationLog[-1] += "\n"
            self.forceEvaluationLog.append(txt)
        except AttributeError:
            pass # Force logging not desired/set up for this simulation

    def newControlSystemLogLine(self, txt):
        try:
            if len(self.controlSystemEvaluationLog) > 0 and self.controlSystemEvaluationLog[-1][-1:] != '\n':
                self.controlSystemEvaluationLog[-1] += "\n"
            self.controlSystemEvaluationLog.append(txt)
        except AttributeError:
            pass # Control system logging no desired/set up for this simulation

    def discardForceLogsForPreviousTimeStep(self, integrator):
        if self.loggingLevel >= 2:
            # Figure out how many times this integrator evaluates a function derivative (rocket forces in our case)
            if integrator.method == "RK12Adaptive":
                numDerivativeEvals = 2
            else:
                numDerivativeEvals = len(integrator.tableau)-1

            # Remove that number of rows from the end of the force evaluation log
            for i in range(numDerivativeEvals):
                self.forceEvaluationLog.pop(-1)

    def _handleSimulationCrash(self):
        ''' After a simulation crashes, tries to create log files and show plots anyways, before printing a stack trace '''
        print("ERROR: Simulation Crashed, Aborting")
        print("Attempting to save log files and show plots")
        self._postProcess(self.simDefinition)

        # Try to print out the stack trace
        print("Attempting to show stack trace")
        import traceback
        tb = traceback.format_exc()
        print(tb)

        print("Exiting")
        sys.exit()

    #### Post-sim ####
    def _postProcess(self, simDefinition):
        simDefinition.printDefaultValuesUsed() # Print these out before logging, to include them in the log

        # Log results
        logFilePaths = self._logSimulationResults(simDefinition)

        # Transfer key time info to flight objects from rocket
        for i in range(len(self.rocketStages)):
            self.stageFlightPaths[i].engineOffTime = self.rocketStages[i].engineShutOffTime
            self.stageFlightPaths[i].mainChuteDeployTime = self.rocketStages[i].mainChuteDeployTime
            self.stageFlightPaths[i].targetLocation = self.rocketStages[i].targetLocation

        # Plot results
        self._plotSimulationResults(self.rocketStages, simDefinition, self.stageFlightPaths, logFilePaths)

        # Print these out after logging to avoid including the log/plot keys in the unused keys
        # #TODO: Add exceptions for these keys, move this line to before logging so that it's output is also included in the simulation log
        simDefinition.printUnusedKeys()

        return logFilePaths

    def _logSimulationResults(self, simDefinition):
        ''' Logs simulation results to file (as/if specified in sim definition) '''
        logFilePaths = None
        if self.loggingLevel > 0:
            logFilePaths = []

            # Find new file name without overwriting old logs
            periodIndex = simDefinition.fileName.rfind('.')
            fileBaseName = simDefinition.fileName[:periodIndex] + "_simulationLog_run"
            mainLogFilePath = Logging.findNextAvailableNumberedFileName(fileBaseName=fileBaseName, extension=".txt")
                
            logFilePaths.append(mainLogFilePath)
            print("Writing main log to: {}".format(mainLogFilePath))

            # Write main log to file
            with open(mainLogFilePath, 'w+') as file:
                file.writelines(self.mainSimulationLog)

            # Write force evaluation log to file if desired
            if self.loggingLevel >= 2:
                forceLogFilePath = mainLogFilePath.replace("simulationLog", "forceEvaluationLog")
                print("Writing force evaluation log to: {}".format(forceLogFilePath))
                logFilePaths.append(forceLogFilePath)
                with open(forceLogFilePath, 'w+') as file:
                    file.writelines(self.forceEvaluationLog)

                # Post process / calculate force/moment coefficients if desired
                if self.loggingLevel >= 3:
                    bodyDiameter = self.rocketStages[0].bodyTubeDiameter
                    crossSectionalArea = math.pi * bodyDiameter * bodyDiameter / 4
                    expandedLogPath = Logging.postProcessForceEvalLog(forceLogFilePath, refArea=crossSectionalArea, refLength=bodyDiameter)
                    logFilePaths.append(expandedLogPath)

                if self.loggingLevel >= 4:
                    controlSystemLogFilePath = mainLogFilePath.replace("simulationLog", "controlSystemEvaluationLog")
                    print("Writing control system evaluation log to: {}".format(controlSystemLogFilePath))
                    logFilePaths.append(controlSystemLogFilePath)
                    with open(controlSystemLogFilePath, 'w+') as file:
                        file.writelines(self.controlSystemEvaluationLog)

        return logFilePaths

    def _plotSimulationResults(self, rocketStages, simDefinition, flights, logFilePaths):
        ''' Plot simulation results (as/if specified in sim definition) '''

        plotsToMake = simDefinition.getValue("SimControl.plot").split()

        if plotsToMake != ["None"]:

            if "FlightAnimation" in plotsToMake:
                print("Showing flight animation")

                # Show animation
                Plotting.flightAnimation(flights)

                # Done, remove from plotsToMake
                plotsToMake.remove("FlightAnimation")

            if "FlightPaths" in plotsToMake:
                earthModel = self.simDefinition.getValue("Environment.EarthModel")
                if earthModel in [ "None", "Flat" ]:
                    Plotting.plotFlightPaths_NoEarth(flights)
                else:
                    Plotting.plotFlightPaths_FullEarth(flights)

                plotsToMake.remove("FlightPaths")

            # Plot all other columns from log files
            for plotDefinitionString in plotsToMake:
                Plotting.plotFromLogFiles(logFilePaths, plotDefinitionString)

try:
    import ray
    rayAvailable = True
except ImportError:
    rayAvailable = False

if rayAvailable:
    @ray.remote
    class RemoteSimulation(Simulation):
        ''' 
            Exactly the same as Simulation, except the class itself, and its .run method are decorated with ray.remote()
            to enable multithreaded/multi-node simulations using [ray](https://github.com/ray-project/ray)
        '''
        @ray.method(num_returns=2)
        def run(self):
            return super().run()
else:
    RemoteSimulation = None

class WindTunnelSimulation(Simulation):
    def __init__(self, parametersToSweep=None, parameterValues=None, simDefinitionFilePath=None, simDefinition=None, silent=False, smoothLine='False'):
        self.parametersToSweep = ["Rocket.velocity"] if (parametersToSweep == None) else parametersToSweep
        self.parameterValues = [["(0 0 100)", "(0 0 200)", "(0 0 300)"]] if (parameterValues == None) else parameterValues
        self.smoothLine = smoothLine

        # Error checks
        if len(self.parametersToSweep) != len(self.parameterValues):
            raise ValueError("Must have a list of values for each parameter to sweep over. Currently have {} parameters, and {} lists of values.".format(len(self.parametersToSweep), len(self.parameterValues)))

        paramValueCounts = [ len(x) for x in self.parameterValues ]
        if not all([ (x == paramValueCounts[0]) for x in paramValueCounts ]):
            raise ValueError("All lists of parameter values must be of equal length. Currently, list lengths are: {}".format(paramValueCounts))

        Simulation.__init__(self, simDefinitionFilePath, simDefinition, silent)

    def runSweep(self):
        '''
            Runs a single force evaluation for each value of self.parameterToSweekKey contained in self.parameterValueList
            Returns:
                List of log file paths. Main Sim Log will be empty. Force eval log and expanded force eval log will have one entry per sweep point
        '''
        # Make sure we're logging forces (the only way to get data out of this sim runner) and avoid producing any plots
        self.simDefinition.setValue("SimControl.loggingLevel", "3")
        self.simDefinition.setValue("SimControl.plot", "None") 
        self.simDefinition.setValue("SimControl.RocketPlot", "Off")

        if strtobool(self.smoothLine): # interpolate between user set parameter values
            self._addPoints() 

        for i in range(len(self.parameterValues[0])): # i corresponds to # of conditions, ie how many times parameter values will be changed (velocity1, velocity2, ...)
            for j in range(len(self.parameterValues)): # j'th parameter (velocity, temperature)
                self.simDefinition.setValue(self.parametersToSweep[j], self.parameterValues[j][i])

            # Run a single force evaluation, which creates a forces log entry for this force evaluation
            rocket = self.createRocket()
            self.rocketStages = [ rocket ]
            rocket._getAppliedForce(0.0, rocket.rigidBody.state)

        # Write Logs to file, return path
        return self._postProcess()

    def _addPoints(self, pointMultiple=10):
        ''' Edits the parameter sweeps to include a multiple of the previous number of points, linearly interpolated between the given values '''
        for i in range(len(self.parameterValues[0]) - 1): # i corresponds to # of tests to run
            for interpPt in range(1, pointMultiple): # Loops over each new point
                
                intervalStartIndex = pointMultiple*i
                # Index at which to add new point (also interval end index)
                newPointIndex = pointMultiple*i + interpPt

                for param in range(len(self.parametersToSweep)): # j'th corresponds to parameters to sweep over (velocity, temperature)
                    try:
                        # Vector sweep
                        first = Vector(self.parameterValues[param][intervalStartIndex])
                        second = Vector(self.parameterValues[param][newPointIndex])
                        change = second - first
                        incrementalValue = str(interpPt/pointMultiple*change + first)
                    except ValueError:
                        # Scalar sweep
                        first = float(self.parameterValues[param][intervalStartIndex])
                        second = float(self.parameterValues[param][newPointIndex])
                        change = second - first
                        incrementalValue = str(interpPt/pointMultiple*change + first)
                    
                    self.parameterValues[param].insert(newPointIndex, incrementalValue)

    def createRocket(self):
        ''' 
            Do all the same stuff as the parent object, but also re-initialize the environment, 
                to make sure changes to environmental properties during the parameter sweep take effect 
        '''
        self.environment = Environment(self.simDefinition, silent=self.silent)
        return super().createRocket()

    def _setUpLogging(self):
        ''' Override to ensure that logs aren't re-initialized for every simulation.
            mainSimulationLog will only be absent the first time this function is run
            Want to keep all the force data in a single log file '''
        if not hasattr(self, 'mainSimulationLog'):
            return super()._setUpLogging()

    def _createLogDataTableHeaders(self, rocket):
        ''' Prevents the log headers from being re-generated over and over '''
        if self.forceEvaluationLog == []:
            return super()._createLogDataTableHeaders(rocket)

    def _postProcess(self):
        ''' Creates an empty flight path object to prevent errors in the parent function, which is still run to create log files.
            Removes mainSimLog from (returned) log file paths since no time steps we taken by this sim '''
        # Create an empty flight path to prevent errors in the parent function)
        self.stageFlightPaths = [ RocketFlight() ]
        logFilePaths = Simulation._postProcess(self, self.simDefinition)

        # Because no time steps were taken, the main simulation log will not contain any tabular data.
            # Remove it from logFilePaths (but file is still generated)
        for logPath in logFilePaths:
            if "simulationLog" in logPath:
                logFilePaths.remove(logPath)

        return logFilePaths


