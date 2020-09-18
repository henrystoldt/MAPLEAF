#! /usr/bin/python

#Created by: Henry Stoldt
#February 2019
'''
Defines a basic single simulation runner `SingleSimRunner`, as well as more specialized classes that run several simulations in one go.

.. image:: https://storage.needpix.com/rsynced_images/important-1705212_1280.png
'''

import sys
from copy import deepcopy
from distutils.util import strtobool
from math import pi

import matplotlib.pyplot as plt
from tqdm import tqdm

import MAPLEAF.IO.Logging as Logging
import MAPLEAF.IO.Plotting as Plotting
from MAPLEAF.ENV import Environment
from MAPLEAF.IO import RocketFlight, SimDefinition, SubDictReader
from MAPLEAF.Motion import Vector
from MAPLEAF.Rocket import Rocket


class SingleSimRunner():

    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False):
        '''
            Inputs:
                
                * simDefinitionFilePath:  (string) path to simulation definition file  
                * fW:                     (`MAPLEAF.IO.SimDefinition`) object that's already loaded and parsed the desired sim definition file  
                * silent:                 (bool) toggles optional outputs to the console  
        '''

        self.simDefinition = None
        ''' Instance of `MAPLEAF.IO.SimDefinition`. Defines the current simulation '''

        # Load simulation definition file - have to do this before creating the environment from it
        if simDefinition == None and simDefinitionFilePath != None:
            self.simDefinition = SimDefinition(simDefinitionFilePath, silent=silent) # Parse simulation definition file
        elif simDefinition != None:
            self.simDefinition = simDefinition # Use the SimDefinition that was passed in
        else:
            raise ValueError(""" Insufficient information to initialize a SingleSimRunner.
                Please provide either simDefinitionFilePath (string) or fW (SimDefinition), which has been created from the desired Sim Definition file.
                If both are provided, the SimDefinition is used.""")

        self.environment = Environment(self.simDefinition, silent=silent)
        ''' Instance of `MAPLEAF.ENV.Environment`. Will be shared by all Rockets created by this sim runner '''

        self.stagingIndex = None # Set in self.prepRocketForSingleSimulation
        ''' (int) Set in `SingleSimRunner.prepRocketForSingleSimulation`. Tracks how many stages have been dropped '''

        self.silent = silent
        ''' (bool) '''

        self.computeStageDropPaths = strtobool(self.simDefinition.getValue("SimControl.StageDropPaths.compute"))

    def runSingleSimulation(self):
        ''' 
            Runs simulation defined by self.simDefinition (which has parsed a simulation definition file)

            Returns:
                * stageFlightsPaths: (list[`MAPLEAF.IO.RocketFlight.RocketFlight`]) Each RocketFlight object represents the flight path of a single stage
                * logFilePaths: (list[string]) list of paths to all log files created by this simulation
        '''
        simDefinition = self.simDefinition

        # Initialize the rocket + environment models and simulation logging
        rocket = self.prepRocketForSingleSimulation() # Initialize rocket on launch pad, with all stages attached
        self.rocketStages = [ rocket ] # In this array, 'stage' means independent rigid bodies. Stages are initialized as new rocket objects and added once they are dropped from the main rocket

        # Create progress bar if appropriate
        progressBar = None
        if simDefinition.getValue("SimControl.EndCondition") == "Time":
            endTime = float(simDefinition.getValue("SimControl.EndConditionValue"))
            progressBar = tqdm(total=endTime+0.01)
            
            try:
                self.logger.continueWritingToTerminal = False
            except AttributeError:
                pass # Logging not set up for this sim

        #### Main Loop Setup #### 
        #TODO: Perhaps it would be nicer to move some of this info into the Rocket class instead of keeping it all here
        self.dts = [ float(simDefinition.getValue("SimControl.timeStep")) ]    # (Initial) time step size
        self.terminationConditionDetectorFunctions = [ self._getSimEndDetectorFunction(rocket, simDefinition) ] # List contains a boolean function that controls sim termination for each stage
        self.stageFlightPaths = [ self._setUpSimulationResultCachingForFlightAnimation(rocket) ] # List will contain resulting flight paths for each stage

        if(rocket.hardwareInTheLoopControl == "yes"):
            print("Setting up hardware in the loop interface")
            rocket.hilInterface.setupHIL(self.rocketStages[0].rigidBody.state)

        #### Main Loop ####
        i = 0
        while i < len(self.rocketStages):

            if i > 0:
                print("Computing stage {} drop path".format(i))

            endSimulation, lastTimeStepDt = self.terminationConditionDetectorFunctions[i](self.dts[i])
                
            while not endSimulation:
                # Take time step
                try:
                    if lastTimeStepDt != None:
                        self.dts[i] = lastTimeStepDt

                    timeStepAdjustmentFactor, self.dts[i] = self.rocketStages[i].timeStep(self.dts[i])

                    if i == 0:
                        try:
                            progressBar.update(self.dts[i])
                        except AttributeError:
                            # No progress bar defined b/c 'Time' is not the simulation end condition
                            pass
                except:
                    # Simulation has crashed
                    # Try to create log files and plot sim results even though we've encountered an error
                    print("ERROR: Simulation Crashed, Aborting")
                    print("Attempting to save log files and show plots")
                    self._postSingleSimCleanup(simDefinition)

                    # Try to print out the stack trace
                    print("Attempting to show stack trace")
                    import traceback
                    tb = traceback.format_exc()
                    print(tb)

                    print("Exiting")
                    sys.exit()

                # Adjust time step: timeStepAdjustmentFactor == 1 for non-adaptive time stepping
                self.dts[i] *= timeStepAdjustmentFactor

                # HIL
                if(self.rocketStages[i].hardwareInTheLoopControl == "yes"):
                    rocket.hilInterface.performHIL(rocket.rigidBody.state,rocket.rigidBody.time)

                # Cache states for flight animation
                time = self.rocketStages[i].rigidBody.time
                self.stageFlightPaths[i].times.append(time)
                self.stageFlightPaths[i].rigidBodyStates.append(self.rocketStages[i].rigidBody.state)
                if self.rocketStages[i].controlSystem != None:
                    try:
                        for a in range(len(self.rocketStages[i].controlSystem.controlledSystem.actuatorList)):
                            self.stageFlightPaths[i].actuatorDefls[a].append(self.rocketStages[i].controlSystem.controlledSystem.actuatorList[a].getDeflection(time))
                            self.stageFlightPaths[i].actuatorTargetDefls[a].append(self.rocketStages[i].controlSystem.controlledSystem.actuatorList[a].targetDeflection)
                    except AttributeError:
                        # Expecting to arrive here when timestepping a dropped stage of a controlled rocket, which doesn't have canards
                        pass

                # Check whether we should break out of loop    
                endSimulation, lastTimeStepDt = self.terminationConditionDetectorFunctions[i](self.dts[i])
            
            # Log last state (would be the starting state of the next time step)
            self.rocketStages[i]._runControlSystemAndLogStartingState(0.0)

            # Move on to next (dropped) stage
            i += 1

            try:
                progressBar.close()
                
                if not self.silent:
                    # Actually editing a MAPLEAF.IO.Logging.Logger object here
                    sys.stdout.continueWritingToTerminal = True
            except AttributeError:
                pass

        print("Simulation Complete")

        logFilePaths = self._postSingleSimCleanup(simDefinition)

        return self.stageFlightPaths, logFilePaths

    #### Pre-sim ####
    def prepRocketForSingleSimulation(self, stage=None):
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
            self._setUpSingleSimLogging()
            self.stagingIndex = 0 # Initially zero, after dropping first stage: 1, after dropping second stage: 2, etc...
            
            # Add data table headers to logs
            self._createLogDataTableHeaders(rocket)

        return rocket

    def _setUpSingleSimLogging(self):
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

    def _getSimEndDetectorFunction(self, rocket, simConfig, droppedStage=False):
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

    def _setUpSimulationResultCachingForFlightAnimation(self, rocket):
        flight = RocketFlight()
        flight.times.append(rocket.rigidBody.time)
        flight.rigidBodyStates.append(rocket.rigidBody.state)
        if rocket.controlSystem != None:  
            # If rocket has moving fins, record their angles for plotting
            flight.actuatorDefls = [ [0] for i in range(rocket.controlSystem.controlledSystem.numFins) ]
            flight.actuatorTargetDefls = [ [0] for i in range(rocket.controlSystem.controlledSystem.numFins) ]
        else:
            flight.actuatorDefls = None
            flight.actuatorTargetDefls = None

        return flight

    #### During sim ####
    def createNewDetachedStage(self):
        ''' Called by Rocket._stageSeparation '''
        if self.computeStageDropPaths:
            newDetachedStage = self.prepRocketForSingleSimulation(stage=self.stagingIndex)
            # Set kinematic properties to match those of the current top-most stage
            topStage = self.rocketStages[0]
            newDetachedStage.rigidBody.state = deepcopy(topStage.rigidBody.state)
            newDetachedStage.rigidBody.time = topStage.rigidBody.time
            self.rocketStages.append(newDetachedStage)

            # New sim termination condition function
            self.terminationConditionDetectorFunctions.append(self._getSimEndDetectorFunction(newDetachedStage, self.simDefinition, droppedStage=True))
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

    def discardForceLogsForLastTimeStep(self, integrator):
        if self.loggingLevel >= 2:
            # Figure out how many times this integrator evaluates a function derivative (rocket forces in our case)
            if integrator.method == "RK12Adaptive":
                numDerivativeEvals = 2
            else:
                numDerivativeEvals = len(integrator.tableau)-1

            # Remove that number of rows from the end of the force evaluation log
            for i in range(numDerivativeEvals):
                self.forceEvaluationLog.pop(-1)

    #### Post-sim ####
    def _postSingleSimCleanup(self, simDefinition):
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
                    crossSectionalArea = pi * bodyDiameter * bodyDiameter / 4
                    expandedLogPath = Logging.postProcessForceEvalLog(forceLogFilePath, refArea=crossSectionalArea, refLength=bodyDiameter)
                    logFilePaths.append(expandedLogPath)

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

class WindTunnelRunner(SingleSimRunner):
    def __init__(self, parameterToSweepKey="Rocket.velocity", parameterValueList=["(0 0 100)", "(0 0 200)", "(0 0 300)"], simDefinitionFilePath=None, fW=None, silent=False, smoothLine='False'):
        self.parameterToSweepKey = parameterToSweepKey
        self.parameterValueList = parameterValueList
        self.smoothLine = smoothLine

        SingleSimRunner.__init__(self, simDefinitionFilePath, fW, silent)

    def runSweep(self):
        '''
            Runs a single force evaluation for each value of self.parameterToSweekKey contained in self.parameterValueList
            Returns:
                List of log file paths. Main Sim Log will be empty. Force eval log and expanded force eval log will have one entry per sweep point
        '''
        # Make sure we're logging forces (the only way to get data out of this sim runner)
        self.simDefinition.setValue("SimControl.loggingLevel", "3")
        self.simDefinition.setValue("SimControl.plot", "None") # Don't produce any plots
        self.simDefinition.setValue("SimControl.RocketPlot", "Off") # Don't produce any plots

        # Run a single force evaluation for each parameter value
        # Regenerate simulation environment each time to allow user to change anything about the sim definition

        # parameterValueDicts = SubDictReader.getImmediateSubDicts(SubDictReader.simDefDictPathToReadFrom + ".ParameterSweep")
        if self.smoothLine == 'True': # interpolate between user set parameter values
            for i in range(len(self.parameterValueList[0])-1): # i corresponds to # of conditions, ie how many times parameter values will be changed (velocity1, velocity2, ...)
                # this loop will run for as many values are given PER parameter. i'th value for each parameter
                # Set sim def value
                if not self.silent:
                    pass
                    # print("Setting value: {} to {}".format(self.parameterToSweepKey, self.paramValueList[j][i]))
                
                k = 0
                while k < 10:
                    for j in range(len(self.parameterValueList)): # j'th parameter (velocity, temperature)
                        # this loop will set a value for each given parameter type that is specified
                        try:
                            first = Vector(self.parameterValueList[j][i])
                            second = Vector(self.parameterValueList[j][i+1])
                            bucket = second - first
                            incrementalValue = str(k/10 * bucket + Vector(self.parameterValueList[j][i]))
                        except ValueError:
                            first = float(self.parameterValueList[j][i])
                            second = float(self.parameterValueList[j][i+1])
                            bucket = second - first
                            incrementalValue = str(k/10 * bucket + float(self.parameterValueList[j][i]))
                        self.simDefinition.setValue(self.parameterToSweepKey[j], incrementalValue)
                    
                    if not self.silent:
                        print("Running Single Force Evaluation")
                    rocket = self.prepRocketForSingleSimulation()
                    self.rocketStages = [rocket]
                    rocket._getAppliedForce(0.0, rocket.rigidBody.state)

                    if i == len(self.parameterValueList[0])-2 and k == 9:
                        for j in range(len(self.parameterValueList)):
                            incrementalValue = self.parameterValueList[j][i+1]
                            self.simDefinition.setValue(self.parameterToSweepKey[j], self.parameterValueList[j][i+1])
                        if not self.silent:
                            print("Running Single Force Evaluation")
                        rocket = self.prepRocketForSingleSimulation()
                        self.rocketStages = [rocket]
                        rocket._getAppliedForce(0.0, rocket.rigidBody.state)

                    k +=1   

        else:
            for i in range(len(self.parameterValueList[0])): # i corresponds to # of conditions, ie how many times parameter values will be changed (velocity1, velocity2, ...)
                # this loop will run for as many values are given PER parameter
                # Set sim def value
                if not self.silent:
                    pass
                    # print("Setting value: {} to {}".format(self.parameterToSweepKey, self.paramValueList[j][i]))
                
                for j in range(len(self.parameterValueList)): # j'th parameter (velocity, temperature)
                    # this loop will set a value for each given parameter type that is specified
                    self.simDefinition.setValue(self.parameterToSweepKey[j], self.parameterValueList[j][i])
            
                # Run + log the force evaluation
                if not self.silent:
                    print("Running Single Force Evaluation")
                rocket = self.prepRocketForSingleSimulation()
                self.rocketStages = [rocket]
                rocket._getAppliedForce(0.0, rocket.rigidBody.state)

        # Write Logs to file
        logFilePaths = self._postSingleSimCleanup(self.simDefinition)

        # Because no time steps were taken, the main simulation log will not contain any plottable, tabular data.
            # Remove it from logFilePaths
        for logPath in logFilePaths:
            if "simulationLog" in logPath:
                logFilePaths.remove(logPath)
                break

        return logFilePaths

    def prepRocketForSingleSimulation(self):
        ''' 
            Do all the same stuff as the parent object, but also re-initialize the environment, 
                to make sure changes to environmental properties during the parameter sweep take effect 
        '''
        self.environment = Environment(self.simDefinition, silent=self.silent)
        return super().prepRocketForSingleSimulation()

    def _setUpSingleSimLogging(self):
        # Override to ensure that logs aren't re-initialized for every simulation.
            # mainSimulationLog will only be absent the first time this function is run
            # Want to keep all the force data in a single log file
        if not hasattr(self, 'mainSimulationLog'):
            return super()._setUpSingleSimLogging()

    def _createLogDataTableHeaders(self, rocket):
        # Prevent the log headers from being re-generated over and over
        if self.forceEvaluationLog == []:
            return super()._createLogDataTableHeaders(rocket)

    def _postSingleSimCleanup(self, simDefinition):
        # Create an empty flight path to prevent errors in the parent function)
        self.stageFlightPaths = [ RocketFlight() ]
        return super()._postSingleSimCleanup(simDefinition)

def isMonteCarloSimulation(simDefinition):
    ''' Returns bool '''
    try:
        nRuns = float(simDefinition.getValue("MonteCarlo.numberRuns"))
        if nRuns > 1:
            return True
    except (KeyError, ValueError):
        pass

    return False

class MonteCarloSimRunner(SingleSimRunner):
    '''
        Runs a simulation repeatedly, always resampling parameters that have a specfified normal distribution in the simulation definition
    '''
    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False):
        SingleSimRunner.__init__(self, simDefinitionFilePath=simDefinitionFilePath, simDefinition=simDefinition, silent=silent)
    
    def _reInitializeEnvironment(self):
        '''
            Aside from normally-distributed parameters defined in the sim definition file, the environment can have their own probabilistic wind values.
            Therefore, we need to re-initialize the environment (and thereby re-sample any probability distributions) before each individual simulation in a monte carlo run
        '''
        self.environment = Environment(self.simDefinition, silent=self.silent)

    def runMonteCarloSimulation(self):
        ''' Pass in SimDefinition with simulation definition loaded '''
        
        simDefinition = self.simDefinition

        # Make sure plots don't show after each sim
        simDefinition.setValue("SimControl.plot", "None")
        simDefinition.setValue("SimControl.RocketPlot", "Off")

        # Set up saving key results
        resultsToOutput = simDefinition.getValue("MonteCarlo.output")
        landingLocations = []
        apogees = []
        maxSpeeds = []
        flightTimes = []
        maxHorizontalVels = []
        flights = []

        #### Set up Logging ####
        mCLogger = Logging.MonteCarloLogger()
        simDefinition.monteCarloLogger = mCLogger # SimDefinition needs to have a reference to the monte carlo log to log whenever it samples a variable

        nRuns = int(simDefinition.getValue("MonteCarlo.numberRuns"))

        mCLogger.log("")
        mCLogger.log("Running Monte Carlo Simulation: {} runs".format(nRuns))                
        
        ### Run simulations ###
        for i in range(nRuns):
            # Start monte carlo log entry for this sim
            mCLogger.log("\nMonte Carlo Run #{}".format(i+1))
            
            # Run sim
            self._reInitializeEnvironment()
            stageFlightPaths, _ = self.runSingleSimulation()
            flight = stageFlightPaths[0]
            
            # Save results
            landingLocations.append(flight.getLandingLocation())
            apogees.append(flight.getApogee())
            maxSpeeds.append(flight.getMaxSpeed())
            flightTimes.append(flight.getFlightTime())
            maxHorizontalVels.append(flight.getMaxHorizontalVel())
            
            if "flightPaths" in resultsToOutput:
                flight = Plotting._keepNTimeSteps(flight, 900) # Limit the number of time steps saved to avoid wasting memory
                flights.append(flight)

        ### Plot/Output results ###
        mCLogger.log("")
        mCLogger.log("Monte Carlo results:")

        if "landingLocations" in resultsToOutput:
            Plotting.plotAndSummarizeVectorResult(landingLocations, name="Landing location", monteCarloLogger=mCLogger)
        if "apogees" in resultsToOutput:
            Plotting.plotAndSummarizeScalarResult(apogees, name="Apogee", monteCarloLogger=mCLogger)
        if "maxSpeeds" in resultsToOutput:
            Plotting.plotAndSummarizeScalarResult(maxSpeeds, name="Max speed", monteCarloLogger=mCLogger)
        if "flightTimes" in resultsToOutput:
            Plotting.plotAndSummarizeScalarResult(flightTimes, name="Flight time", monteCarloLogger=mCLogger)
        if "maxHorizontalVels" in resultsToOutput:
            Plotting.plotAndSummarizeScalarResult(maxHorizontalVels, name="Max horizontal speed", monteCarloLogger=mCLogger)
        if "flightPaths" in resultsToOutput:
            Plotting.plotFlightPaths_NoEarth(flights)

        
        if resultsToOutput != "None" and len(resultsToOutput) > 0:
            dotIndex = simDefinition.fileName.rfind('.')
            extensionFreeSimDefFileName = simDefinition.fileName[0:dotIndex]
            logFilePath = extensionFreeSimDefFileName + "_monteCarloLog_run"

            logPath = mCLogger.writeToFile(fileBaseName=logFilePath)
            print("Wrote Monte Carlo Log to: {}".format(logPath))

def runParallelMonteCarloSim(simDefinition):
    # TODO: Finish implementation, make parallelism optional
    # No plotting for monte carlo sims
    simDefinition.setValue("SimControl.plot", "None")
    simDefinition.setValue("SimControl.RocketPlot", "Off")

    import ray
    ray.init()

    global SingleSimRunner

    SingleSimRunner = ray.remote(SingleSimRunner)

    runner1 = SingleSimRunner.remote(simDefinition=simDefinition, silent=True, num_return_vals=3)
    runner2 = SingleSimRunner.remote(simDefinition=simDefinition, silent=True)

    f1 = runner1.runSingleSimulation.remote()
    f2 = runner2.runSingleSimulation.remote()
    
    results = ray.get(f1)
    results2 = ray.get(f2)

    print("1: {}".format(results))
    print("2: {}".format(results2))

class ConvergenceSimRunner(SingleSimRunner):
    '''
        Runs a simulation repeatedly, decreasing the time step or target error each time, monitoring for convergence
    '''
    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False):
        SingleSimRunner.__init__(self, simDefinitionFilePath=simDefinitionFilePath, simDefinition=simDefinition, silent=silent)

    def convergeSimEndPosition(self, refinementRatio=2, simLimit=10, plot=True, stopAtConvergence=False, showPlot=True, plotLineLabel="Simulations", ax1=None, ax2=None):
        '''
            Takes simulation and runs it repeatedly, cutting the time step in half each time.
            Once convergence is approximately asymptotic, exits and returns series of final positions, convergence order, and extrapolated final position
            Should use with simulations that have an EndCondition of type "Time"
                # Otherwise sim will be run using current settings, and its endtime will be taken as the new end time for future convergence sims
            This Fn called by compareIntegrationSchemes functions

            Parameters:
                simConfigFilePath       string, /path/to/simConfigFile
                fW                      SimDefinition, overrides simConfigFilePath
                refinementRatio         Number, Each time sim is run, time step or target error is divided by this number
                simLimit                Number, Max number of simulations to run (takes exponentially more time to run more simulations)
                plot                    True/False, whether to plot the results
                stopAtConvergence       True/False, if False, runs simLimit simulations even if asymptotic convergence is reached earlier
                showPlot                True/False, if True, calls plt.show()
                plotLineLabel           string, Label of line on plot
                ax1                     matplotlib Axes, Z-location (Y) axis
                ax2                     matplotlib Axes, Wall Time (Y) axis (2nd Y-axis)
        '''
        from MAPLEAF.IO.gridConvergenceFunctions import checkConvergence
        from statistics import mean
        import time

        self._setUpConfigFileForConvergenceRun()
        
        timeStepMethod = self.simDefinition.getValue("SimControl.timeDiscretization")
        adaptiveTimeStepping = "Adaptive" in timeStepMethod

        timeStepKey = "SimControl.timeStep"
        targetErrorKey = "SimControl.TimeStepAdaptation.targetError"

        #### Run Simulations ####
        print("Starting convergence simulations")
        if not adaptiveTimeStepping:
            timeStep = float(self.simDefinition.getValue(timeStepKey))*refinementRatio # Multiplied by 2 to give correct time step in first iteration
        else:
            targetError = float(self.simDefinition.getValue(targetErrorKey))*refinementRatio # Multiplied by 2 to give correct time step in first iteration

        simCount = 1
        finalPositionHistory = []
        convergenceHistory = []
        timeStepHistory = []
        simTimeHistory = []

        def printConvergenceHistory(ax1=ax1, ax2=ax2):
            print("")
            print("Convergence History:")
            print("Integration Method: {}".format(timeStepMethod))

            xPos = []
            yPos = []
            zPos = []

            for i in range(len(finalPositionHistory)):
                finalPos = finalPositionHistory[i]
                xPos.append(finalPos[0])
                yPos.append(finalPos[1])
                zPos.append(finalPos[2])
                printString = "FinalPosition(m): {:>7.3f} WallTime(s): {:>7.3f} ".format(finalPos, simTimeHistory[i])

                if i > 1: # TODO: Get convergence results into the .csv file
                    ordersOfConvergence, GCI12s, GCI23s, asymptoticChecks, richardsonExtrapVals, uncertainties = convergenceHistory[i-2]
                    printString += " Avg Order: {:>4.2f}, Avg Asymptotic Check: {:>6.3f}".format(mean(ordersOfConvergence), mean(asymptoticChecks))

                print(printString)

            if plot:
                if ax1 == None:
                    ax1 = plt.gca()
                if ax2 == None:
                    ax2 = ax1.twinx()

                ax1.plot(timeStepHistory, zPos, ":D", label=plotLineLabel)
                ax1.set_ylabel("Final Z Coordinate (m)")

                ax2.plot(timeStepHistory, simTimeHistory, "-*", label=plotLineLabel + " Wall Time")
                ax2.set_ylabel("Wall Time (s)")
                
                plt.xscale("log")
                plt.xlabel("Time Step (s)")
                plt.legend()
                plt.tight_layout()

                if showPlot:
                    plt.show()

        while simCount <= simLimit:
            if not adaptiveTimeStepping:
                timeStep /= refinementRatio
                self.simDefinition.setValue(timeStepKey, str(timeStep))
                timeStepHistory.append(timeStep)
                print("Simulation {}, Time step: {}".format(simCount, timeStep))
            else:
                targetError /= refinementRatio
                self.simDefinition.setValue(targetErrorKey, str(targetError))
                timeStepHistory.append(targetError)
                print("Simulation {}, Time step: {}".format(simCount, targetError))

            startTime = time.time()
            flights, _ = self.runSingleSimulation()
            flight = flights[0]
            wallTime = time.time() - startTime
            simTimeHistory.append(wallTime)

            finalPositionHistory.append(flight.rigidBodyStates[-1].position)
            print("Final Position: {:1.3f}".format(finalPositionHistory[-1]))

            if len(finalPositionHistory) >= 3:
                # Check whether result is converging
                cV, mV, fV = finalPositionHistory[-3:]
                print("Checking convergence")
                convergResult = checkConvergence(cV, mV, fV, refinementRatio)
                ordersOfConvergence, GCI12s, GCI23s, asymptoticChecks, richardsonExtrapVals, uncertainties = convergResult
                convergenceHistory.append(convergResult)
                directions = ["X", "Y", "Z"]
                for d in range(len(directions)):
                    print("{}-Direction: Order: {:>4.3f}, Asymptotic Check: {:>6.3f}, RichardsonExtrap: {:>7.3f}, Estimated Uncertainty: {:>6.3f}".format(directions[d], ordersOfConvergence[d], asymptoticChecks[d], richardsonExtrapVals[d], uncertainties[d]))
                
                if stopAtConvergence and abs(sum(asymptoticChecks) / len(asymptoticChecks) - 1) < 0.1 and max(asymptoticChecks) - min(asymptoticChecks) < 0.2:
                    print("Simulation Converging Asymptotically")
                    printConvergenceHistory()
                    return timeStepHistory, finalPositionHistory, flight
            
            simCount += 1

        # Output whether convergence was achieved
        if simLimit >= 3:
            print("Asymptotic convergence not reached within {} simulations".format(simLimit))
        else:
            print("Asymptotic convergence impossible to reach with less than 3 iterations (performed {}). Adjust the parameter 'simLimit' to perform more iterations".format(simLimit))

        printConvergenceHistory(ax1, ax2)

        return timeStepHistory, finalPositionHistory, simTimeHistory

    def compareClassicalIntegrationSchemes(self, saveFigure=False, showPlot=True, simLimit=10, integrationSchemes = [ "Euler", "RK2Midpoint", "RK2Heun", "RK4" ], convergenceResultFilePath="convergenceResult.csv"):
        ''' Arguments:
                simConfigFilePath (string)
                saveFigure (Bool)
                convergenceFilePath (string or None) - will overwrite old files

            Outputs:
                Plot
                .csv file (Optional)

            Returns:
                Nothing
        '''

        plt.figure(figsize=(3.5,3))
        ax1 = plt.gca()
        ax2 = plt.twinx()
        
        initTimeStep = float(self.simDefinition.getValue("SimControl.timeStep"))

        # Lists to store results
        timeStepHistory = []
        finalPositionHistories = []
        wallTimeHistory = []

        # Run series of simulations for each integration scheme
        for scheme in integrationSchemes:
            self.simDefinition.setValue("SimControl.timeDiscretization", scheme)
            self.simDefinition.setValue("SimControl.timeStep", str(initTimeStep))
            timeSteps, finalPositions, wallTimes = self.convergeSimEndPosition(showPlot=False, simLimit=simLimit, plotLineLabel=scheme, ax1=ax1, ax2=ax2)
            
            timeStepHistory = timeSteps
            finalPositionHistories.append(finalPositions)
            wallTimeHistory.append(wallTimes)

        print("Simulations complete")

        if convergenceResultFilePath != None:
            import csv
            print("Writing convergence results to: {}".format(convergenceResultFilePath))

            with open(convergenceResultFilePath, 'w', newline='') as file:
                writer = csv.writer(file)
                
                # Write Column Headers
                headerRow = [ "TimeStep(s)" ]
                for timeStep in range(len(integrationSchemes)):
                    intScheme = integrationSchemes[timeStep]
                    headerRow += [ "{}_FinalX(m)".format(intScheme), "{}_FinalY(m)".format(intScheme), "{}_FinalZ(m)".format(intScheme), "{}_WallTime(s)".format(intScheme) ]
                
                writer.writerow(headerRow)
                
                # Write convergence results, time step by time step
                for timeStep in range(len(timeStepHistory)):
                    row = [ timeStepHistory[timeStep] ]

                    for integrationScheme in range(len(integrationSchemes)):
                        row.append(finalPositionHistories[integrationScheme][timeStep].X)
                        row.append(finalPositionHistories[integrationScheme][timeStep].Y)
                        row.append(finalPositionHistories[integrationScheme][timeStep].Z)
                        row += [ wallTimeHistory[integrationScheme][timeStep] ]

                    writer.writerow(row)

        if saveFigure:
            try:
                plt.savefig("/home/hhstoldt/Documents/flightSimPaper/Figures/Images/AdaptTimeStepConvergence_ConstTimeStep.eps", bbox_inches="tight", pad_inches=0)
            except:
                plt.savefig("C:/Users/rando/Documents/flightSimPaper/Figures/Images/AdaptTimeStepConvergence_ConstTimeStep.eps", bbox_inches="tight", pad_inches=0)

        print("Showing plot")
        if showPlot:
            plt.show()

    def compareAdaptiveIntegrationSchemes(self, saveFigure=False, showPlot=True, integrationSchemes=["RK12Adaptive", "RK23Adaptive", "RK45Adaptive"], simLimit=10, convergenceResultFilePath="adaptiveConvergenceResult.csv"):
        ''' Arguments:
                simConfigFilePath (string)
                saveFigure (Bool)
                convergenceFilePath (string or None) - will overwrite old files

            Outputs:
                Plot
                .csv file (Optional)

            Returns:
                Nothing
        '''

        plt.figure(figsize=(3.5,3))
        ax1 = plt.gca()
        ax2 = plt.twinx()
        
        initErrorTarget = float(self.simDefinition.getValue("SimControl.TimeStepAdaptation.targetError"))
        
        # Lists to store results
        targetErrorHistory = []
        finalPositionHistories = []
        wallTimeHistory = []

        # Run simulations
        for scheme in integrationSchemes:
            self.simDefinition.setValue("SimControl.timeDiscretization", scheme)
            self.simDefinition.setValue("SimControl.TimeStepAdaptation.targetError", str(initErrorTarget))
            timeSteps, finalPositions, wallTimes = self.convergeSimEndPosition(showPlot=False, plotLineLabel=scheme, refinementRatio=2, simLimit=simLimit, ax1=ax1, ax2=ax2)
            
            targetErrorHistory = timeSteps
            finalPositionHistories.append(finalPositions)
            wallTimeHistory.append(wallTimes)

        # Write results to .csv file
        if convergenceResultFilePath != None:
            import csv
            print("Writing convergence results to: {}".format(convergenceResultFilePath))

            with open(convergenceResultFilePath, 'w', newline='') as file:
                writer = csv.writer(file)
                
                # Write Column Headers
                headerRow = [ "TargetError" ]
                for timeStep in range(len(integrationSchemes)):
                    intScheme = integrationSchemes[timeStep]
                    headerRow += [ "{}_FinalX(m)".format(intScheme), "{}_FinalY(m)".format(intScheme), "{}_FinalZ(m)".format(intScheme), "{}_WallTime(s)".format(intScheme) ]
                
                writer.writerow(headerRow)
                
                # Write convergence results, time step by time step
                for timeStep in range(len(targetErrorHistory)):
                    row = [ targetErrorHistory[timeStep] ]

                    for integrationScheme in range(len(integrationSchemes)):
                        row.append(finalPositionHistories[integrationScheme][timeStep].X)
                        row.append(finalPositionHistories[integrationScheme][timeStep].Y)
                        row.append(finalPositionHistories[integrationScheme][timeStep].Z)
                        row += [ wallTimeHistory[integrationScheme][timeStep] ]

                    writer.writerow(row)

        # Save results figure
        if saveFigure:
            try:
                plt.savefig("/home/hhstoldt/Documents/flightSimPaper/Figures/Images/TimeStepConvergence_ConstTimeStep.eps", bbox_inches="tight", pad_inches=0)
            except:
                plt.savefig("C:/Users/rando/Documents/flightSimPaper/Figures/Images/TimeStepConvergence_ConstTimeStep.eps", bbox_inches="tight", pad_inches=0)

        # Show Plot
        plt.xlabel("Target Error")

        if showPlot:
            plt.show()

    def _setUpConfigFileForConvergenceRun(self):
        print("Will attempt to converge final rocket position of simulation: {}".format(self.simDefinition.fileName))
        self.simDefinition.disableDistributionSampling = True # Don't sample from probability distributions while trying to converge a sim

        # Make sure no plots are created every time the sim runs
        self.simDefinition.setValue("SimControl.plot", "None")

        #### Make sure End Condition is a time ####
        endCondition = self.simDefinition.getValue("SimControl.EndCondition")
        if endCondition != "Time":
            print("Running simulation to determine end time")
            # Otherwise run the sim, get the end time and 
            flights, _ = self.runSingleSimulation()
            endTime = flights[0].times[-1]
            # set that to the end condition
            print("Setting EndCondition = Time, EndConditionValue = {}".format(endTime))
            self.simDefinition.setValue("SimControl.EndCondition", "Time")
            self.simDefinition.setValue("SimControl.EndConditionValue", str(endTime))

if __name__ == "__main__":
    simDef = SimDefinition("MAPLEAF/Examples/Simulations/AdaptTimeStep.mapleaf")
    runParallelMonteCarloSim(simDef)
