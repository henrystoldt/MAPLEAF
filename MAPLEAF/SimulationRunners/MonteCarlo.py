import ray
import random

from MAPLEAF.IO import Logging, Plotting, SimDefinition
from MAPLEAF.SimulationRunners import RemoteSimRunner, SingleSimRunner

__all__ = [ "runMonteCarloSimulation" ]

def runMonteCarloSimulation(simDefinitionFilePath=None, simDefinition=None, silent=False, nCores=1):
    if nCores > 1:
        return _runMonteCarloSimulation_Parallel(simDefinitionFilePath, simDefinition, silent, nCores)
    else:
        return _runMonteCarloSimulation_SingleThreaded(simDefinitionFilePath, simDefinition, silent)

def _runMonteCarloSimulation_SingleThreaded(simDefinitionFilePath=None, simDefinition=None, silent=False):
    # Load simulation definition file
    if simDefinition == None and simDefinitionFilePath != None:
        simDefinition = SimDefinition(simDefinitionFilePath, silent=silent) # Parse simulation definition file
    elif simDefinition == None:
        raise ValueError(""" Insufficient information to initialize a Simulation.
            Please provide either simDefinitionFilePath (string) or fW (SimDefinition), which has been created from the desired Sim Definition file.
            If both are provided, the SimDefinition is used.""")

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
        simRunner = SingleSimRunner(simDefinition=simDefinition, silent=True)
        stageFlightPaths, _ = simRunner.runSingleSimulation()
        Logging.removeLogger() # Remove the logger created by simmRunner #TODO Logging needs improvement
        
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

def _runMonteCarloSimulation_Parallel(simDefinitionFilePath=None, simDefinition=None, silent=False, nProcesses=1):
    '''
        Runs a probabilistic simulation a several times, collects and displays average results for common parameters
        Parallelized using [ray](https://github.com/ray-project/ray)
    '''
    # Load simulation definition file
    if simDefinition == None and simDefinitionFilePath != None:
        simDefinition = SimDefinition(simDefinitionFilePath, silent=silent) # Parse simulation definition file
    elif simDefinition == None:
        raise ValueError(""" Insufficient information to initialize a Simulation.
            Please provide either simDefinitionFilePath (string) or fW (SimDefinition), which has been created from the desired Sim Definition file.
            If both are provided, the SimDefinition is used.""")
    
    # Make sure plots don't show after each sim
    simDefinition.setValue("SimControl.plot", "None")
    simDefinition.setValue("SimControl.RocketPlot", "Off")
    try:
        randomSeed = simDefinition.getValue("MonteCarlo.randomSeed")
    except KeyError:
        randomSeed = random.randrange(1e7)
    rng = random.Random(randomSeed)    
    
    # Set arrays to save key results
    resultsToOutput = simDefinition.getValue("MonteCarlo.output")
    landingLocations = []
    apogees = []
    maxSpeeds = []
    flightTimes = []
    maxHorizontalVels = []
    flights = []

    def postProcess(rayObject):
        # Get sim results
        stagePaths = ray.get(rayObject)
        
        # Save results from the top stage
        flight = stagePaths[0]

        landingLocations.append(flight.getLandingLocation())
        apogees.append(flight.getApogee())
        maxSpeeds.append(flight.getMaxSpeed())
        flightTimes.append(flight.getFlightTime())
        maxHorizontalVels.append(flight.getMaxHorizontalVel())
        
        if "flightPaths" in resultsToOutput:
            flight = Plotting._keepNTimeSteps(flight, 900) # Limit the number of time steps saved to avoid wasting memory
            flights.append(flight)

    #### Set up Logging ####
    mCLogger = Logging.MonteCarloLogger()
    simDefinition.monteCarloLogger = mCLogger # SimDefinition needs to have a reference to the monte carlo log to log whenever it samples a variable

    nRuns = int(simDefinition.getValue("MonteCarlo.numberRuns"))     

    ### Run simulations ###
    # TODO: Adapt this to work on a cluster
        # Reminder that ray must be initialized separately on a cluster, before running ray.init()
        # https://docs.ray.io/en/latest/cluster/index.html

    ray.init()

    # Start simulations
    runningJobs = []
    for i in range(nRuns):
        # Don't start more sims than there are processes available
        if i >= nProcesses:
            completedJobs, runningJobs = ray.wait(runningJobs)
            for completedJob in completedJobs:
                # Save results
                postProcess(completedJob)

        # Make sure each copy of simDefinition has a different, but repeatable random seed
        newRandomSeed = rng.randrange(1e7)
        simDefinition.rng = random.Random(newRandomSeed)

        # Start sim
        simRunner = RemoteSimRunner.remote(simDefinition=simDefinition, silent=True)
        flightPathsFuture, logPathsFuture = simRunner.runSingleSimulation.remote()
        runningJobs.append(flightPathsFuture)

    # Wait for remaining sims to complete
    for remainingJob in runningJobs:
        postProcess(remainingJob)

    ray.shutdown()

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
