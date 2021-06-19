import random
from copy import deepcopy

from MAPLEAF.IO import Logging, Plotting
from .SingleSimulations import runSimulation, Simulation, loadSimDefinition

__all__ = [ "runMonteCarloSimulation" ]


def runMonteCarloSimulation(simDefinitionFilePath=None, simDefinition=None, silent=False, nCores=1):
    simDefinition = loadSimDefinition(simDefinitionFilePath, simDefinition, silent)

    nRuns, mCLogger, outputLists = _prepSim(simDefinition)    

    if nCores > 1:
        _runSimulations_Parallel(simDefinition, nRuns, outputLists, silent, nCores)
    else:            
        _runSimulations_SingleThreaded(simDefinition, nRuns, outputLists, mCLogger, silent)
    
    _showResults(simDefinition, outputLists, mCLogger)

def _prepSim(simDefinition):
    # Make sure plots don't show after each sim
    simDefinition.setValue("SimControl.plot", "None")
    simDefinition.setValue("SimControl.RocketPlot", "Off")

    # Get # Runs
    nRuns = int(simDefinition.getValue("MonteCarlo.numberRuns"))
    
    # Set up Logging
    mCLogger = Logging.MonteCarloLogger()
    mCLogger.log("")
    mCLogger.log("Running Monte Carlo Simulation: {} runs".format(nRuns))  

    simDefinition.monteCarloLogger = mCLogger # SimDefinition needs to have a reference to the monte carlo log to log whenever it samples a variable

    # Set up saving key results
    landingLocations = []
    apogees = []
    maxSpeeds = []
    flightTimes = []
    maxHorizontalVels = []
    flights = [] 
    outputLists = [ landingLocations, apogees, maxSpeeds, flightTimes, maxHorizontalVels, flights ]

    return nRuns, mCLogger, outputLists

def _runSimulations_SingleThreaded(simDefinition, nRuns, outputLists, mCLogger, silent=False):
    landingLocations, apogees, maxSpeeds, flightTimes, maxHorizontalVels, flights = outputLists
    resultsToOutput = simDefinition.getValue("MonteCarlo.output")

    def postProcess(flight):
        # Save results
        landingLocations.append(flight.getLandingLocation())
        apogees.append(flight.getApogee())
        maxSpeeds.append(flight.getMaxSpeed())
        flightTimes.append(flight.getFlightTime())
        maxHorizontalVels.append(flight.getMaxHorizontalVel())
        
        if "flightPaths" in resultsToOutput:
            flight = Plotting._keepNTimeSteps(flight, 900) # Limit the number of time steps saved to avoid wasting memory
            flights.append(flight)

    ### Run simulations ###
    for i in range(nRuns):
        # Start monte carlo log entry for this sim
        mCLogger.log("\nMonte Carlo Run #{}".format(i+1))
        
        # Run sim
        simDefinition.resampleProbabilisticValues()
        simRunner = Simulation(simDefinition=simDefinition, silent=True)
        stageFlightPaths, _ = simRunner.run()
        Logging.removeLogger() # Remove the logger created by simRunner #TODO Logging needs improvement
        
        # Post process the flight of the top stage
        postProcess(stageFlightPaths[0])

def _runSimulations_Parallel(simDefinition, nRuns, outputLists, silent=False, nProcesses=1):
    '''
        Runs a probabilistic simulation a several times, collects and displays average results for common parameters
        Parallelized using [ray](https://github.com/ray-project/ray)
    '''
    import ray
    runRemoteSimulation = ray.remote(runSimulation)
    runRemoteSimulation.options(num_returns=2)

    landingLocations, apogees, maxSpeeds, flightTimes, maxHorizontalVels, flights = outputLists
    resultsToOutput = simDefinition.getValue("MonteCarlo.output")
   
    def postProcess(rayObject):
        ''' Gets sim results from worker, appends results to outputLists '''
        # Get sim results
        stagePaths, logPaths = ray.get(rayObject)
        
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

    # Create an instance of random to generate random seeds for each copy of sim definition sent to workers
    #NOTE: This means Monte Carlo repeatability does not transfer across single-threaded / parallel sims
    try:
        randomSeed = simDefinition.getValue("MonteCarlo.randomSeed")
    except KeyError:
        randomSeed = random.randrange(1e7)
    rng = random.Random(randomSeed)   

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
        simDef = deepcopy(simDefinition)
        simDef.rng = random.Random(newRandomSeed)
        simDef.resampleProbabilisticValues()

        # Start sim
        flightPathsFuture = runRemoteSimulation.remote(simDefinition=simDef, silent=True)
        runningJobs.append(flightPathsFuture)

    # Wait for remaining sims to complete
    for remainingJob in runningJobs:
        postProcess(remainingJob)

    ray.shutdown()

def _showResults(simDefinition, outputLists, mCLogger):
    landingLocations, apogees, maxSpeeds, flightTimes, maxHorizontalVels, flights = outputLists
    resultsToOutput = simDefinition.getValue("MonteCarlo.output")

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
