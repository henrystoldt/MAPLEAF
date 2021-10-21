''' 
Script to run flight simulations from the command line 
If MAPLEAF has been installed with pip, this script is accessible through the 'mapleaf' command
'''

import argparse
import os
import sys
import time
from pathlib import Path
from typing import List

import MAPLEAF.IO.Logging as Logging
import MAPLEAF.IO.Plotting as Plotting
from MAPLEAF.IO import SimDefinition, getAbsoluteFilePath
from MAPLEAF.SimulationRunners import (ConvergenceSimRunner, Simulation,
                                       optimizationRunnerFactory,
                                       runMonteCarloSimulation)
from MAPLEAF.SimulationRunners.Batch import main as batchMain


def buildParser() -> argparse.ArgumentParser:
    ''' Builds the command-line argument parser using argparse '''
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description="""
    Run individual MAPLEAF simulations.
    Expects simulations to be defined by simulation definition files like those in ./MAPLEAF/Examples/Simulations 
    See ./SimDefinitionTemplate.mapleaf for definition of all possible options
    """)

    mutexGroup = parser.add_mutually_exclusive_group()
    mutexGroup.add_argument(
        "--converge", 
        action='store_true', 
        help="Runs the current simulation using successively finer time steps, attempting to provide a converged final location"
    )
    mutexGroup.add_argument(
        "--compareIntegrationSchemes", 
        action='store_true', 
        help="Attempts to converge the current simulation using a variety of classical integration schemes."
    )
    mutexGroup.add_argument(
        "--compareAdaptiveIntegrationSchemes", 
        action='store_true', 
        help="Attempts to converge the current simulation using a variety of adaptive time integration schemes"
    )
    mutexGroup.add_argument(
        "--plotFromLog", 
        nargs=2, 
        default=[], 
        metavar=("plotDefinition", "pathToLogFile"),
        help="plotDefinition works the same way as SimControl.plot entries in simulation definition files"
    )

    parser.add_argument(
        "--parallel",
        action='store_true',
        help="Use to run Monte Carlo or Optimization studies in parallel using ray. Check whether ray's Windows support has exited alpha, or use only on Linux/Mac."
    )
    parser.add_argument(
        "--silent", 
        action='store_true', 
        help="If present, does not output to console - faster on windows"
    )
    parser.add_argument(
        "simDefinitionFile", 
        nargs='?', 
        default="NASATwoStageOrbitalRocket.mapleaf",
        help="Path to a simulation definition (.mapleaf) file. Not required if using --plotFromLog"
    )

    return parser

def findSimDefinitionFile(providedPath):
    # It is already a path, just return it
    if os.path.isfile(providedPath):
        return providedPath

    # Track if it is a relative path, relative to a different location than the current terminal (example/default cases)
    installationLocation = Path(__file__).parent.parent
    alternateLocations = [
        installationLocation / "MAPLEAF/Examples/Simulations/", 
        installationLocation / "MAPLEAF/Examples/BatchSims/" 
    ]

    possibleRelativePaths = [ providedPath ]
    
    if len(providedPath) < 8 or providedPath[-8:] != ".mapleaf":
        # If it's just the case name (ex: 'Staging') try also adding the file extension
        possibleRelativePaths = [ providedPath, providedPath + ".mapleaf" ]

    for path in possibleRelativePaths:
        for alternateLocation in alternateLocations:
            absPath = getAbsoluteFilePath(path, alternateLocation, silent=True)

            if os.path.isfile(absPath):
                # We've located the file!
                return absPath

    print("ERROR: Unable to locate simulation definition file: {}! Checked whether the path was relative to the current command line location, the MAPLEAF installation directory, or one of the example cases. To be sure that your file will be found, try using an absolute path.".format(providedPath))
    sys.exit()

def isOptimizationProblem(simDefinition) -> bool:
    try:
        simDefinition.getValue("Optimization.costFunction")
        return True
    except KeyError:
        return False

def isMonteCarloSimulation(simDefinition) -> bool:
    try:
        nRuns = float(simDefinition.getValue("MonteCarlo.numberRuns"))
        if nRuns > 1:
            return True
    except (KeyError, ValueError):
        pass

    return False

def isBatchSim(batchDefinition) -> bool:
    ''' Checks whether the file does not contain a 'Rocket' dictionary, and instead contains dictionaries that have a simDefinitionFile key  '''
    rootDicts = batchDefinition.getImmediateSubDicts("")

    for rootDict in rootDicts:
        if rootDict == 'Rocket' and 'Rocket.simDefinitionFile' not in batchDefinition:
            return False
    
    return True

def main(argv=None) -> int:
    ''' 
        Main function to run a MAPLEAF simulation. 
        Expects to be called from the command line, usually using the `mapleaf` command
        
        For testing purposes, can also pass a list of command line arguments into the argv parameter
    '''
    startTime = time.time()

    # Parse command line call, check for errors
    parser = buildParser()
    args = parser.parse_args(argv) 

    if len(args.plotFromLog):
        # Just plot a column from a log file, and not run a whole simulation
        Plotting.plotFromLogFiles([args.plotFromLog[1]], args.plotFromLog[0])
        print("Exiting")
        sys.exit()
     
    # Load simulation definition file
    simDefPath = findSimDefinitionFile(args.simDefinitionFile)
    simDef = SimDefinition(simDefPath)

    #### Run simulation(s) ####
    if args.parallel:
        try:
            import ray
        except:
            print("""
            Error importing ray. 
            Ensure ray is installed (`pip install -U ray`) and importable (`import ray` should not throw an error). 
            If on windows, consider trying Linux or running in WSL, at the time this was written, ray on windows was still in beta and unreliable.
            Alternatively, run without parallelization.
            """)

    if isOptimizationProblem(simDef):
        optSimRunner = optimizationRunnerFactory(simDefinition=simDef, silent=args.silent, parallel=args.parallel)
        optSimRunner.runOptimization()

    elif isMonteCarloSimulation(simDef):        
        if not args.parallel:
            nCores = 1
        else:
            import multiprocessing
            nCores = multiprocessing.cpu_count()

        runMonteCarloSimulation(simDefinition=simDef, silent=args.silent, nCores=nCores)

    elif args.parallel:
        raise ValueError("ERROR: Can only run Monte Carlo of Optimization-type simulations in multi-threaded mode. Support for multi-threaded batch simulations coming soon.")

    elif isBatchSim(simDef):
        print("Batch Simulation\n")
        batchMain([ simDef.fileName ])

    elif args.converge or args.compareIntegrationSchemes or args.compareAdaptiveIntegrationSchemes: 
        cSimRunner = ConvergenceSimRunner(simDefinition=simDef, silent=args.silent)
        if args.converge:
            cSimRunner.convergeSimEndPosition()
        elif args.compareIntegrationSchemes:
            cSimRunner.compareClassicalIntegrationSchemes(convergenceResultFilePath='convergenceResult.csv')
        elif args.compareAdaptiveIntegrationSchemes:
            cSimRunner.compareAdaptiveIntegrationSchemes(convergenceResultFilePath='adaptiveConvergenceResult.csv')
    
    else: 
        # Run a regular, single simulation  
        sim = Simulation(simDefinition=simDef, silent=args.silent)
        sim.run()

    Logging.removeLogger()

    print("Run time: {:1.2f} seconds".format(time.time() - startTime))
    print("Exiting")

if __name__ == "__main__":
    main()
