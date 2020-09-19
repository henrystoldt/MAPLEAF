''' 
Script to run flight simulations from the command line 
If MAPLEAF has been installed with pip, this script is accessible through the 'mapleaf' command
'''

import argparse
import os
import sys
import time
from typing import List

import MAPLEAF.IO.Logging as Logging
import MAPLEAF.IO.Plotting as Plotting
from MAPLEAF.IO import SimDefinition, getAbsoluteFilePath
from MAPLEAF.SimulationRunners import (ConvergenceSimRunner, runMonteCarloSimulation,
                                   SingleSimRunner, OptimizingSimRunner)


def buildParser() -> argparse.ArgumentParser:
    ''' Builds the command-line argument parser using argparse '''
    parser = argparse.ArgumentParser(description="""
    Command line interface for the rocket simulator 
    Runs simulations defined by simulation definition files like those in ./MAPLEAF/Examples/Simulations/ 
    All possible options for sim definition files defined in ./SimDefinitionTemplate.txt
    """)

    parser.add_argument(
        "--converge", 
        action='store_true', 
        help="Runs the current simulation using successively finer time steps, attempting to provide a converged final location"
    )
    parser.add_argument(
        "--compareIntegrationSchemes", 
        action='store_true', 
        help="Attempts to converge the current simulation using a variety of classical integration schemes."
    )
    parser.add_argument(
        "--compareAdaptiveIntegrationSchemes", 
        action='store_true', 
        help="Attempts to converge the current simulation using a variety of adaptive time integration schemes"
    )
    parser.add_argument(
        "--plotFromLog", 
        nargs=1, 
        default=[], 
        help="Instead of a path to a sim definition file, provide a path to a log file from a previous simulation. Also provide \
            a plotDefinitionString - works the same way the SimControl.plot entries work"
    )
    parser.add_argument(
        "--nCores",
        type=int,
        nargs=1,
        default=[1],
        help="Set this to a number > 1 to run Monte Carlo or Optimization studies in parallel using ray. Check whether ray's Windows support has exited alpha, or use only on Linux/Mac."
    )
    parser.add_argument(
        "--silent", 
        action='store_true', 
        help="If present, simulation(s) are run w/o outputting to console - which can be significantly faster on some setups"
    )
    parser.add_argument(
        "simDefinitionFile", 
        nargs=1, 
        default="MAPLEAF/Examples/Simulations/NASATwoStagOrbitalRocket.mapleaf",
        help="Path to a simulation definition (.mapleaf) file"
    )

    return parser

def checkForMutuallyExclusiveArgs(args):
    ''' Check that we haven't passed in mutually exclusive command-line arguments (ex. --converge and --plotFromLog) '''
    mutuallyExclusiveArgs = [ args.converge, args.compareIntegrationSchemes, args.compareAdaptiveIntegrationSchemes, args.plotFromLog ]
    mutExCount = 0
    for item in mutuallyExclusiveArgs:
        if item == True:
            mutExCount += 1
    if mutExCount > 1:
        print("ERROR: --converge, --compareIntegrationSchemes, --compareAdaptiveIntegrationSchemes, and --plotFromLog are mutually exclusive. Please only use one at a time.")
        sys.exit()

def findSimDefinitionFile(providedPath):
    if os.path.isfile(providedPath):
        return providedPath

    # Check if it's a relative path that needs to be made absolute
    possibleRelPath = providedPath
    absPath = getAbsoluteFilePath(possibleRelPath)
    if os.path.isfile(absPath):
        return absPath
    
    # Check if it's an example case
    if possibleRelPath[-8:] != ".mapleaf":
        # If it's just the case name (ex: 'Staging') add the file extension
        possibleRelPath += ".mapleaf"
    
    possibleRelPath = "MAPLEAF/Examples/Simulations/" + possibleRelPath
    absPath = getAbsoluteFilePath(possibleRelPath)
    if os.path.isfile(absPath):
        return absPath  

    print("ERROR: Unable to locate simulation definition file: {}!".format(providedPath))
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

def main(argv: List[str]=None) -> int:
    ''' 
        Main function to run a MAPLEAF simulation. 
        Expects to be called from the command line, usually using the `mapleaf` command
        
        For testing purposes, can also pass a list of command line arguments into the argv parameter
    '''
    startTime = time.time()
    parser = buildParser()

    args = parser.parse_args(argv)
    
    checkForMutuallyExclusiveArgs(args)    

    # Check if we actually just want to plot a column from a log file, and not run a whole simulation
    if len(args.plotFromLog):
        Plotting.plotFromLogFiles([args.simDefinitionFile[0]], args.plotFromLog[0])
        print("Exiting")
        sys.exit()
     
    # Load simulation definition file
    simDefPath = findSimDefinitionFile(args.simDefinitionFile[0])
    simDef = SimDefinition(simDefPath)

    #### Run simulation(s) ####
    # Optimization
    if isOptimizationProblem(simDef):
        optSimRunner = OptimizingSimRunner(simDefinition=simDef, silent=args.silent, nProcesses=args.nCores[0])
        optSimRunner.runOptimization()

    # Monte Carlo Sim
    elif isMonteCarloSimulation(simDef):
        runMonteCarloSimulation(simDefinition=simDef, silent=args.silent, nCores=args.nCores[0])

    # Convergence Sim
    elif args.converge or args.compareIntegrationSchemes or args.compareAdaptiveIntegrationSchemes: 
        cSimRunner = ConvergenceSimRunner(simDefinition=simDef, silent=args.silent)
        if args.converge:
            cSimRunner.convergeSimEndPosition()
        elif args.compareIntegrationSchemes:
            cSimRunner.compareClassicalIntegrationSchemes(convergenceResultFilePath='convergenceResult.csv')
        elif args.compareAdaptiveIntegrationSchemes:
            cSimRunner.compareAdaptiveIntegrationSchemes(convergenceResultFilePath='adaptiveConvergenceResult.csv')
    
    # Regular, single simulation  
    else: 
        simRunner = SingleSimRunner(simDefinition=simDef, silent=args.silent)
        simRunner.runSingleSimulation()

    Logging.removeLogger()

    print("Run time: {:1.2f} seconds".format(time.time() - startTime))
    print("Exiting")

if __name__ == "__main__":
    main()
