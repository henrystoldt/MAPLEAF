import importlib
import re
import sys
from abc import ABC, abstractmethod
from copy import deepcopy
from statistics import mean

import matplotlib.pyplot as plt
import numpy as np
from MAPLEAF.IO import Logging, SimDefinition, SubDictReader, subDictReader
from MAPLEAF.IO.simDefinition import getAbsoluteFilePath
from MAPLEAF.SimulationRunners import Simulation, loadSimDefinition
from MAPLEAF.SimulationRunners.Batch import (BatchRun,
                                             _implementParameterOverrides)
from MAPLEAF.Utilities import evalExpression
from scipy.optimize import minimize

from .SingleSimulations import Simulation, loadSimDefinition, runSimulation

__all__ = [ "optimizationRunnerFactory" ]


def _computeCostFunction(simDefinition: SimDefinition, costFunctionDefinition: str):
    """
        For a single simulation definition and cost function (defined by a string), returns the result of the cost function (typically a number)
        The version here is single threaded, remote versions are created in OptimizingSimRunner._computeCostFunctionValues_Parallel using ray.remote
            if running a parallel optimization
    """
    # Run the simulation
    stageFlights, logFilePaths = runSimulation(simDefinition=simDefinition, silent=True)
    Logging.removeLogger()

    # Evaluate the cost function
    if ":" in costFunctionDefinition:
        # Cost function is expected to be a custom function defined in an importable module
        modulePath, funcName = costFunctionDefinition.split(':')

        customModule = importlib.import_module(modulePath)
        customCostFunction = getattr(customModule, funcName)

        # Call the user's custom function, passing in the paths to all log files from the present run
        # User's function is expected to return a scalar value           
        return float( customCostFunction(logFilePaths) )

    else:
        # Cost function is expected to be an anonymous function defined in costFunctionDefinition
        topStageFlight = stageFlights[0]
        varVals = {
            "flightTime":   topStageFlight.getFlightTime(),
            "apogee":       topStageFlight.getApogee(),
            "maxSpeed":     topStageFlight.getMaxSpeed(),
            "maxHorizontalVel": topStageFlight.getMaxHorizontalVel(),
        }

        return evalExpression(costFunctionDefinition, varVals)

#### CLASSES THAT RUN OPTIMIZATIONS ####
class OptimizingSimRunner(ABC):
    '''
        Abstract base class for optimizers in mapleaf
        Reads the top-level 'Optimization' dictionary in .mapleaf files
        Child classes implement actual optimization algorithms (ex. particle swarm, scipy optimizers)
    '''
    #### Initialization ####
    def __init__(self, optimizationReader, silent=False, parallel=False):
        ''' 
            Pass in nProcesses > 1 to run Optimization in parallel using [ray](https://github.com/ray-project/ray).
            At the time of this writing, Ray is not yet fully supported on Windows, so this option is intended primarily for Linux and Mac computers.
        '''
        self.silent = silent
        self.parallel = parallel
        self.optimizationReader = optimizationReader

        # Parse the simulation definition's Optimization dictionary, but don't run it yet
        self.costFunctionDefinition = optimizationReader.getString("costFunction")
        self.varKeys, self.varNames, self.minVals, self.maxVals = self._loadIndependentVariables()
        self.dependentVars, self.dependentVarDefinitions = self._loadDependentVariables()
        self.bestPosition, self.bestCost = self._loadBestPosition()

        self.costFunctionDefinition = optimizationReader.getString("Optimization.costFunction")
        self.simDefinitions = [ optimizationReader.simDefinition ]

        # Check if this is a batch optimization by looking for the batch file entry in the sim definition file
        result = optimizationReader.tryGetString("batchDefinition", defaultValue="notPresent.fakeFile")
        if result != "notPresent.fakeFile":
            self._loadBatchOptimization()            

    def _loadIndependentVariables(self):
        ''' 
            Parses the independent variables section of Optimization dictionary.
            Returns four lists:

            * A list of string paths to the corresponding values in the simulation definition
            * Parameter names
            * A list of minimum parameter values
            * A list of maximum parameter values

            All lists are in corresponding order
        '''
        varKeys = []
        varNames = []
        minVals = []
        maxVals = []

        for key in self.optimizationReader.getImmediateSubKeys("IndependentVariables"):
            # Value expected to be 'min < key.Path < max'
            # Split into three parts using the '<' characters
            strings = self.optimizationReader.getString(key).split('<')

            if len(strings) != 3:
                # ERROR: too many or too few values
                raise ValueError("Parameters in the Optimization.IndependentVariables dictionary should be scalars and conform to the following format: '[minValue] < [path.To.Parameter] < [maxValue]' \
                                Problem key is {}, which has a value of {}".format(key, " ".join(strings)))
            
            # Remove spaces
            minVal, keyPath, maxVal = [ s.strip() for s in strings ]
            varName = key.split('.')[-1] # User's given name

            # Parse / Save
            varKeys.append(keyPath)
            varNames.append(varName)
            minVals.append(float(minVal))
            maxVals.append(float(maxVal))

        if not self.silent:
            # Output setup to console
            print("Independent Variables: ")
            for i in range(len(varNames)):
                print("    {} < {} < {}".format(minVals[i], varNames[i], maxVals[i]))
            print("")

        return varKeys, varNames, minVals, maxVals

    def _loadDependentVariables(self):
        '''
            Parses the dependent variables section of Optimization dictionary.
            Returns two lists:

            * A list of string paths to the corresponding values in the simulation definition
            * Dependent parameter names

            Both in corresponding order
        '''
        depVarNames = []
        depVarDefinitions = []

        for depVar in self.optimizationReader.getSubKeys("DependentVariables"):
            # Value expected: [paramName]  [paramDefinitionString]
            depVarKey = re.sub("Optimization.*DependentVariables.", "", depVar)
            depVarNames.append(depVarKey)
            depVarDefinitions.append(self.optimizationReader.getString(depVar))

        if not self.silent and len(depVarNames) > 0:
            # Output results to console
            print("Dependent variables:")
            for i in range(len(depVarNames)):
                print("    {} = {}".format(depVarNames[i], depVarDefinitions[i]))
            print("")

        return depVarNames, depVarDefinitions

    def _loadBestPosition(self):
        bestPosition = self.optimizationReader.tryGetString('IndependentVariables.InitialParticlePositions.bestPosition', defaultValue=None)
        bestCost = self.optimizationReader.tryGetFloat('IndependentVariables.InitialParticlePositions.bestCost', defaultValue=None)

        if bestPosition != None and bestCost != None:
            bestPosition = [ float(x) for x in bestPosition.split(' ')]
            bestPosition = np.array(bestPosition, np.float64)
            bestCost = np.float64(bestCost)

            if not self.silent:
                print("Loaded previous best position: {}".format(bestPosition))
                print("Loaded previous best cost: {}\n".format(bestCost))

            return bestPosition, bestCost
        elif bestPosition == None and bestCost == None:
            return None, None
        else:
            raise ValueError('Please specify bestPosition and bestCost or neither, current values: {} and {}, respectively'.format(bestPosition, bestCost))

    def _loadBatchOptimization(self):
        if not self.silent:
            print("Batch Optimization")

        # Optimization dictionary now requires an entry called batchDefinition
            # In this case the 'simDefinition' might only contain an Optimization dictionary
                # Could also contain the Rocket definition, if the same 'simDefinition' file is referenced in the Batch Definition
        batchDefinitionPath = self.optimizationReader.getString("Optimization.batchDefinition")
        batchDefinitionPath = getAbsoluteFilePath(batchDefinitionPath) # Make sure path points to a real file, turn it into an absolute path
        
        # Load batch definition file
        batchDefinition = SimDefinition(batchDefinitionPath)

        # Prep and store the cases it references
        self.simDefinitions = []

        caseDictNames = batchDefinition.getImmediateSubDicts("") # Each root-level dictionary represents a case
        print("Found the following cases:")
        for case in sorted(caseDictNames):
            print(case, file=sys.__stdout__)

            # Get sim definition file path
            simDefPath = batchDefinition.getValue("{}.simDefinitionFile".format(case))
            simDefPath = getAbsoluteFilePath(simDefPath)

            # Load it, implement overrides defined in the batch file
            caseDefinition = SimDefinition(simDefPath, silent=True)
            _implementParameterOverrides(case, batchDefinition, caseDefinition)

            # Save
            self.simDefinitions.append(caseDefinition)       

        print("") # Add some white space

    #### Running the optimization ####
    def _createNestedOptimization(self, simDef):
        innerOptimizationReader = SubDictReader(self.optimizationReader.simDefDictPathToReadFrom + '.InnerOptimization', simDef)
        return optimizationRunnerFactory(optimizationReader=innerOptimizationReader, silent=self.silent, parallel=self.parallel)

    def _computeCostFunctionValues_Parallel(self, trialSolutions) -> float:
        ''' Given a values the independent variable, returns the cost function value '''
        import ray
        _computeCostFunctionRemotely = ray.remote(_computeCostFunction)

        costFunctionValues = []
        
        
        for independentVariableValues in trialSolutions:
            caseResults = []
            for case in self.simDefinitions:
                # Create new sim definition
                simDef = deepcopy(case)
            
                # Update variable values
                varDict = self._updateIndependentVariableValues(simDef, independentVariableValues)
                self._updateDependentVariableValues(simDef, varDict)

                # Start the simulation and save the future returned
                caseResults.append(_computeCostFunctionRemotely.remote(simDefinition=simDef, costFunctionDefinition=self.costFunctionDefinition))
            
            costFunctionValues.append(caseResults)

        # All simulation now started, wait for and retrieve the results
        if len(self.simDefinitions) > 1:
            costFunctionValues = [ mean([ ray.get(value) for value in caseResults ]) for caseResults in costFunctionValues ]
        else:
            costFunctionValues = [ [ ray.get(value) for value in caseResults ][0] for caseResults in costFunctionValues ]

        return costFunctionValues

    def _computeCostFunctionValues_SingleThreaded(self, trialSolutions) -> float:
        ''' Given a values the independent variable, returns the cost function value '''
        costFunctionValues = []
        for independentVariableValues in trialSolutions:
            caseResults = []
            
            for case in self.simDefinitions:
                # Create new sim definition
                simDef = deepcopy(case)
                
                # Update variable values
                varDict = self._updateIndependentVariableValues(simDef, independentVariableValues)
                self._updateDependentVariableValues(simDef, varDict)

                if self.optimizationReader.simDefDictPathToReadFrom + '.InnerOptimization' in self.optimizationReader.getImmediateSubDicts():
                    innerOptimizer = self._createNestedOptimization(simDef)
                    cost, pos = innerOptimizer.runOptimization()
                    varDict = innerOptimizer._updateIndependentVariableValues(simDef, pos)
                    innerOptimizer._updateDependentVariableValues(simDef, varDict)   

                # Run the simulation and compute the cost function value, save the result
                caseResults.append(_computeCostFunction(simDef, costFunctionDefinition=self.costFunctionDefinition))

            if len(caseResults) > 1:
                costFunctionValues.append(mean(caseResults))
            else:
                costFunctionValues.append(caseResults[0])

        return costFunctionValues
    
    def _updateIndependentVariableValues(self, simDefinition, trialSolution):
        ''' 
            Updates simDefinition with the independent variable values
            Returns a dictionary map of independent variable names mapped to their values, suitable for passing to eval
        '''
        # Independent variable values
        indVarValueDict = {}
        for i in range(len(trialSolution)):
            simDefinition.setValue(self.varKeys[i], str(trialSolution[i]))
            
            varName = self.varNames[i]
            indVarValueDict[varName] = trialSolution[i]

        return indVarValueDict

    def _updateDependentVariableValues(self, simDefinition, indVarValueDict):
        ''' Set all the dependent variables defined in Optimization.DependentVariables in simDefinition. Each can be a function of the independent variable values in indVarValueDict '''
        
        for i in range(len(self.dependentVars)):
            # Take the definition string, split out the parts to be computed (delimited by exclamation marks)
                # "(0 0 !a+b!)" -> [ "(0 0", "a+b", ")" ] -> Need to evaluate all the odd-indexed values
            splitDepVarDef = self.dependentVarDefinitions[i].split('!')
            for j in range(1, len(splitDepVarDef), 2):
                functionValue = evalExpression(splitDepVarDef[j], indVarValueDict)
                # Overwrite the function definition with its string value
                splitDepVarDef[j] = str(functionValue)
            
            # Re-combine strings, save result
            depValue = "".join(splitDepVarDef)
            simDefinition.setValue(self.dependentVars[i], depValue)

    @abstractmethod
    def runOptimization(self):
        ''' Child classes must implement this method, will vary depending on the optimizer used (ex. pyswarms, scipy.minimize) '''
        pass

def optimizationRunnerFactory(simDefinitionFilePath=None, simDefinition=None, optimizationReader=None, silent=False, parallel=False) -> OptimizingSimRunner:
    ''' Provide a subdictionary reader pointed at an optimization dictionary. Will read the dictionary, initialize an optimizing simulation runner and return it. '''
    if simDefinition != None or simDefinitionFilePath != None:
        simDefinition = loadSimDefinition(simDefinitionFilePath, simDefinition, silent)
        optimizationReader = SubDictReader('Optimization', simDefinition)

        # Ensure no output is produced during each simulation (cost function evaluation)
        simDefinition.setValue("SimControl.plot", "None")
        simDefinition.setValue("SimControl.RocketPlot", "Off")
    else:
        if optimizationReader == None:
            raise ValueError('subDictReader not initialized for a nested optimization')

    method = optimizationReader.tryGetString('method', defaultValue='PSO')

    if method == 'PSO':
        return PSORunner(optimizationReader, silent=silent, parallel=parallel)
    elif 'scipy.optimize.minimize' in method:
        return ScipyMinimizeRunner(optimizationReader, silent=silent, parallel=parallel)
    else:
        raise ValueError('Optimization method: {} not implemented, try PSO'.format(method))

class PSORunner(OptimizingSimRunner):
    '''
        Implements particle swarm optimization using pyswarms
    '''
    def __init__(self, optimizationReader, silent=False, parallel=False):
        if not silent:
            print("Particle Swarm Optimization")

        super().__init__(optimizationReader, silent, parallel)
        
        self.initPositions = self._loadInitialPositions()

        # Actually create the optimizer
        self.optimizer, self.nIterations, self.showConvergence = self._createOptimizer()

    def _loadInitialPositions(self):
        nParticles = self.optimizationReader.getInt("ParticleSwarm.nParticles")
        nVars = len(self.varNames)

        particlePositionDicts = self.optimizationReader.getImmediateSubDicts("IndependentVariables.InitialParticlePositions")

        if len(particlePositionDicts) > nParticles:
            raise ValueError("Number of initial particle positions: {}({}) cannot exceed the total number of particles: {}".format(len(particlePositionDicts), particlePositionDicts, nParticles))

        initialParticlePositions = np.empty((nParticles, nVars), dtype=np.float64)
        
        for i in range(nParticles):
            if i < len(particlePositionDicts):
                # Some initialization data provided
                particleDictionary = particlePositionDicts[i]

                if i == 0 and not self.silent:
                    print("Loading specified initial particle positions")

                if not self.silent:
                    print('    ' + particleDictionary + ':')

                specifiedVariablePaths = self.optimizationReader.getImmediateSubKeys(particleDictionary)
                specifiedVariableNames = []

                # Populate name vector and Check that all provided variables exist
                for v in specifiedVariablePaths:
                    vName = v.split('.')[-1]
                    specifiedVariableNames.append(vName)

                    if vName not in self.varNames:
                        raise ValueError("Variable {} not expected in {}. Only expecting independent variables: {}".format(v, particleDictionary, self.varNames))
            else:
                # All available initialization data already used
                specifiedVariablePaths = []
                specifiedVariableNames = []

            # Read or generate value for each variable
            for j in range(nVars):
                independentVariable = self.varNames[j]

                # Read provided data if available
                if independentVariable in specifiedVariableNames:
                    index = specifiedVariableNames.index(independentVariable)
                    value = self.optimizationReader.getFloat(specifiedVariablePaths[index])

                    if not self.silent:
                        print('        {} = {}'.format(independentVariable, value))

                    # Check the value is within the expected bounds
                    if value < self.minVals[j] or value > self.maxVals[j]:
                        raise ValueError("{} value in initial particle position: {} ({}) outside of expected range ({}-{}".format(independentVariable, particleDictionary, value, self.minVals[j], self.maxVals[j]))

                # Otherwise, choose a random value
                else:
                    value = np.random.uniform(self.minVals[j], self.maxVals[j])

                initialParticlePositions[i,j] = value
        
        return initialParticlePositions

    def _createOptimizer(self):
        ''' 
            Reads the Optimization.ParticleSwarm dictionary and creates a pyswarms.GlobalBestPSO object 
            Returns the Optimizer, the user's desired number of iterations, and showConvergence (bool)
        '''
        pathToParticleSwarmDict = self.optimizationReader.simDefDictPathToReadFrom + '.ParticleSwarm'
        pSwarmReader = SubDictReader(pathToParticleSwarmDict, self.optimizationReader.simDefinition)

        nParticles = pSwarmReader.tryGetInt("nParticles", defaultValue=20)
        nIterations = pSwarmReader.tryGetInt("nIterations", defaultValue=50)
        
        c1 = pSwarmReader.tryGetFloat("cognitiveParam", defaultValue=0.5)
        c2 = pSwarmReader.tryGetFloat("socialParam", defaultValue=0.6)
        w = pSwarmReader.tryGetFloat("inertiaParam", defaultValue=0.9)
        pySwarmOptions = { 'c1':c1, 'c2':c2, 'w':w }

        nVars = len(self.varNames)
        varBounds = (self.minVals, self.maxVals)

        from pyswarms.single import \
            GlobalBestPSO  # Import here because for most sims it's not required
        optimizer = GlobalBestPSO(nParticles, nVars, pySwarmOptions, bounds=varBounds, init_pos=self.initPositions)
        
        if self.bestPosition != None and self.bestCost != None:
            optimizer.swarm.best_pos = self.bestPosition
            optimizer.swarm.best_cost = self.bestCost

        showConvergence = self.optimizationReader.tryGetBool("showConvergencePlot", defaultValue=False)

        if not self.silent:
            print("Optimization Parameters:")
            print("{} Particles".format(nParticles))
            print("{} Iterations".format(nIterations))
            print("c1 = {}, c2 = {}, w = {}\n".format(c1, c2, w))
            
            costFunctionDefinition = self.optimizationReader.getString("costFunction")
            print("Cost Function:")
            print(costFunctionDefinition + "\n")

        return optimizer, nIterations, showConvergence

    def _createContinuationFile(self, particlePositions, bestPosition, bestCost):
        # Create new simulation definition
        restartDefinition = deepcopy(self.optimizationReader.simDefinition)

        # Remove any existing initial particle positions
        oldPositionEntries = self.optimizationReader.getSubKeys('IndependentVariables.InitialParticlePositions')
        for p in oldPositionEntries:
            restartDefinition.removeKey(p)

        # Save best position and cost from present run
        path = self.optimizationReader.simDefDictPathToReadFrom
        positionString = ' '.join([ str(x) for x in bestPosition ])
        restartDefinition.setValue(path + '.IndependentVariables.InitialParticlePositions.bestPosition', positionString)
        restartDefinition.setValue(path + '.IndependentVariables.InitialParticlePositions.bestCost', str(bestCost))

        # Add the last particle positions
        for i in range(len(particlePositions)):
            position = particlePositions[i]
            
            for j in range(len(position)):
                varName = self.varNames[j]
                value = position[j]
                path = self.optimizationReader.simDefDictPathToReadFrom + '.IndependentVariables.InitialParticlePositions.p{}.{}'.format(i, varName)
                
                restartDefinition.setValue(path, str(value))

        # Save the result to file
        newFileName = restartDefinition.fileName.replace('.mapleaf', '_continue.mapleaf')
        if not self.silent:
            print('Creating continuation file: {}'.format(newFileName))
        restartDefinition.writeToFile(newFileName)

    #### Main Function ####
    def runOptimization(self):
        ''' Run the Optimization and show convergence history '''
        if self.parallel:
            import ray
            ray.init()
            cost, pos = self.optimizer.optimize(self._computeCostFunctionValues_Parallel, iters=self.nIterations)
            ray.shutdown()
        
        else:                
            cost, pos = self.optimizer.optimize(self._computeCostFunctionValues_SingleThreaded, iters=self.nIterations)

        self._createContinuationFile(self.optimizer.swarm.position, pos, cost)

        if self.showConvergence:
            print("Showing optimization convergence plot")

            # Show optimization history
            from pyswarms.utils.plotters import plot_cost_history
            plot_cost_history(self.optimizer.cost_history)
            plt.show()

        return cost, pos

class ScipyMinimizeRunner(OptimizingSimRunner):
    '''
        Implements optimization using scipy.minimize's optimization methods
    '''
    def __init__(self, optimizationReader, silent=False, parallel=False):
        method = optimizationReader.getString("method")
        splitMethod = method.split(' ')
        if len(splitMethod) > 2:
            raise ValueError('Unexpected format for optimization method. Expected scipy.optimize.minimize [method], got: {}'.format(method))

        self.minimizeMethod = splitMethod[1]

        if not silent:
            print("Optimization using scipy.optimize.minimize, method={}".format(self.minimizeMethod))

        ### Run parent initialization function ###
        super().__init__(optimizationReader, silent, parallel)

        self.showConvergence = optimizationReader.tryGetBool("showConvergencePlot", defaultValue=True)
        self.costHistory = []

    def _evaluateCostFunction(self, independentVariableValues):
        try:
            # Scipy optimizers might not respect bounds, therefore things will crash every so often
            resultInList = self._computeCostFunctionValues_SingleThreaded([independentVariableValues])
            result = resultInList[0]
        except:
            # When this is the case, return a high cost
            if not self.silent:
                print("Simulation crash, assigning high cost: ", end="")

            if len(self.costHistory) > 5:
                result = max(self.costHistory)
            elif len(self.costHistory) > 0:
                result = max(self.costHistory)*1.1
            else:
                # Crash on first iteration, should be within bounds -> crash
                raise

        self.costHistory.append(result)

        if not self.silent:
            print(result)

        return result

    def _createContinuationFile(self, currentSolution):
        # Create new simulation definition
        restartDefinition = deepcopy(self.optimizationReader.simDefinition)

        # To set the starting positions, constrain the bounds
        for i in range(len(self.varNames)):
            path = self.optimizationReader.simDefDictPathToReadFrom + '.IndependentVariables.' + self.varNames[i]
            eps = 1e-12
            value = "{} < {} < {}".format(currentSolution[i]-eps, self.varKeys[i], currentSolution[i]+eps)
            restartDefinition.setValue(path, value)

        # Save the result to file
        newFileName = restartDefinition.fileName.replace('.mapleaf', '_continue.mapleaf')
        if not self.silent:
            print('Creating continuation file: {}'.format(newFileName))
        restartDefinition.writeToFile(newFileName)

    def runOptimization(self):
        # Read options
        tolerance = self.optimizationReader.tryGetFloat('ScipyMinimize.tolerance', defaultValue=0.01)
        maxIterations = self.optimizationReader.tryGetInt('ScipyMinimize.maxIterations', defaultValue=50)
        printConvergence = self.optimizationReader.tryGetBool('ScipyMinimize.printStatus', defaultValue=True)
        if self.silent:
            printConvergence = False

        if not self.silent:
            print("Options:")
            print("    Tolerance: {}".format(tolerance))
            print("    Max Iterations: {}\n".format(maxIterations))
            print("Running optimization:")

        options = {
            "maxiter": maxIterations,
            "disp": printConvergence,
        }

        # Get initial position (centre of bounds)
        variableCount = len(self.varNames)
        initialPosition = np.array([0.0] * variableCount)
        for i in range(variableCount):
            initialPosition[i] = (self.minVals[i] + self.maxVals[i])/2

        # Perform the minimization
        result = minimize(self._evaluateCostFunction, initialPosition, method=self.minimizeMethod, options=options, tol=tolerance)

        self._createContinuationFile(result.x)

        if self.showConvergence:
            plt.plot(self.costHistory)
            plt.ylabel('Cost')
            plt.xlabel('Cost evaluation number')
            plt.show()

        if not self.silent:
            print("\nBest position: {}\nBest cost: {}\n".format(result.x, result.fun))

        return result.fun, result.x
