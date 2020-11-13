import importlib
from copy import deepcopy
from statistics import mean
import sys

import matplotlib.pyplot as plt
from MAPLEAF.IO import SimDefinition, SubDictReader
from MAPLEAF.IO.simDefinition import getAbsoluteFilePath
from MAPLEAF.SimulationRunners import (RemoteSimulation, Simulation,
                                       loadSimDefinition)
from MAPLEAF.SimulationRunners.Batch import (BatchRun,
                                             _implementParameterOverrides)
from MAPLEAF.Utilities import evalExpression

__all__ = [ "OptimizingSimRunner", "ParallelOptimizingSimRunner", "BatchOptimizingSimRunner", "ParallelBatchOptimizingSimRunner", "isBatchOptimization" ]

# Check if ray is available for parallel runs
try:
    import ray
    rayAvailable = True
except ImportError:
    rayAvailable = False


def isBatchOptimization(simDefinition) -> bool:
    ''' Checks whether a simulation is a batch optimization '''
    if "Optimization.batchDefinition" in simDefinition:
        return True
    else:
        return False


#### UTILITY FUNCTIONS - used by the classes below to evaluate cost functions by running INDIVIDUAL simulations ####
def _evaluateCostFunction_SingleSimulation(costFunctionDefinition, stageFlights, logFilePaths):
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

def computeCostFunctionValue_SingleSimulation(simDefinition, costFunctionDefinition):
    runner = Simulation(simDefinition=simDefinition, silent=True)
    stageFlights, logFilePaths = runner.run()
    return _evaluateCostFunction_SingleSimulation(costFunctionDefinition, stageFlights, logFilePaths)

@ray.remote
def computeCostFunctionValue_SingleSimulation_Remote(simDefinition, costFunctionDefinition):
    return computeCostFunctionValue_SingleSimulation(simDefinition, costFunctionDefinition)

def computeCostFunctionValue_BatchSimulation(caseDefinitions, costFunctionDefinition):
    ''' Returns the average cost function value over all of the cases in the list caseDefinitions. Each case definition should be of type SimDefinition '''
    batchResults = []
    for caseDefinition in caseDefinitions:
        batchResults.append(computeCostFunctionValue_SingleSimulation(caseDefinition, costFunctionDefinition))

    # Returns the average of the results for all of the cases in the batch file
    return mean(batchResults)

@ray.remote
def computeCostFunctionValue_BatchSimulation_Remote(caseDefinitions, costFunctionDefinition):
    ''' Simple wrapper function around the non remote version of this function '''
    return computeCostFunctionValue_BatchSimulation(caseDefinitions, costFunctionDefinition)


#### CLASSES THAT RUN OPTIMIZATIONS ####
class OptimizingSimRunner():
    '''
        Glue code to make MAPLEAF serve as a metric/cost function calculator for particle-swarm optimization using PySwarms.
        Configurable using the top-level 'Optimization' dictionary in .mapleaf files
    '''
    #### Initialization ####
    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False):
        ''' 
            Pass in nProcesses > 1 to run Optimization in parallel using [ray](https://github.com/ray-project/ray).
            At the time of this writing, Ray is not yet fully supported on Windows, so this option is intended primarily for Linux and Mac computers.
        '''
        self.silent = silent
        self.simDefinition = loadSimDefinition(simDefinitionFilePath, simDefinition, silent)

        if not silent:
            print("Particle Swarm Optimization")

        # Ensure no output is produced during each simulation (cost function evaluation)
        self.simDefinition.setValue("SimControl.plot", "None")
        self.simDefinition.setValue("SimControl.RocketPlot", "Off")

        # Parse the simulation definition's Optimization dictionary, but don't run it yet
        self.varKeys, self.varNames, self.minVals, self.maxVals = self._loadIndependentVariables()
        self.dependentVars, self.dependentVarDefinitions = self._loadDependentVariables()
        self.optimizer, self.nIterations, self.showConvergence = self._createOptimizer()

        self.costFunctionDefinition = self.simDefinition.getValue("Optimization.costFunction")      

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

        for key in self.simDefinition.getSubKeys("Optimization.IndependentVariables"):
            # Value expected to be 'min < key.Path < max'
            # Split into three parts using the '<' characters
            strings = self.simDefinition.getValue(key).split('<')

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
                print("{} < {} < {}".format(minVals[i], varNames[i], maxVals[i]))
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

        for depVar in self.simDefinition.getSubKeys("Optimization.DependentVariables"):
            # Value expected: [paramName]  [paramDefinitionString]
            depVarKey = depVar.replace("Optimization.DependentVariables.", "")
            depVarNames.append(depVarKey)
            depVarDefinitions.append(self.simDefinition.getValue(depVar))

        if not self.silent and len(depVarNames) > 0:
            # Output results to console
            print("Dependent variables:")
            for i in range(len(depVarNames)):
                print("{} = {}".format(depVarNames[i], depVarDefinitions[i]))
            print("")

        return depVarNames, depVarDefinitions

    def _createOptimizer(self):
        ''' 
            Reads the Optimization.ParticleSwarm dictionary and creates a pyswarms.GlobalBestPSO object 
            Returns the Optimizer, the user's desired number of iterations, and showConvergence (bool)
        '''
        pSwarmReader = SubDictReader("Optimization.ParticleSwarm", self.simDefinition)

        nParticles = pSwarmReader.getInt("nParticles")
        nIterations = pSwarmReader.getInt("nIterations")
        
        c1 = pSwarmReader.getFloat("cognitiveParam")
        c2 = pSwarmReader.getFloat("socialParam")
        w = pSwarmReader.getFloat("inertiaParam")
        pySwarmOptions = { 'c1':c1, 'c2':c2, 'w':w }

        nVars = len(self.varNames)
        varBounds = (self.minVals, self.maxVals)

        from pyswarms.single import \
            GlobalBestPSO  # Import here because for most sims it's not required
        optimizer = GlobalBestPSO(nParticles, nVars, pySwarmOptions, bounds=varBounds)

        showConvergence = pSwarmReader.getBool("Optimization.showConvergencePlot")

        if not self.silent:
            print("Optimization Parameters:")
            print("{} Particles".format(nParticles))
            print("{} Iterations".format(nIterations))
            print("c1 = {}, c2 = {}, w = {}\n".format(c1, c2, w))
            
            costFunctionDefinition = self.simDefinition.getValue("Optimization.costFunction")
            print("Cost Function:")
            print(costFunctionDefinition + "\n")

        return optimizer, nIterations, showConvergence

    #### Running the optimization ####    
    def _evaluateTrialSolution(self, simDefinition, trialSolution):
        # Create new sim definition
        simDef = deepcopy(simDefinition)
        
        # Update variable values
        varDict = self._updateIndependentVariableValues(simDef, trialSolution)
        self._updateDependentVariableValues(simDef, varDict)

        return computeCostFunctionValue_SingleSimulation(simDef, self.costFunctionDefinition)

    def _evaluateTrialSolutions(self, trialSolutions):
        ''' 
            trialSolutions: (list[list[float]]) Array of particle positions. Each position represents a trial solution in n-dimensional space. Where n = number of independent variables.
            Returns a list of n cost function values, one for each trial solution.
        '''
        costs = []
        for trialSolution in trialSolutions:
            costs.append(self._evaluateTrialSolution(self.simDefinition, trialSolution))

        return costs

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

    #### Main Function ####
    def runOptimization(self):
        ''' Run the Optimization and show convergence history '''                
        self.optimizer.optimize(self._evaluateTrialSolutions, iters=self.nIterations)
        
        if self.showConvergence:
            print("Showing optimization convergence plot")

            # Show optimization history
            from pyswarms.utils.plotters import plot_cost_history
            plot_cost_history(self.optimizer.cost_history)
            plt.show()

class ParallelOptimizingSimRunner(OptimizingSimRunner):
    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False, nCores=1):
        super().__init__(simDefinitionFilePath, simDefinition, silent)
        self.nCores = nCores

    def _evaluateTrialSolution(self, trialSolution):
        # Create new sim definition
        simDef = deepcopy(self.simDefinition)
        
        # Update variable values
        varDict = self._updateIndependentVariableValues(simDef, trialSolution)
        self._updateDependentVariableValues(simDef, varDict)

        # Run the simulation, return the future
        return computeCostFunctionValue_SingleSimulation_Remote.remote(simDef, self.costFunctionDefinition)

    def _evaluateTrialSolutions(self, trialSolutions):
        ''' Given a values the independent variable, returns the cost function value '''
        results = []
        resultFutures = []
        
        nSims = len(trialSolutions)
        for i in range(nSims):
            # Don't start more sims than we have cores
            if i >= self.nCores:
                # Post-process sims in order to ensure order of results matches order of inputs
                index = i-self.nCores
                results.append(ray.get(resultFutures[index]))

            # Start a new simulation, save the future
            resultFutures.append(self._evaluateTrialSolution(trialSolutions[i]))

        # Post process remaining sims
        nRemaining = min(nSims, self.nCores)
        if nRemaining > 0:
            for i in range(nRemaining):
                index = i - nRemaining
                results.append(ray.get(resultFutures[index]))

        return results

    def runOptimization(self):
        if self.nCores > 1 and rayAvailable:
            ray.init()
            self.optimizer.optimize(self._evaluateTrialSolutions, iters=self.nIterations)
            ray.shutdown()
        
        else:
            if self.nCores > 1 and not rayAvailable:
                print("ERROR: ray not found. Reverting to single-threaded mode.")
                
            self.optimizer.optimize(super()._evaluateTrialSolutions, iters=self.nIterations)
        
        if self.showConvergence:
            print("Showing optimization convergence plot")

            # Show optimization history
            from pyswarms.utils.plotters import plot_cost_history
            plot_cost_history(self.optimizer.cost_history)
            plt.show()

class BatchOptimizingSimRunner(OptimizingSimRunner):

    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False):
        super().__init__(simDefinitionFilePath, simDefinition, silent)

        if not self.silent:
            print("Batch Optimization")

        # Optimization dictionary now requires an entry called batchDefinition
            # TODO: Rename/re-locate this to wherever you want, and put it in the SimDefinitionTemplate
            # In this case the 'simDefinition' might only contain an Optimization dictionary?
                # Could also contain the Rocket definition, if the same 'simDefinition' file is referenced in the Batch Definition
        batchDefinitionPath = self.simDefinition.getValue("Optimization.batchDefinition")
        batchDefinitionPath = getAbsoluteFilePath(batchDefinitionPath) # Make sure path points to a real file, turn it into an absolute path
        
        # Load batch definition file
        batchDefinition = SimDefinition(batchDefinitionPath)

        # Prep and store the cases it references
        self.batchCaseDefinitions = []

        caseDictNames = batchDefinition.getImmediateSubDicts("") # Each root-level dictionary represents a case
        print("Found the following cases:")
        for case in caseDictNames:
            print(case, file=sys.__stdout__)

            # Get sim definition file path
            simDefPath = batchDefinition.getValue("{}.simDefinitionFile".format(case))
            simDefPath = getAbsoluteFilePath(simDefPath)

            # Load it, implement overrides
            caseDefinition = SimDefinition(simDefPath, silent=True)
            _implementParameterOverrides(case, batchDefinition, caseDefinition)

            # Save
            self.batchCaseDefinitions.append(caseDefinition)       

        print("") # Add some white space

    def _evaluateTrialSolution(self, _, trialSolution):
        ''' Computes a cost function value for a single trial solution '''
        simulationDefinitions = []
        for caseDefinition in self.batchCaseDefinitions:                
            simDef = deepcopy(caseDefinition)
        
            # Update variable values
            varDict = self._updateIndependentVariableValues(simDef, trialSolution)
            self._updateDependentVariableValues(simDef, varDict)
            simulationDefinitions.append(simDef)

        return computeCostFunctionValue_BatchSimulation(simulationDefinitions, self.costFunctionDefinition)

class ParallelBatchOptimizingSimRunner(BatchOptimizingSimRunner, ParallelOptimizingSimRunner):
    ''' 
        Parallelizes optimization by running one batch evaluation per core 
        Inherits the parallel run optimization function from the parallel simulation runner
    '''
    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False, nCores=1):
        BatchOptimizingSimRunner.__init__(self, simDefinitionFilePath, simDefinition, silent) # Load the batch definition file
        self.nCores = nCores # Save the number of cores for the parallel run

    def _evaluateTrialSolution(self, _, trialSolution):
        ''' Computes a cost function value for a single trial solution '''
        simulationDefinitions = []
        for caseDefinition in self.batchCaseDefinitions:                
            simDef = deepcopy(caseDefinition)
        
            # Update variable values
            varDict = self._updateIndependentVariableValues(simDef, trialSolution)
            self._updateDependentVariableValues(simDef, varDict)
            simulationDefinitions.append(simDef)

        return computeCostFunctionValue_BatchSimulation_Remote.remote(simulationDefinitions, self.costFunctionDefinition)

    def _evaluateTrialSolutions(self, trialSolutions):
        results = []
        resultFutures = []
        
        nSims = len(trialSolutions)
        for i in range(nSims):
            # Don't start more sims than we have cores
            if i >= self.nCores:
                # Post-process sims in order to ensure order of results matches order of inputs
                index = i-self.nCores
                results.append(ray.get(resultFutures[index]))

            # Start a new simulation, save the future
            resultFutureID = self._evaluateTrialSolution(self, trialSolutions[i])
            resultFutures.append(resultFutureID)

        # Post process remaining sims
        nRemaining = min(nSims, self.nCores)
        if nRemaining > 0:
            for i in range(nRemaining):
                index = i - nRemaining
                results.append(ray.get(resultFutures[index]))

        return results