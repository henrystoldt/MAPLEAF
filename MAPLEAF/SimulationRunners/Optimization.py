import importlib
from copy import deepcopy
from statistics import mean

import matplotlib.pyplot as plt
from MAPLEAF.IO import SimDefinition, SubDictReader
from MAPLEAF.IO.simDefinition import getAbsoluteFilePath
from MAPLEAF.SimulationRunners import (RemoteSimulation, Simulation,
                                       loadSimDefinition)
from MAPLEAF.SimulationRunners.Batch import (BatchRun,
                                             _implementParameterOverrides)
from MAPLEAF.Utilities import evalExpression

__all__ = [ "OptimizingSimRunner", "ParallelOptimizingSimRunner", "BatchOptimizingSimRunner" ]

try:
    import ray
    rayAvailable = True
except ImportError:
    rayAvailable = False

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

        if not self.silent:
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
    def _runSim(self, simDefinition, indVarValues):
        # Create new sim definition
        simDef = deepcopy(simDefinition)
        
        # Update variable values
        varDict = self._updateIndependentVariableValues(simDef, indVarValues)
        self._updateDependentVariableValues(simDef, varDict)

        # Run the simulation
        simRunner = Simulation(simDefinition=simDef, silent=True)
        stageFlights, logFilePaths = simRunner.run()

        return stageFlights, logFilePaths

    def _postProcessSim(self, logFilePaths, stageFlights):
        if ":" in self.costFunctionDefinition:
            # Cost function is expected to be a custom function defined in an importable module
            modulePath, funcName = self.costFunctionDefinition.split(':')

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
            return evalExpression(self.costFunctionDefinition, varVals)

    def _computeCostFunctionValues(self, trialSolutions):
        ''' 
            trialSolutions: (list[list[float]]) Array of particle positions. Each position represents a trial solution in n-dimensional space. Where n = number of independent variables.
            Returns a list of n cost function values, one for each trial solution.
        '''
        results = []
        for indVarValues in trialSolutions:
            stageFlights, logFilePaths = self._runSim(self.simDefinition, indVarValues)
            costFunctionValue = self._postProcessSim(logFilePaths, stageFlights) 
            results.append(costFunctionValue)

        return results

    def _updateIndependentVariableValues(self, simDefinition, indVarValues):
        ''' 
            Updates simDefinition with the independent variable values
            Returns a dictionary map of independent variable names mapped to their values, suitable for passing to eval
        '''
        # Independent variable values
        indVarValueDict = {}
        for i in range(len(indVarValues)):
            simDefinition.setValue(self.varKeys[i], str(indVarValues[i]))
            
            varName = self.varNames[i]
            indVarValueDict[varName] = indVarValues[i]

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
        self.optimizer.optimize(self._computeCostFunctionValues, iters=self.nIterations)
        
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

    def _runSim(self, indVarValues):
        # Create new sim definition
        simDef = deepcopy(self.simDefinition)
        
        # Update variable values
        varDict = self._updateIndependentVariableValues(simDef, indVarValues)
        self._updateDependentVariableValues(simDef, varDict)

        # Run the simulation
        simRunner = RemoteSimulation.remote(simDefinition=simDef, silent=True)
        rayFlightPathsID, rayLogPathsID = simRunner.run.remote()

        return rayFlightPathsID, rayLogPathsID

    def _postProcessSim(self, rayFlightPathsID, rayLogPathsID) -> float:
        if ":" in self.costFunctionDefinition:
            # Get log file paths results
            logFilePaths = ray.get(rayLogPathsID)

            # Cost function is expected to be a custom function defined in an importable module
            modulePath, funcName = self.costFunctionDefinition.split(':')

            customModule = importlib.import_module(modulePath)
            customCostFunction = getattr(customModule, funcName)

            # Call the user's custom function, passing in the paths to all log files from the present run
            # User's function is expected to return a scalar value           
            return float( customCostFunction(logFilePaths) )

        else:
            # Get flight path results
            stageFlights = ray.get(rayFlightPathsID)

            # Cost function is expected to be an anonymous function defined in costFunctionDefinition
            topStageFlight = stageFlights[0]
            varVals = {
                "flightTime":   topStageFlight.getFlightTime(),
                "apogee":       topStageFlight.getApogee(),
                "maxSpeed":     topStageFlight.getMaxSpeed(),
                "maxHorizontalVel": topStageFlight.getMaxHorizontalVel(),
            }
            return evalExpression(self.costFunctionDefinition, varVals)

    def _computeCostFunctionValues(self, trialSolutions):
        ''' Given a values the independent variable, returns the cost function value '''
        results = []

        flightPathFutures = []
        logPathFutures = []
        
        nSims = len(trialSolutions)
        for i in range(nSims):
            # Don't start more sims than we have cores
            if i >= self.nCores:
                # Post-process sims in order to ensure order of results matches order of inputs
                index = i-self.nCores
                results.append(self._postProcessSim(flightPathFutures[index], logPathFutures[index]))

            # Start a new simulation
            rayFlightPathsID, rayLogPathsID = self._runSim(trialSolutions[i])
            
            # Save futures
            flightPathFutures.append(rayFlightPathsID)
            logPathFutures.append(rayLogPathsID)

        # Post process remaining sims
        nRemaining = min(nSims, self.nCores)
        if nRemaining > 0:
            for i in range(nRemaining):
                index = i - nRemaining
                results.append(self._postProcessSim(flightPathFutures[index], logPathFutures[index]))

        return results

    def runOptimization(self):
        if self.nCores > 1 and rayAvailable:
            ray.init()
            self.optimizer.optimize(self._computeCostFunctionValues, iters=self.nIterations)
            ray.shutdown()
        
        else:
            if self.nCores > 1 and not rayAvailable:
                print("ERROR: ray not found. Reverting to single-threaded mode.")
                
            self.optimizer.optimize(super()._computeCostFunctionValues, iters=self.nIterations)
        
        if self.showConvergence:
            print("Showing optimization convergence plot")

            # Show optimization history
            from pyswarms.utils.plotters import plot_cost_history
            plot_cost_history(self.optimizer.cost_history)
            plt.show()

class BatchOptimizingSimRunner(OptimizingSimRunner):

    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False):
        super().__init__(simDefinitionFilePath, simDefinition, silent)

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
        for case in caseDictNames:
            # Get sim definition file path
            simDefPath = batchDefinition.getValue("{}.simDefinitionFile".format(case))
            simDefPath = getAbsoluteFilePath(simDefPath)

            # Load it, implement overrides
            caseDefinition = SimDefinition(simDefPath, silent=True)
            _implementParameterOverrides(case, batchDefinition, caseDefinition)

            # Save
            self.batchCaseDefinitions.append(caseDefinition)       

    def _computeCostFunctionValues(self, trialSolutions):
        ''' 
            trialSolutions: (list[list[float]]) Array of particle positions. Each position represents a trial solution in n-dimensional space. Where n = number of independent variables.
            Returns a list of n cost function values, one for each trial solution.
        '''
        resultsForEachSolution = []
        for indVarValues in trialSolutions:
            
            # Calculate cost function for each case in our batch, store in batchResults
            batchResults = []
            for caseDef in self.batchCaseDefinitions:    
                
                stageFlights, logFilePaths = self._runSim(caseDef, indVarValues)
                costFunctionValue = self._postProcessSim(logFilePaths, stageFlights) 
                batchResults.append(costFunctionValue)

            # TODO: Currently using mean() here, could use some other operation to combine results from each case in the batch
            resultsForEachSolution.append(mean(batchResults))

        return resultsForEachSolution

#TODO: Parallel Batch Optimizing Runner