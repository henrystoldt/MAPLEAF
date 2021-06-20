import os
import shutil
import test.testUtilities
import unittest

import matplotlib.pyplot as plt
from MAPLEAF.IO import SimDefinition
from MAPLEAF.Main import isMonteCarloSimulation
from MAPLEAF.SimulationRunners import (ConvergenceSimRunner, Simulation,
                                       WindTunnelSimulation,
                                       optimizationRunnerFactory,
                                       runMonteCarloSimulation)
from MAPLEAF.Utilities import evalExpression


def runOptimization(simDef):
    # Make the optimization perform a single iteration of a simulation with only a single time step
    simDef.setValue('Optimization.ScipyMinimize.maxIterations', '1')
    simDef.setValue('Optimization.method', 'scipy.optimize.minimize Nelder-Mead')
    simDef.setValue('SimControl.EndCondition', 'Time')
    simDef.setValue('SimControl.EndConditionValue', '0.005')
    simDef.setValue('Optimization.showConvergencePlot', 'False')        
    
    optSimRunner = optimizationRunnerFactory(simDefinition=simDef, silent=True)
    optSimRunner.runOptimization()

class TestSimRunners(unittest.TestCase):
    def test_Init(self):
        with self.assertRaises(ValueError):
            shouldCrash = Simulation() # Needs a sim definition file

        shouldntCrash = Simulation("MAPLEAF/Examples/Simulations/AdaptTimeStep.mapleaf", silent=True)
        simDefinition = shouldntCrash.simDefinition
        shouldntCrash2 = Simulation(simDefinitionFilePath="MAPLEAF/Examples/Simulations/AdaptTimeStep.mapleaf", simDefinition=simDefinition, silent=True)

        #### Set up sim definition ####
        test.testUtilities.setUpSimDefForMinimalRunCheck(simDefinition)
        shouldntCrash2 = Simulation(simDefinition=simDefinition, silent=True)
        shouldntCrash2.run()

    def test_isMonteCarloSim(self):
        monteCarloSimDef = SimDefinition("MAPLEAF/Examples/Simulations/MonteCarlo.mapleaf", silent=True)
        self.assertTrue(isMonteCarloSimulation(monteCarloSimDef))

        nonMonteCarloSimDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        self.assertFalse(isMonteCarloSimulation(nonMonteCarloSimDef))

    def test_MonteCarlo(self):
        #### Set up sim definition ####
        mCSimDef = SimDefinition("MAPLEAF/Examples/Simulations/MonteCarlo.mapleaf", silent=True)

        test.testUtilities.setUpSimDefForMinimalRunCheck_MonteCarlo(mCSimDef)

        #### Run Monte Carlo Simulations ####
        runMonteCarloSimulation(simDefinition=mCSimDef, silent=True)

        # Make sure monte carlo sampling is occurring
        wind = mCSimDef.getValue("Environment.ConstantMeanWind.velocity")
        self.assertNotEqual(wind, "( 2 0 0 )")
        self.assertTrue("Environment.ConstantMeanWind.velocity_mean" in mCSimDef.dict)

        # Run second simulation, expect new wind value
        runMonteCarloSimulation(simDefinition=mCSimDef, silent=True)
        wind2 = mCSimDef.getValue("Environment.ConstantMeanWind.velocity")
        self.assertNotEqual(wind, wind2)

    def test_PSOOptimization(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Optimization.mapleaf", silent=True)
        optSimRunner = optimizationRunnerFactory(simDefinition=simDef, silent=True)

        # Check output of _loadIndependentVariables()
        self.assertEqual(optSimRunner.varKeys, [ "Rocket.Sustainer.UpperBodyTube.mass" ])        
        self.assertEqual(optSimRunner.varNames, [ "bodyWeight" ])
        self.assertEqual(optSimRunner.minVals, [ 0.01 ])
        self.assertEqual(optSimRunner.maxVals, [ 0.2 ])

        # Check out of _loadDependentVariables()
        self.assertEqual(optSimRunner.dependentVars, [ "Rocket.Sustainer.Nosecone.mass" ])        
        self.assertEqual(optSimRunner.dependentVarDefinitions, [ "!0.007506 + 0.015/bodyWeight!" ])

        # Check output of _createOptimizer()
        self.assertEqual(optSimRunner.nIterations, 20)
        self.assertEqual(optSimRunner.showConvergence, True)
        self.assertEqual(optSimRunner.optimizer.n_particles, 5)

        # Check updating independent variable values
        indVarDict = optSimRunner._updateIndependentVariableValues(simDef, [0.15] )
        self.assertEqual(simDef.getValue("Rocket.Sustainer.UpperBodyTube.mass"), "0.15")
        self.assertEqual(indVarDict, { "bodyWeight": 0.15 })

        # Check updating dependent variables values
        optSimRunner._updateDependentVariableValues(simDef, indVarDict)
        self.assertAlmostEqual(float(simDef.getValue("Rocket.Sustainer.Nosecone.mass")), 0.107506)

    def test_BatchOptimization(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/CanardsOptimization.mapleaf")        
        simRunner = optimizationRunnerFactory(simDefinition=simDef, silent=True)
    
    def test_scipyMinimize(self):
        # Test regular run
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/ScipyOptimization.mapleaf", silent=True)
        simDef.setValue('Optimization.method', 'scipy.optimize.minimize BFGS')
        runOptimization(simDef)

        # Test second method
        simDef.setValue('Optimization.method', 'scipy.optimize.minimize Nelder-Mead')
        runOptimization(simDef)

        # Test continuation
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/ScipyOptimization_continue.mapleaf", silent=True)
        runOptimization(simDef)

    def test_nestedOptimization(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/MultiLoopOptimization.mapleaf", silent=True)
        outerSimRunner = optimizationRunnerFactory(simDefinition=simDef, silent=True)
        innerSimRunner = outerSimRunner._createNestedOptimization(simDef)
        # Check output of _loadIndependentVariables()
        self.assertEqual(innerSimRunner.varKeys, [ "Rocket.Sustainer.GeneralMass.mass" ])        
        self.assertEqual(innerSimRunner.varNames, [ "GeneralMass" ])
        self.assertEqual(innerSimRunner.minVals, [ 0.0045 ])
        self.assertEqual(innerSimRunner.maxVals, [ 0.01 ])

        # Check out of _loadDependentVariables()
        self.assertEqual(innerSimRunner.dependentVars, [ "Rocket.Sustainer.AltimeterMass.mass" ])        
        self.assertEqual(innerSimRunner.dependentVarDefinitions, [ "!0.01 + 0.0000475/GeneralMass!" ])

        # Check output of _createOptimizer()
        self.assertEqual(innerSimRunner.nIterations, 5)
        self.assertEqual(innerSimRunner.showConvergence, False)
        self.assertEqual(innerSimRunner.optimizer.n_particles, 2)

        # Check updating independent variable values
        indVarDict = innerSimRunner._updateIndependentVariableValues(simDef, [0.15] )
        self.assertEqual(simDef.getValue("Rocket.Sustainer.GeneralMass.mass"), "0.15")
        self.assertEqual(indVarDict, { "GeneralMass": 0.15 })

        # Check updating dependent variables values
        innerSimRunner._updateDependentVariableValues(simDef, indVarDict)
        self.assertAlmostEqual(float(simDef.getValue("Rocket.Sustainer.AltimeterMass.mass")), 0.010316666667)

    def test_readInitialParticlePositions(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/InitializedOptimization.mapleaf", silent=True)
        opt = optimizationRunnerFactory(simDefinition=simDef, silent=True)

        # Check that initial position has been loaded
        bW1 = opt.initPositions[0][0]
        self.assertAlmostEqual(bW1, 0.1)

        # Check that the other initial position has been generated and is inside the expected bands
        bW2 = opt.initPositions[1][0]
        self.assertGreaterEqual(bW2, 0.01)
        self.assertLessEqual(bW2, 0.2)

        # Check that additional unexpected variables cause a crash
        extraVariableKey = 'Optimization.IndependentVariables.InitialParticlePositions.p1.extraVariable'
        with self.assertRaises(ValueError):
            simDef.setValue(extraVariableKey, '25')
            opt = optimizationRunnerFactory(simDefinition=simDef, silent=True)            
        
        simDef.removeKey(extraVariableKey)

        # Check that an out of bounds value causes a crash
        with self.assertRaises(ValueError):
            simDef.setValue('Optimization.IndependentVariables.InitialParticlePositions.p1.bodyWeight', '0.21')
            opt = optimizationRunnerFactory(simDefinition=simDef, silent=True)            

        # Check that specifying the position of too many particles causes a crash
        with self.assertRaises(ValueError):
            simDef.setValue('Optimization.IndependentVariables.InitialParticlePositions.p2.bodyWeight', '0.15')            
            simDef.setValue('Optimization.IndependentVariables.InitialParticlePositions.p3.bodyWeight', '0.15')            
            opt = optimizationRunnerFactory(simDefinition=simDef, silent=True)

    def test_optimizationContinuation(self):
        definitionPath = "MAPLEAF/Examples/Simulations/InitializedOptimization.mapleaf"
        simDef = SimDefinition(definitionPath, silent=True)

        # Make the optimization use a single particle, single iteration, and only a single time step
        simDef.setValue('Optimization.ParticleSwarm.nParticles', '1')
        simDef.setValue('Optimization.ParticleSwarm.nIterations', '1')
        simDef.setValue('SimControl.EndCondition', 'Time')
        simDef.setValue('SimControl.EndConditionValue', '0.005')
        simDef.setValue('Optimization.showConvergencePlot', 'False')

        opt = optimizationRunnerFactory(simDefinition=simDef, silent=True)        
        cost, pos = opt.runOptimization()

        # Make sure the position matches the specified initial position
        self.assertAlmostEqual(pos, 0.1)

        # Make sure a continue file has been created
        continuationPath = definitionPath.replace('.mapleaf', '_continue.mapleaf')
        self.assertTrue(os.path.isfile(continuationPath))

        # Check that it can be run
        restartDefinition = SimDefinition(continuationPath)
        opt = optimizationRunnerFactory(simDefinition=restartDefinition, silent=True)
        opt.runOptimization()
        
        # Clean up files
        os.remove(continuationPath)
        secondContinuationPath = continuationPath.replace('.mapleaf', '_continue.mapleaf')
        os.remove(secondContinuationPath)        

    def test_convergenceSimulations(self):
        #### Set up sim definition ####
        convSimDef = SimDefinition("MAPLEAF/Examples/Simulations/AdaptTimeStep.mapleaf", silent=True)

        test.testUtilities.setUpSimDefForMinimalRunCheck(convSimDef)

        #### Run Convergence Simulations ####
        convSR = ConvergenceSimRunner(simDefinition=convSimDef, silent=True)
        convSR.convergeSimEndPosition(simLimit=2, showPlot=False)

        tempTestFilePath = "testConvergResult.csv"
        convSR.compareClassicalIntegrationSchemes(showPlot=False, simLimit=2, integrationSchemes=["Euler"], convergenceResultFilePath=tempTestFilePath)

        convSR.simDefinition.setValue("SimControl.TimeStepAdaptation.targetError", "1e-4")
        convSR.simDefinition.setValue("SimControl.TimeStepAdaptation.controller", "PID")
        convSR.compareAdaptiveIntegrationSchemes(showPlot=False, simLimit=2, integrationSchemes=["RK12Adaptive"], convergenceResultFilePath=tempTestFilePath)

        # Remove temp file
        os.remove(tempTestFilePath)

    def test_RK45AdaptSim(self):
        # Smoke test
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        simDef.setValue("SimControl.timeDiscretization", "RK45Adaptive")
        simDef.setValue("SimControl.TimeStepAdaptation.controller", "PID")
        simRunner = Simulation(simDefinition=simDef, silent=True)
        rocket = simRunner.createRocket()

        # 6-DoF time step
        rocket.timeStep(0.05)

        # 3-DoF time step
        rocket.isUnderChute = True
        rocket._switchTo3DoF()
        rocket.timeStep(0.05)

    def test_RK4Sim(self):
        # Smoke test
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        simRunner = Simulation(simDefinition=simDef, silent=True)
        rocket = simRunner.createRocket()

        # 6-DoF time step
        rocket.timeStep(0.05)

        # 3-DoF time step
        rocket.isUnderChute = True
        rocket._switchTo3DoF()
        rocket.timeStep(0.05)

    def test_WindTunnelSim_addPoints(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf")
        windTunnelSim = WindTunnelSimulation(parametersToSweep=["Rocket.velocity"], parameterValues=[["(0 0 100)", "(0 0 200)", "(0 0 300)"]], simDefinition=simDef, silent=True)
        
        # Check that smoothline is working correctly
        windTunnelSim._addPoints(pointMultiple=4)

        expectedParamsToSweep = [ "Rocket.velocity" ]
        self.assertEqual(windTunnelSim.parametersToSweep, expectedParamsToSweep)

        expectedParamValues = [ [ "(0 0 100)", "(0.0 0.0 125.0)", "(0.0 0.0 150.0)", "(0.0 0.0 175.0)", "(0 0 200)", "(0.0 0.0 225.0)", "(0.0 0.0 250.0)","(0.0 0.0 275.0)", "(0 0 300)" ] ]
        self.assertEqual(windTunnelSim.parameterValues, expectedParamValues)

    def test_WindTunnelSim(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf")
        windTunnelSim = WindTunnelSimulation(parametersToSweep=["Rocket.velocity"], parameterValues=[["(0 0 100)", "(0 0 200)"]], simDefinition=simDef, silent=True, smoothLine="False")

        # Test running a sweep
        logfilePaths = windTunnelSim.runSweep()
        
        # Check number of rows in the force evaluation log (2 evaluations)
        self.assertEqual(len(windTunnelSim.rocketStages[0].derivativeEvaluationLog.logColumns["Time(s)"]), 2)
        
        logDirectory = os.path.dirname(logfilePaths[0])
        shutil.rmtree(logDirectory)

    def tearDown(self):
        plt.cla()
        plt.clf()
