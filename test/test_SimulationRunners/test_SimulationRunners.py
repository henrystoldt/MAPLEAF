import os
import test.testUtilities
import unittest

import matplotlib.pyplot as plt

from MAPLEAF.IO import SimDefinition
from MAPLEAF.Main import isMonteCarloSimulation
from MAPLEAF.SimulationRunners import (ConvergenceSimRunner,
                                       OptimizingSimRunner, Simulation,
                                       runMonteCarloSimulation, WindTunnelSimulation)
from MAPLEAF.Utilities import evalExpression


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

    def test_simLoggingColumnsMatchHeaders(self):
        # Load sim definition file
        simDefinition = SimDefinition("MAPLEAF/Examples/Simulations/AdaptTimeStep.mapleaf", silent=True)
        simDefinition.setValue("SimControl.EndConditionValue", "0.02")
        test.testUtilities.setUpSimDefForMinimalRunCheck(simDefinition)

        # Generate log file from a simulation
        simDefinition.setValue("SimControl.loggingLevel", "2")
        simDefinition.fileName = "test/tempTestFileasdf.mapleaf"
        simRunner = Simulation(simDefinition=simDefinition, silent=True)
        rocket = simRunner.createRocket()

        forcesLogHeaderItemCount = len(simRunner.forceEvaluationLog[0].split())
        mainLogHeaderItemCount = len(simRunner.mainSimulationLog[-1].split())

        rocket.timeStep(0.001)

        forcesLogFirstLineItemCount = len(simRunner.forceEvaluationLog[-1].split())
        mainLogFirstLineItemCount = len(simRunner.mainSimulationLog[-1].split())

        self.assertEqual(mainLogFirstLineItemCount, mainLogHeaderItemCount)
        self.assertEqual(forcesLogHeaderItemCount, forcesLogFirstLineItemCount)

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

    def test_Optimization(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Optimization.mapleaf", silent=True)
        optSimRunner = OptimizingSimRunner(simDefinition=simDef, silent=True)

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
    
    def test_nestedOptimization(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/MultiLoopOptimization.mapleaf", silent=True)
        outerSimRunner = OptimizingSimRunner(simDefinition=simDef, silent=True)
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

    def test_initializedOptimization(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/InitializedOptimization.mapleaf", silent=True)
        opt = OptimizingSimRunner(simDefinition=simDef, silent=True)

        # Check that initial position has been loaded
        bW1 = opt.initPositions[0][0]
        self.assertAlmostEqual(bW1, 0.1)

        # Check that the other initial position has been generated and is inside the expected bands
        bW2 = opt.initPositions[1][0]
        self.assertGreaterEqual(bW2, 0.01)
        self.assertLessEqual(bW2, 0.2)

        # TODO: Test an actual run

        # Check that additional unexpected variables cause a crash
        extraVariableKey = 'Optimization.IndependentVariables.InitialParticlePositions.p1.extraVariable'
        with self.assertRaises(ValueError):
            simDef.setValue(extraVariableKey, '25')
            opt = OptimizingSimRunner(simDefinition=simDef, silent=True)            
        
        simDef.removeKey(extraVariableKey)

        # Check that an out of bounds value causes a crash
        with self.assertRaises(ValueError):
            simDef.setValue('Optimization.IndependentVariables.InitialParticlePositions.p1.bodyWeight', '0.21')
            opt = OptimizingSimRunner(simDefinition=simDef, silent=True)            

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
        windTunnelSim.runSweep()
        
        # Check number of rows in the force evaluation log (2 evaluations + 1 row of headers)
        self.assertEqual(len(windTunnelSim.forceEvaluationLog), 3)

    def tearDown(self):
        plt.cla()
        plt.clf()
