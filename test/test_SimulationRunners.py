#Created by: Henry Stoldt
#May 2020

#To run tests:
#In this file: [test_ Main.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest
import os

import matplotlib.pyplot as plt

from MAPLEAF.SimulationRunners import SingleSimRunner, MonteCarloSimRunner, ConvergenceSimRunner, isMonteCarloSimulation
from MAPLEAF.IO.SimDefinition import SimDefinition
import test.testUtilities

class TestSimRunners(unittest.TestCase):
    def test_Init(self):
        with self.assertRaises(ValueError):
            shouldCrash = SingleSimRunner() # Needs a sim definition file

        shouldntCrash = SingleSimRunner("test/simDefinitions/AdaptTimeStep.mapleaf")
        
        simDefinition = SimDefinition("test/simDefinitions/AdaptTimeStep.mapleaf", silent=True)
        shouldntCrash2 = SingleSimRunner(simDefinitionFilePath="test/simDefinitions/AdaptTimeStep.mapleaf", simDefinition=simDefinition, silent=True)

        #### Set up sim definition ####
        simDefinition = SimDefinition("test/simDefinitions/AdaptTimeStep.mapleaf", silent=True)

        test.testUtilities.setUpSimDefForMinimalRunCheck(simDefinition)

        shouldntCrash2 = SingleSimRunner(simDefinition=simDefinition, silent=True)
        shouldntCrash2.runSingleSimulation()

    def test_simLoggingColumnsMatchHeaders(self):
        # Load sim definition file
        simDefinition = SimDefinition("test/simDefinitions/AdaptTimeStep.mapleaf", silent=True)
        simDefinition.setValue("SimControl.EndConditionValue", "0.02")
        test.testUtilities.setUpSimDefForMinimalRunCheck(simDefinition)

        # Generate log file from a simulation
        simDefinition.setValue("SimControl.loggingLevel", "2")
        simDefinition.fileName = "test/tempTestFileasdf.mapleaf"
        simRunner = SingleSimRunner(simDefinition=simDefinition, silent=True) # Not silent so we can capture main sim log
        rocket = simRunner.prepRocketForSingleSimulation()

        forcesLogHeaderItemCount = len(simRunner.forceEvaluationLog[0].split())
        mainLogHeaderItemCount = len(simRunner.mainSimulationLog[-1].split())

        rocket.timeStep(0.001)

        forcesLogFirstLineItemCount = len(simRunner.forceEvaluationLog[-1].split())
        mainLogFirstLineItemCount = len(simRunner.mainSimulationLog[-1].split())

        self.assertEqual(mainLogFirstLineItemCount, mainLogHeaderItemCount)
        self.assertEqual(forcesLogHeaderItemCount, forcesLogFirstLineItemCount)

    def test_isMonteCarloSim(self):
        monteCarloSimDef = SimDefinition("test/simDefinitions/MonteCarlo.mapleaf", silent=True)
        self.assertTrue(isMonteCarloSimulation(monteCarloSimDef))

        nonMonteCarloSimDef = SimDefinition("test/simDefinitions/Wind.mapleaf", silent=True)
        self.assertFalse(isMonteCarloSimulation(nonMonteCarloSimDef))

    def test_MonteCarlo(self):
        #### Set up sim definition ####
        mCSimDef = SimDefinition("test/simDefinitions/MonteCarlo.mapleaf", silent=True)

        test.testUtilities.setUpSimDefForMinimalRunCheck_MonteCarlo(mCSimDef)

        #### Run Monte Carlo Simulations ####
        mCSR = MonteCarloSimRunner(simDefinition=mCSimDef, silent=True)
        mCSR.runMonteCarloSimulation()

    def test_convergenceSimulations(self):
        #### Set up sim definition ####
        convSimDef = SimDefinition("test/simDefinitions/AdaptTimeStep.mapleaf", silent=True)

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
        simDef = SimDefinition("test/simDefinitions/Wind.mapleaf", silent=True)
        simDef.setValue("SimControl.timeDiscretization", "RK45Adaptive")
        simDef.setValue("SimControl.TimeStepAdaptation.controller", "PID")
        simRunner = SingleSimRunner(simDefinition=simDef, silent=True)
        rocket = simRunner.prepRocketForSingleSimulation()

        # 6-DoF time step
        rocket.timeStep(0.05)

        # 3-DoF time step
        rocket.isUnderChute = True
        rocket._switchTo3DoF()
        rocket.timeStep(0.05)

    def test_RK4Sim(self):
        # Smoke test
        simDef = SimDefinition("test/simDefinitions/Wind.mapleaf", silent=True)
        simRunner = SingleSimRunner(simDefinition=simDef, silent=True)
        rocket = simRunner.prepRocketForSingleSimulation()

        # 6-DoF time step
        rocket.timeStep(0.05)

        # 3-DoF time step
        rocket.isUnderChute = True
        rocket._switchTo3DoF()
        rocket.timeStep(0.05)

    def tearDown(self):
        plt.cla()
        plt.clf()