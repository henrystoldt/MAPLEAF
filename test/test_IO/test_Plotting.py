# Created by: Henry Stoldt
# June 2020

#To run tests:
#In this file: [python3 test/test_Plotting.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import os
import shutil
import test.testUtilities
import unittest

import MAPLEAF.IO.Plotting as Plotting
import MAPLEAF.Main as Main
import matplotlib.pyplot as plt
from MAPLEAF.IO import RocketFlight, SimDefinition
from MAPLEAF.Motion import (Quaternion, RigidBodyState, RigidBodyState_3DoF,
                            Vector)


class TestPlotting(unittest.TestCase):
    '''
        All the tests in this section really just check that the functions are not crashing
        Plotting function outputs are not checked
    '''
    
    def setUp(self):
        pos1 = Vector(0, 0, 10)
        vel1 = Vector(0, 0, -1)
        pos2 = Vector(1, 2, 5)
        vel2 = Vector(0, 0, -1)
        t1 = 0.0
        t2 = 1.0
        state1 = RigidBodyState_3DoF(pos1, vel1)
        state2 = RigidBodyState_3DoF(pos2, vel2)

        orientation1 = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        angVel1 = Vector(0,0,0)

        state3 = RigidBodyState(pos2, vel2, orientation1, angVel1)
        
        flight = RocketFlight()
        flight.rigidBodyStates = [ state1, state2, state3 ]
        flight.times = [ t1, t2, 2.0 ]
        flight.actuatorDefls = [ [2, 3, 4], [3, 4, 5], [ 4, 5, 6] ]

        self.flight = flight

    def test_plotColumn(self):
        # Load sim definition file
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/AdaptTimeStep.mapleaf", silent=True)
        test.testUtilities.setUpSimDefForMinimalRunCheck(simDef)
        simDef.setValue("SimControl.EndConditionValue", "0.03")

        # Generate log file from a simulation
        simDef.setValue("SimControl.loggingLevel", "2")
        simDef.fileName = "test/tempTestFileasdf.txt"
        simRunner = Main.Simulation(simDefinition=simDef, silent=True)
        _, logfilePaths = simRunner.run()
        
        # Try to plot one of its columns (AeroForce)
        Plotting.tryPlottingFromLog(logfilePaths[1], ["AeroF"], showPlot=False)
        ax = plt.gca()
        nLines = len(ax.lines)
        self.assertEqual(nLines, 3)

        Plotting.tryPlottingFromLog(logfilePaths[0], ["Position"], showPlot=False)
        ax = plt.gca()
        nLines = len(ax.lines)
        self.assertEqual(nLines, 3)

        # Delete temp files
        logDirectory = os.path.dirname(logfilePaths[0])
        shutil.rmtree(logDirectory)

    def test_plotFromLog(self):
        # Load sim definition file
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/AdaptTimeStep.mapleaf", silent=True)
        test.testUtilities.setUpSimDefForMinimalRunCheck(simDef)
        simDef.setValue("SimControl.EndConditionValue", "0.03")

        # Generate log file from a simulation
        simDef.setValue("SimControl.loggingLevel", "2")
        simDef.fileName = "test/tempTestFileasdf.txt"
        simRunner =  Main.Simulation(simDefinition=simDef, silent=True)
        _, logfilePaths = simRunner.run()
        
        # Try to plot one of its columns (AeroForce)
        Plotting.plotFromLogFiles(logfilePaths, "Position&^Velocity", showPlot=False)
        # The '^' in front of Velocity is for a Regex match, indicating the start of a line
            # This will prevent it from also plotting the AngularVelocity along with the Position and Velocity columns
        ax = plt.gca()
        nLines = len(ax.lines)
        self.assertEqual(nLines, 6)

        # Try to plot one of its columns (AeroForce)
        Plotting.plotFromLogFiles(logfilePaths, "Position&Velocity", showPlot=False)
        ax = plt.gca()
        nLines = len(ax.lines)
        self.assertEqual(nLines, 9)

        # Delete temp files
        logDirectory = os.path.dirname(logfilePaths[0])
        shutil.rmtree(logDirectory)

        # Clear the plot
        plt.clf()

    def test_flightAnimation(self):
        Plotting.flightAnimation([self.flight], showPlot=False)
        plt.clf()
        
        maxDimension = 1
        centerOfPlot = Vector(0,0,0)
        refAxis, perpVectors = Plotting._createReferenceVectors(3, maxDimension)
        fig, ax = Plotting._createAnimationFigure(maxDimension, centerOfPlot)
        cgPoints, flightPathLines, longitudinalRocketLines, allCanardLines, allPerpLines = Plotting._createInitialFlightAnimationPlot(ax, 3, [self.flight], refAxis, perpVectors)
        
        # Try updating the plot
        Plotting._update_plot(2, [self.flight], refAxis, perpVectors, cgPoints, flightPathLines, longitudinalRocketLines, allCanardLines, allPerpLines)

    def test_plotFlightPaths(self):
        Plotting.plotFlightPaths_NoEarth([self.flight], showPlot=False)

    def test_KeepNTimeSteps(self):
        Plotting._keepNTimeSteps([self.flight], nFramesToKeep=2)

    def test_plotAndSummarizeVectorResult(self):
        vectorList = [ Vector(0,1,2), Vector(1,2,3)]
        Plotting.plotAndSummarizeVectorResult(vectorList, showPlot=False)

    def test_plotAndSummarizeScalarResult(self):
        scalarList = [ 1, 2, 3 ]
        Plotting.plotAndSummarizeScalarResult(scalarList, showPlot=False)

    def tearDown(self):
        plt.clf()
