#Created by: Declan Quinn
#May 2019

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import math
import unittest
from copy import deepcopy
from test.testUtilities import assertVectorsAlmostEqual

import numpy as np
from MAPLEAF.IO import SimDefinition, SubDictReader
from MAPLEAF.IO.Logging import removeLogger
from MAPLEAF.Main import Simulation
from MAPLEAF.Motion import AngularVelocity, Quaternion, Vector
from MAPLEAF.Motion.Integration import ClassicalIntegrator
from MAPLEAF.Rocket import RecoverySystem, Rocket


class TestRocket(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        simRunner1 = Simulation("MAPLEAF/Examples/Simulations/test2.mapleaf", silent=True)
        cls.rocket = simRunner1.createRocket()

        simRunner2 = Simulation("MAPLEAF/Examples/Simulations/test6.mapleaf", silent=True)
        cls.rocket2 = simRunner2.createRocket()

        canardRunner = Simulation("MAPLEAF/Examples/Simulations/Canards.mapleaf", silent=True)
        canardRunner.simDefinition.setValue("SimControl.loggingLevel", "0")        
        cls.canardRocket = canardRunner.createRocket()
        cls.originalCanardRocketState = cls.canardRocket.rigidBody.state

        cls.stagingRunner = Simulation("MAPLEAF/Examples/Simulations/Staging.mapleaf", silent=True)
        cls.stagingRunner.simDefinition.setValue("SimControl.loggingLevel", "0")
        cls.stagingRocket = cls.stagingRunner.createRocket()

    def resetCanardRocket(self):
        self.canardRocket.rigidBody.time = 0
        self.canardRocket.rigidBody.state = self.originalCanardRocketState

    def test_getStageSubDicts(self):
        result = self.rocket._getStageSubDicts()
        expectedResult = [ "Rocket.Sustainer" ]
        self.assertEqual(result, expectedResult)

    def test_getMaxDiameter(self):
        result = self.rocket2._getMaxBodyTubeDiameter()
        expectedResult = 0.1524
        self.assertAlmostEqual(result, expectedResult)

    def test_Aref(self):
        Aref = self.rocket2.Aref
        ExpectedResult = 0.1524**2 * math.pi / 4
        self.assertAlmostEqual(Aref, ExpectedResult)

    def test_finControlIndependentOfForceEvaluations(self):
        env = self.canardRocket._getEnvironmentalConditions(0, self.canardRocket.rigidBody.state)
        self.canardRocket.controlSystem.runControlLoopIfRequired(0, self.canardRocket.rigidBody.state, env)

        # Check that fin targets are unaffected by performing a rigid body time step, which calls all the component-buildup functions
        firstFinTargets = [ x.targetDeflection for x in self.canardRocket.controlSystem.controlledSystem.actuatorList ]
        self.canardRocket.rigidBody.timeStep(0.01)
        newFinTargets = [ x.targetDeflection for x in self.canardRocket.controlSystem.controlledSystem.actuatorList ]

        self.almostEqualVectors(firstFinTargets, newFinTargets)

    def test_finControlIndependentOfTimeStep(self):
        self.resetCanardRocket()

        # Check that fin targets are unaffected by performing rocket time steps smaller than the control loop time step
        self.canardRocket.timeStep(0.005)
        firstFinTargets = [ x.targetDeflection for x in self.canardRocket.controlSystem.controlledSystem.actuatorList ]
        self.canardRocket.timeStep(0.005)
        newFinTargets1 = [ x.targetDeflection for x in self.canardRocket.controlSystem.controlledSystem.actuatorList ]
        self.canardRocket.timeStep(0.005)
        newFinTargets2 = [ x.targetDeflection for x in self.canardRocket.controlSystem.controlledSystem.actuatorList ]

        self.assertTrue(np.allclose(firstFinTargets, newFinTargets1, 1e-13))
        self.assertFalse(np.allclose(firstFinTargets, newFinTargets2, 1e-13))

    def test_controlTimeStepChecking(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Canards.mapleaf", silent=True)
        simDef.setValue("SimControl.timeDiscretization", "Euler")
        simDef.setValue("SimControl.loggingLevel", "0")
        
        simDef.setValue("SimControl.timeStep", "0.01")
        simDef.setValue("Rocket.ControlSystem.updateRate", "101")
        
        rocketDictReader = SubDictReader("Rocket", simDef)
        # controlledCanardRocket = Rocket(rocketDictReader, silent=True)
        # self.assertAlmostEqual(controlledCanardRocket.controlSystem.controlTimeStep, 1/101)

        simDef.setValue("SimControl.timeStep", "0.011")
        simDef.setValue("Rocket.ControlSystem.updateRate", "100")
        controlledCanardRocket = Rocket(rocketDictReader, silent=True)
        self.assertAlmostEqual(controlledCanardRocket.controlSystem.controlTimeStep, 1/100)

        # simDef.setValue("SimControl.timeStep", "0.009")
        # controlledCanardRocket = Rocket(rocketDictReader, silent=True)
        # self.assertAlmostEqual(controlledCanardRocket.controlSystem.controlTimeStep, 1/100)

        # simDef.setValue("SimControl.timeStep", "0.1")
        # controlledCanardRocket = Rocket(rocketDictReader, silent=True)
        # self.assertAlmostEqual(controlledCanardRocket.controlSystem.controlTimeStep, 1/100)

        simDef.setValue("SimControl.timeStep", "0.001")
        simDef.setValue("SimControl.timeDiscretization", "RK45Adaptive")
        simDef.setValue("SimControl.TimeStepAdaptation.controller", "PID")
        simDef.setValue("Rocket.ControlSystem.updateRate", "100")
        controlledCanardRocket = Rocket(rocketDictReader, silent=True)
        self.assertEqual(controlledCanardRocket.rigidBody.integrate.method, "RK4")
        self.assertEqual(type(controlledCanardRocket.rigidBody.integrate), ClassicalIntegrator)
        self.assertEqual(controlledCanardRocket.controlSystem.controlTimeStep, 1/100)

    def test_forcesSymmetricForSymmetricRocket(self):
        self.rocket2.rigidBody.state.angularVelocity = AngularVelocity(0, 0, 0)
        self.rocket2.rigidBody.state.velocity = Vector(0, 10, 100) #5.7 degree angle of attack
        self.rocket2.rigidBody.state.orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        appliedForce = self.rocket2._getAppliedForce(0, self.rocket2.rigidBody.state)
        self.assertAlmostEqual(appliedForce.moment.Y, 0)
        self.assertAlmostEqual(appliedForce.moment.Z, 0)

        self.rocket2.rigidBody.state.velocity = Vector(0, 100, 100) #45 degree angle of attack
        self.rocket2.rigidBody.state.orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        appliedForce = self.rocket2._getAppliedForce(0, self.rocket2.rigidBody.state)
        self.assertAlmostEqual(appliedForce.moment.Y, 0)
        self.assertAlmostEqual(appliedForce.moment.Z, 0)
        self.assertAlmostEqual(appliedForce.moment.Z, 0)

        self.rocket2.rigidBody.state.velocity = Vector(0, 500, 500) #45 degree angle of attack
        self.rocket2.rigidBody.state.orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        appliedForce = self.rocket2._getAppliedForce(0, self.rocket2.rigidBody.state)
        self.assertAlmostEqual(appliedForce.moment.Y, 0)
        self.assertAlmostEqual(appliedForce.moment.Z, 0)

    def test_stagedInitialization(self):
        # Check that positions of objects in first and second stages are different. The stages are otherwise identical, except the first stage doesn't have a nosecone
        bodyTube1 = self.stagingRocket.stages[0].getComponentsOfType(RecoverySystem)[0]
        firstStageRecoveryPosition = bodyTube1.position.Z
        
        bodyTube2 = self.stagingRocket.stages[1].getComponentsOfType(RecoverySystem)[0]
        secondStageRecoveryPosition = bodyTube2.position.Z
        self.assertEqual(firstStageRecoveryPosition-4.011, secondStageRecoveryPosition)

        orbitalSimRunner = Simulation("MAPLEAF/Examples/Simulations/NASATwoStageOrbitalRocket.mapleaf", silent=True)
        rocket = orbitalSimRunner.createRocket()
        initInertia = rocket.getInertia(0, rocket.rigidBody.state)
        self.assertAlmostEqual(initInertia.mass, 314000.001, 2)

    def test_staging(self):
        stagingSimRunner = self.stagingRunner
        twoStageRocket = stagingSimRunner.createRocket()

        #### Combined (two-stage) rocket checks ####
        # Check that two stages have been created
        self.assertEqual(len(twoStageRocket.stages), 2)

        # Check combined inertia is correct
        inertia = twoStageRocket.getInertia(0, "fakeState")
        self.assertEqual(inertia.mass, 60)
        assertVectorsAlmostEqual(self, inertia.CG, Vector(0,0,-4.6555))
        assertVectorsAlmostEqual(self, inertia.MOI, Vector(411.321815, 411.321815, 1.0))

        # Check that the first stage motor will ignite and the second wont
        self.assertEqual(twoStageRocket.stages[-1].motor.ignitionTime, 0)
        self.assertTrue(twoStageRocket.stages[0].motor.ignitionTime > 100 )

        # Set up simRunner so it's able to create the detached (dropped) stage
        stagingSimRunner.rocketStages = [twoStageRocket]
        stagingSimRunner.dts = [ 0.05 ]
        stagingSimRunner.endDetectors = [ stagingSimRunner._getEndDetectorFunction(twoStageRocket, stagingSimRunner.simDefinition) ]
        stagingSimRunner.stageFlightPaths = [ stagingSimRunner._setUpCachingForFlightAnimation(twoStageRocket) ]

        # Trigger stage separation
        twoStageRocket._stageSeparation()

        #### Upper stage checks ####
        self.assertEqual(len(twoStageRocket.stages), 1)

        topStageInertia = twoStageRocket.getInertia(2, "fakeState")
        self.assertEqual(topStageInertia.mass, 30)
        assertVectorsAlmostEqual(self, topStageInertia.CG, Vector(0,0,-2.65))
        assertVectorsAlmostEqual(self, topStageInertia.MOI, Vector(85, 85, 0.5))

        self.assertEqual(twoStageRocket.stages[0].motor.ignitionTime, 0)
        self.assertEqual(len(twoStageRocket.stages[0].components), 7) # Originally 6 - added 1 for zero-length boattail

        #### Dropped stage checks ####
        self.assertEqual(len(stagingSimRunner.rocketStages), 2)
        droppedStage = stagingSimRunner.rocketStages[1]

        droppedStageInertia = droppedStage.getInertia(2, "fakeState")
        self.assertEqual(droppedStageInertia.mass, topStageInertia.mass)
        self.assertEqual(droppedStageInertia.MOI, topStageInertia.MOI)
        self.assertNotEqual(droppedStageInertia.CG, topStageInertia.CG)
        self.assertNotEqual(droppedStageInertia.MOICentroidLocation, topStageInertia.MOICentroidLocation)

        self.assertTrue(droppedStage.stages[0].motor.ignitionTime < 0)

        self.assertEqual(len(droppedStage.stages[0].components), 6) # Originally 5 - added 1 for zero-length boattail

    def test_delayedStaging(self):
        # Create simRunner & Rocket
        simDef = deepcopy(self.stagingRunner.simDefinition)
        simDef.setValue("SimControl.timeDiscretization", "RK4")
        simDef.setValue("Rocket.FirstStage.separationDelay", "0.01")
        stagingSimRunner = Simulation(simDefinition=simDef, silent=True)
        twoStageRocket = stagingSimRunner.createRocket()
        
        # Initialize properties in the simRunner required for stage separation. These are normally created in simRunner._run
        stagingSimRunner.rocketStages = [ twoStageRocket ]
        stagingSimRunner.dts = [ 0.011 ]
        stagingSimRunner.endDetectors = [ stagingSimRunner._getEndDetectorFunction(twoStageRocket, stagingSimRunner.simDefinition) ]
        stagingSimRunner.stageFlightPaths = [ stagingSimRunner._setUpCachingForFlightAnimation(twoStageRocket) ]

        # Set separation delay, take first time step
        twoStageRocket.rigidBody.time = 4.99 # 0.01 seconds before motor burnout (5 seconds)
        twoStageRocket.timeStep(0.011) # Past motor burnout - should schedule delayed separation for 10.511 seconds
        twoStageRocket.simEventDetector.triggerEvents()
        # Sim time reached: 5.01

        # Check that separation has not triggered
        self.assertEqual(len(twoStageRocket.stages), 2)

        # Take time step that triggers separation
        twoStageRocket.timeStep(0.011) # Past delayed separation point (5.011 seconds)
        twoStageRocket.simEventDetector.triggerEvents()
        # Sim time reached: 5.012

        # Check that separation has triggered
        self.assertEqual(len(twoStageRocket.stages), 1)

    def test_rocketInertiaOverrides(self):
        # Check CG Override
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf")
        rocketDictReader = SubDictReader("Rocket", simDef)
        rocket = Rocket(rocketDictReader, silent=True)
        cgLoc = rocket.getCG(0, rocket.rigidBody.state)
        expectedCGLoc = Vector(0,0,-2.65)
        assertVectorsAlmostEqual(self, cgLoc, expectedCGLoc)

        # Check Mass override
        expectedMass = 50
        actualMass = rocket.getMass(0, rocket.rigidBody.state)
        self.assertAlmostEqual(actualMass, expectedMass)

        # Check MOI Override + getInertia function
        expectedMOI = Vector(85, 85, 0.5)
        actualInertia = rocket.getInertia(0, rocket.rigidBody.state)
        assertVectorsAlmostEqual(self, actualInertia.MOI, expectedMOI)
        assertVectorsAlmostEqual(self, actualInertia.CG, expectedCGLoc)
        self.assertAlmostEqual(actualInertia.mass, expectedMass)
        
    def test_rocketCGCalculation(self):
        # Check actual CG Loc Calculation
        
        # Top stage CG: #
        # Nosecone: 1 kg @ 0m
        # Recovery: 5 kg @ -2m
        # BodyTube: 1 kg @ -0.762m
        # Mass:     50 kg @ -2.6m
        # Motor:    11.4885 kg @ 2.130434783
        # Fins:     1 kg @ -4.011m
        # Total Weight = 69.4885 kg
        # Stage 1 CG: -2.4564359077698

        expectedCG = Vector(0,0,-1.731185736)

        # Remove overrides from rocket definition file
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        simDef.removeKey("Rocket.Sustainer.constCG")
        simDef.removeKey("Rocket.Sustainer.constMass")
        simDef.removeKey("Rocket.Sustainer.constMOI")

        rocketDictReader = SubDictReader("Rocket", simDef)
        rocket = Rocket(rocketDictReader, silent=True)
        rocketInertia = rocket.getInertia(0, rocket.rigidBody.state)
        assertVectorsAlmostEqual(self, rocketInertia.CG, expectedCG, n=4)

    # Utility Function
    def almostEqualVectors(self, Vector1, Vector2, n=7):
        for x in zip(Vector1, Vector2):
            self.assertAlmostEqual(x[0], x[1], n)

    def tearDown(self):
        removeLogger()

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
