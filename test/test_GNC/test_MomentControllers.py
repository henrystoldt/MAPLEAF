import unittest
from math import pi

import numpy as np

from MAPLEAF.GNC import \
    TableScheduledGainPIDRocketMomentController, EquationScheduledGainPIDRocketMomentController
from MAPLEAF.GNC import Stabilizer
from MAPLEAF.Motion import AngularVelocity, Quaternion, RigidBodyState, Vector
from MAPLEAF.IO import SimDefinition
from MAPLEAF.Main import Simulation
from test.testUtilities import assertVectorsAlmostEqual

class TestScheduledGainPIDRocketMomentController(unittest.TestCase):
    def setUp(self):
        self.momentController = TableScheduledGainPIDRocketMomentController("MAPLEAF/Examples/TabulatedData/testPIDControlLaw.txt", ["Mach", "Altitude"])
        self.stabilizer = Stabilizer(Vector(0,0,1))

    def test_getOrientationErrorAndGetTargetOrientation(self):
        # Basic spin case
        pos = Vector(0,0,0)
        vel = Vector(0,0,0)
        orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0.12)
        angularVelocity = AngularVelocity(axisOfRotation=Vector(0,0,1), angularVel=0)
        rigidBodyState = RigidBodyState(pos, vel, orientation, angularVelocity)

        # targetDir = Vector(0,0,1)
        # targetSpin = 0
        targetState = [ 0,0,1,0 ]
        ExpectedResult = ( 0, 0, -0.12 )

        targetOrientation = self.stabilizer.getTargetOrientation(rigidBodyState, targetState, 0)
        result = self.momentController._getOrientationError(rigidBodyState, targetOrientation)
        for i in range(3):
            self.assertAlmostEqual(result[i], ExpectedResult[i])

        # Longitudinal 1
        orientation = Quaternion(axisOfRotation=Vector(0,1,0), angle=0.12)
        rigidBodyState = RigidBodyState(pos, vel, orientation, angularVelocity)

        # targetDir = Vector(0,0,1)
        # targetSpin = 0
        targetState = [ 0,0,1,0 ]
        ExpectedResult = ( 0, -0.12, 0 )
        targetOrientation = self.stabilizer.getTargetOrientation(rigidBodyState, targetState, 0)
        result = self.momentController._getOrientationError(rigidBodyState, targetOrientation)
        for i in range(3):
            self.assertAlmostEqual(result[i], ExpectedResult[i])

        # Longitudinal 2
        orientation = Quaternion(axisOfRotation=Vector(1,0,0), angle=0.12)
        rigidBodyState = RigidBodyState(pos, vel, orientation, angularVelocity)

        # targetDir = Vector(0,0,1)
        # targetSpin = 0
        targetState = [ 0,0,1,0 ]
        ExpectedResult = ( -0.12, 0, 0 )
        targetOrientation = self.stabilizer.getTargetOrientation(rigidBodyState, targetState, 0)
        result = self.momentController._getOrientationError(rigidBodyState, targetOrientation)
        for i in range(3):
            self.assertAlmostEqual(result[i], ExpectedResult[i])

        # Pre-rotated longitudinal
        rot1 = Quaternion(axisOfRotation=Vector(1,0,0), angle=0.12)
        rot2 = Quaternion(axisOfRotation=Vector(0,0,1), angle=pi/4)
        orientation = rot2*rot1
        rigidBodyState = RigidBodyState(pos, vel, orientation, angularVelocity)

        # targetDir = Vector(0,0,1)
        # targetSpin = 0
        targetState = [ 0,0,1,0 ]

        ExpectedResult = orientation.conjugate().toRotationVector()
        targetOrientation = self.stabilizer.getTargetOrientation(rigidBodyState, targetState, 0)
        result = self.momentController._getOrientationError(rigidBodyState, targetOrientation)
        for i in range(3):
            self.assertAlmostEqual(result[i], ExpectedResult[i])

        # Post-rotated longitudinal 2
        orientation = Quaternion(axisOfRotation=Vector(1,1,0), angle=0.12)
        rigidBodyState = RigidBodyState(pos, vel, orientation, angularVelocity)

        # targetDir = Vector(0,0,1)
        # targetSpin = 0
        targetState = [ 0,0,1,0 ]

        ExpectedResult = orientation.conjugate().toRotationVector() #Zerror here assumed correct - not calculated analytically
        targetOrientation = self.stabilizer.getTargetOrientation(rigidBodyState, targetState, 0)
        result = self.momentController._getOrientationError(rigidBodyState, targetOrientation)
        for i in range(3):
            self.assertAlmostEqual(result[i], ExpectedResult[i])

    def test_getDesiredMoments(self):
        # Basic spin case
        pos = Vector(0,0,0)
        vel = Vector(0,0,0)
        orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0.12)
        targetOrientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        angularVelocity = AngularVelocity(axisOfRotation=Vector(0,0,1), angularVel=0)
        rigidBodyState = RigidBodyState(pos, vel, orientation, angularVelocity)
        expectedAngleError = np.array([ 0, 0, -0.12 ])

        dt = 1
        ExpectedPIDCoeffs = [[ 4, 5, 6 ], [ 4, 5, 6 ], [ 7.5, 8.5, 9.5 ]]

        ExpectedDer = expectedAngleError / dt
        ExpectedIntegral = expectedAngleError * dt / 2

        ExpectedM = []
        for i in range(3):
            moment = ExpectedPIDCoeffs[i][0]*expectedAngleError[i] + ExpectedPIDCoeffs[i][1]*ExpectedIntegral[i] + ExpectedPIDCoeffs[i][2]*ExpectedDer[i]
            ExpectedM.append(moment)

        # Replace keyFunctions with these ones that return fixed values
        def fakeMach(*args):
            # MachNum = 0.1
            return 0.1

        def fakeAltitude(*args):
            # Altitude = 0.5
            return 0.5

        self.momentController.keyFunctionList = [ fakeMach, fakeAltitude ]

        calculatedMoments = self.momentController.getDesiredMoments(rigidBodyState, "fakeEnvironemt", targetOrientation, 0, dt)
        for i in range(3):
            # print(calculatedMoments[i])
            # print(ExpectedM[i])
            self.assertAlmostEqual(calculatedMoments[i], ExpectedM[i])

class TestEquationScheduledGainPIDROcketMomentController(unittest.TestCase):
    
    def setUp(self):
      parameterList = ["Mach", "Altitude"]
      equationOrder = 2
      self.momentController = EquationScheduledGainPIDRocketMomentController("MAPLEAF/Examples/TabulatedData/lateralPIDEquationCoeffs.txt",\
        "MAPLEAF/Examples/TabulatedData/lateralPIDEquationCoeffs.txt", parameterList, equationOrder)

      self.stabilizer = Stabilizer(Vector(0,0,1))

    def test_getEquationCoefficientsFromTextFile(self):
      result = self.momentController.pitchController.PcoefficientList
      expectedResult = [1,2,3,4,5,6]

      for i in range(len(expectedResult)):
        self.assertAlmostEqual(result[i],expectedResult[i])

    def test_getDesiredMoments(self):
      # Basic spin case
        pos = Vector(0,0,0)
        vel = Vector(0,0,0)
        orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0.12)
        targetOrientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        angularVelocity = AngularVelocity(axisOfRotation=Vector(0,0,1), angularVel=0)
        rigidBodyState = RigidBodyState(pos, vel, orientation, angularVelocity)
        expectedAngleError = np.array([ 0, 0, -0.12 ])

        dt = 1
        ExpectedPIDCoeffs = [[ 687, 687, 687], [ 687, 687, 687], [ 687, 687, 687]]

        ExpectedDer = expectedAngleError / dt
        ExpectedIntegral = expectedAngleError * dt / 2

        ExpectedM = []
        for i in range(3):
            moment = ExpectedPIDCoeffs[i][0]*expectedAngleError[i] + ExpectedPIDCoeffs[i][1]*ExpectedIntegral[i] + ExpectedPIDCoeffs[i][2]*ExpectedDer[i]
            ExpectedM.append(moment)

        # Replace keyFunctions with these ones that return fixed values
        def fakeMach(*args):
            # MachNum = 1
            return 1

        def fakeAltitude(*args):
            # Altitude = 10
            return 10

        self.momentController.parameterFetchFunctionList = [ fakeMach, fakeAltitude ]

        calculatedMoments = self.momentController.getDesiredMoments(rigidBodyState, "fakeEnvironemt", targetOrientation, 0, dt)
        for i in range(3):
            # print(calculatedMoments[i])
            # print(ExpectedM[i])
            self.assertAlmostEqual(calculatedMoments[i], ExpectedM[i])


      
class TestIdealMomentController(unittest.TestCase):
    def test_instantTurn(self):
        simulationDefinition = SimDefinition("MAPLEAF/Examples/Simulations/Canards.mapleaf", silent=True)
        # Use an ideal moment controller
        simulationDefinition.setValue("Rocket.ControlSystem.MomentController.Type", "IdealMomentController")
        # Set initial direction to be pointing directly upwards
        simulationDefinition.setValue("Rocket.initialDirection", "(0 0 1)")
        # Ask for an Immediate Turn
        simulationDefinition.setValue("Rocket.ControlSystem.desiredFlightDirection", "(0 1 1)")

        runner = Simulation(simDefinition=simulationDefinition, silent=True)
        rocket = runner.createRocket()

        # Take a time step and check that the desired direction has been achieved immediately
        rocket.timeStep(0.01)

        currentDirection = rocket.rigidBody.state.orientation.rotate(Vector(0,0,1))
        expectedDirection = Vector(0,1,1).normalize() 
        assertVectorsAlmostEqual(self, currentDirection, expectedDirection, 5)