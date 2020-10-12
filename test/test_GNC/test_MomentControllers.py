import unittest
from math import pi

import numpy as np

from MAPLEAF.GNC import \
    ScheduledGainPIDRocketMomentController
from MAPLEAF.GNC import Stabilizer
from MAPLEAF.Motion import AngularVelocity, Quaternion, RigidBodyState, Vector


class TestScheduledGainPIDRocketMomentController(unittest.TestCase):
    def setUp(self):
        self.momentController = ScheduledGainPIDRocketMomentController("MAPLEAF/Examples/TabulatedData/testPIDControlLaw.txt", ["Mach", "Altitude"])
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
