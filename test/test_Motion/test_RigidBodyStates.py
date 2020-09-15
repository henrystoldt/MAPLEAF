#Created by: Henry Stoldt
#May 2019

#To run tests:
#In this file: [test_Vector.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest
from math import pi

from MAPLEAF.Motion import AngularVelocity
from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import RigidBodyState, RigidBodyState_3DoF, RigidBodyStateDerivative_3DoF, RigidBodyStateDerivative
from MAPLEAF.Motion import Vector
from test.testUtilities import assertQuaternionsAlmostEqual, assertRigidBodyStatesalmostEqual, assertVectorsAlmostEqual


class TestRigidBodyState(unittest.TestCase):
    def setUp(self):
        pos1 = Vector(0,0,0)
        pos2 = Vector(1,0,0)
        Vel1 = Vector(0,0,0)
        Vel2 = Vector(2,0,0)
        Orientation1 = Quaternion(axisOfRotation=Vector(1,0,0), angle=0)
        Orientation2 = Quaternion(axisOfRotation=Vector(1,0,0), angle=pi/2)
        AngVel1 = AngularVelocity(axisOfRotation=Vector(1,0,0), angularVel=0)
        AngVel2 = AngularVelocity(axisOfRotation=Vector(1,0,0), angularVel=pi)
        
        self.iRBS1 = RigidBodyState(pos1, Vel1, Orientation1, AngVel1)
        self.iRBS2 = RigidBodyState(pos2, Vel2, Orientation2, AngVel2)

    def nearlyEqualAngVel(self, av1, av2):
        for i in range(3):
            self.assertAlmostEqual(av1[i], av2[i])

    def test_AddRigidBodyState(self):
        iRBS3 = self.iRBS1 + self.iRBS2
        self.assertEqual(iRBS3.position, Vector(1,0,0))
        self.assertEqual(iRBS3.velocity, Vector(2,0,0))
        self.assertEqual(iRBS3.orientation, Quaternion(axisOfRotation=Vector(1,0,0), angle=pi/2))
        self.nearlyEqualAngVel(iRBS3.angularVelocity, AngularVelocity(axisOfRotation=Vector(1,0,0), angularVel=pi))
    
    def test_MultiplyRigidBodyState(self):
        iRBS3 = self.iRBS2 * 0.5
        self.assertEqual(iRBS3.position, Vector(0.5,0,0))
        self.assertEqual(iRBS3.velocity, Vector(1,0,0))
        assertQuaternionsAlmostEqual(self, iRBS3.orientation, Quaternion(axisOfRotation=Vector(1,0,0), angle=pi/4), n=14)
        self.nearlyEqualAngVel(iRBS3.angularVelocity, AngularVelocity(axisOfRotation=Vector(1,0,0), angularVel=pi/2))

    def test_RigidBodyState_add(self):
        pos = Vector(0,0,0)
        vel = Vector(0,0,0)
        orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        angVel = AngularVelocity(rotationVector=Vector(0,0,0))
        state1 = RigidBodyState(pos, vel, orientation, angVel)

        pos2 = Vector(0,0,0.1)
        vel2 = Vector(0,0,0.2)
        orientation2 = Quaternion(axisOfRotation=Vector(0,0,1), angle=pi/2)
        angVel2 = AngularVelocity(rotationVector=Vector(0,0,0.3))
        state2 = RigidBodyState(pos2, vel2, orientation2, angVel2)

        result = state1 + state2
        assertRigidBodyStatesalmostEqual(self, result, state2)

        pos3 = Vector(0,0,0.1)
        vel3 = Vector(0,0,0.2)
        orientation3 = Quaternion(axisOfRotation=Vector(0,1,0), angle=pi/2)
        angVel3 = AngularVelocity(rotationVector=Vector(0,0,0.3))
        state3 = RigidBodyState(pos3, vel3, orientation3, angVel3)

        result = state3 + state2
        testVec = result.orientation.rotate(Vector(0,0,1))
        assertVectorsAlmostEqual(self, testVec, Vector(0,1,0))

    # def test_RigidBodyState_sub(self):
    #     pos = Vector(0,0,0)
    #     vel = Vector(0,0,0)
    #     orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
    #     angVel = AngularVelocity(rotationVector=Vector(0,0,0))
    #     state1 = RigidBodyState(pos, vel, orientation, angVel)

    #     pos2 = Vector(0,0,0.1)
    #     vel2 = Vector(0,0,0.2)
    #     orientation2 = Quaternion(axisOfRotation=Vector(0,0,1), angle=1)
    #     angVel2 = AngularVelocity(rotationVector=Vector(0,0,0.3))
    #     state2 = RigidBodyState(pos2, vel2, orientation2, angVel2)

    #     result = state2 - state2

    #     assertRigidBodyStatesalmostEqual(self, result, state1)

    #     result = state2 + state2
    #     result = result - state2
    #     assertRigidBodyStatesalmostEqual(self, result, state2)


    #     state2.orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=pi/2)
    #     pos3 = Vector(0,0,0.1)
    #     vel3 = Vector(0,0,0.2)
    #     orientation3 = Quaternion(axisOfRotation=Vector(0,1,0), angle=pi/2)
    #     angVel3 = AngularVelocity(rotationVector=Vector(0,0,0.3))
    #     state3 = RigidBodyState(pos3, vel3, orientation3, angVel3)

    #     result = state3 + state2
    #     result2 = -state2 + result
    #     testVec = result2.orientation.rotate(Vector(0,0,1))
    #     assertVectorsAlmostEqual(self, testVec, Vector(0,1,0))
    #     testVec = result2.orientation.rotate(Vector(1,0,0))
    #     assertVectorsAlmostEqual(self, testVec, Vector(1,0,0))
    #     testVec = result2.orientation.rotate(Vector(0,1,0))
    #     assertVectorsAlmostEqual(self, testVec, Vector(0,0,-1))
    #     result2 = -state2 + (-state3 + result) 
    #     assertQuaternionsAlmostEqual(self, result2.orientation, orientation)

    #     takeMeToZero = state1 - state3
    #     result = state3 + takeMeToZero
    #     assertRigidBodyStatesalmostEqual(self, result, state1)


    #     result3 = (state2 + state3) - state3
    #     assertRigidBodyStatesalmostEqual(self, result3, state2)

    def test_Logging(self):
        headerItemCount = len(self.iRBS1.getLogHeader().split())
        logItemCount = len(self.iRBS1.__str__().split())
        self.assertEqual(headerItemCount, logItemCount)

class TestRigidBodyState_3DoF(unittest.TestCase):
    def setUp(self):
        pos1 = Vector(0,1,2)
        pos2 = Vector(1,2,3)

        Vel1 = Vector(-1,0,1)
        Vel2 = Vector(2,3,4)
        
        self.iRBS1 = RigidBodyState_3DoF(pos1, Vel1)
        self.iRBS2 = RigidBodyState_3DoF(pos2, Vel2)

    def test_Mul(self):
        multiplied = self.iRBS2 * 2
        expectedPos = Vector(2,4,6)
        expectedVel = Vector(4,6,8)
        assertVectorsAlmostEqual(self, expectedPos, multiplied.position)
        assertVectorsAlmostEqual(self, expectedVel, multiplied.velocity)

    # def test_Sub(self):
    #     subtracted = self.iRBS1 - self.iRBS2
    #     expectedPos = Vector(-1,-1,-1)
    #     expectedVel = Vector(-3,-3,-3)
    #     assertVectorsAlmostEqual(self, expectedPos, subtracted.position)
    #     assertVectorsAlmostEqual(self, expectedVel, subtracted.velocity)

    def test_Logging(self):
        headerItemCount = len(self.iRBS1.getLogHeader().split())
        logItemCount = len(self.iRBS1.__str__().split())
        self.assertEqual(headerItemCount, logItemCount)
