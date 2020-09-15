#Created by: Henry Stoldt
#April 2020

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest

from MAPLEAF.Motion import linInterp, linInterpWeights, interpolateRigidBodyStates
from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import RigidBodyState_3DoF, RigidBodyState
from test.testUtilities import assertIterablesAlmostEqual, assertQuaternionsAlmostEqual
from MAPLEAF.Motion import Vector

class TestInterpolation(unittest.TestCase):

    def setUp(self):
        pass
    
    def test_linInterp(self):
        self.assertEqual(linInterp([0, 1], [0, 1], 0.5), 0.5)
        self.assertEqual(linInterp([0, 1, 2, 3], [0, 1, 3, 5], 1.5), 2)
        self.assertEqual(linInterp([0, 1, 2, 3], [0, 1, 3, 5], 3), 5)

    def test_linInterpWeights(self):
        Xvals = [ 0, 1, 2, 3 ]
        desiredX = 0.8
        smYI, smYW, lgYI, lgYW = linInterpWeights(Xvals, desiredX)
        self.assertAlmostEqual(smYI, 0)
        self.assertAlmostEqual(lgYI, 1)
        self.assertAlmostEqual(smYW, 0.2)
        self.assertAlmostEqual(lgYW, 0.8)

    def test_interpolateRigidBodyStates_3DoF(self):
        vel1 = Vector(0,1,2)
        vel2 = Vector(2,3,4)
        pos1 = vel1
        pos2 = Vector(4,5,6)

        rBS1 = RigidBodyState_3DoF(pos1, vel1)
        rBS2 = RigidBodyState_3DoF(pos2, vel2)

        rBS1p5 = interpolateRigidBodyStates(rBS1, rBS2, 0.5)
        expectedResult = RigidBodyState_3DoF(Vector(2,3,4), Vector(1,2,3))
        assertIterablesAlmostEqual(self, expectedResult.position, rBS1p5.position)
        assertIterablesAlmostEqual(self, expectedResult.velocity, rBS1p5.velocity)

    def test_interpolateRigidBodyStates(self):
        vel1 = Vector(0,1,2)
        vel2 = Vector(2,3,4)
        pos1 = vel1
        pos2 = Vector(4,5,6)

        angVel1 = vel1
        angVel2 = Vector(6,7,8)

        quat1 = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        quat2 = Quaternion(axisOfRotation=Vector(0,0,1), angle=2)

        rBS1 = RigidBodyState(pos1, vel1, quat1, angVel1)
        rBS2 = RigidBodyState(pos2, vel2, quat2, angVel2)

        rBS1p5 = interpolateRigidBodyStates(rBS1, rBS2, 0.5)
        assertIterablesAlmostEqual(self, Vector(2,3,4), rBS1p5.position)
        assertIterablesAlmostEqual(self, Vector(1,2,3), rBS1p5.velocity)
        assertQuaternionsAlmostEqual(self, rBS1p5.orientation, Quaternion(axisOfRotation=Vector(0,0,1), angle=1))
        assertIterablesAlmostEqual(self, rBS1p5.angularVelocity, Vector(3,4,5))

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
