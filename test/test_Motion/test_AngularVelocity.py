# Created by: Henry Stoldt
# January 2019

# To run tests:
# In this file  [ test_AngularVelocity ]
# In all files in the current directory: [ python -m unittest discover ]
# Add -v for verbose output (displays names of all tests

import unittest

from MAPLEAF.Motion import AngularVelocity
from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import Vector

class TestAngularVelocity(unittest.TestCase):
    def setUp(self):
        self.angVel = AngularVelocity(rotationVector=Vector(0,0,1))
        self.angVel2 = AngularVelocity(rotationVector=Vector(0,0.5,0))
        self.quat1 = (self.angVel + self.angVel2).toQuaternion()

    def test_add(self):
        result = self.angVel + self.angVel2
        self.assertEqual(result, AngularVelocity(rotationVector=Vector(0,0.5,1)))

    def test_sub(self):
        result = self.angVel - self.angVel2
        self.assertEqual(result, AngularVelocity(rotationVector=Vector(0,-0.5,1)))

    def test_initFromComp(self):
        angVel = AngularVelocity(0, 2, 5)
        self.assertEqual(Vector(0,2,5), angVel)

    def test_ToQuat(self):
        result = self.angVel.toQuaternion()
        self.assertEqual(result, Quaternion(axisOfRotation=self.angVel, angle=self.angVel.length()))
