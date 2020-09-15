#Created by: Henry Stoldt
#January 2019

#To run tests:
#In this file: [test_Vector.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest
from math import pi, sin, sqrt

from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import Vector
from test.testUtilities import assertQuaternionsAlmostEqual, assertVectorsAlmostEqual

class TestQuaternion(unittest.TestCase):
    def setUp(self):
      self.q1 = Quaternion(components=[ 1.0,2.0,3.0,4.0 ])
      self.q2 = Quaternion(components=[ -1.0,-2.0,-3.0,-4.0])
      self.q3 = Quaternion(axisOfRotation=Vector(1.0,0.0,0.0), angle=pi/2)

      self.q4 = Quaternion(components=[ 1.0,0.0,1.0,0.0 ])
      self.q5 = Quaternion(components=[ 1.0,0.5,0.5,0.75 ])

      self.rotQ = Quaternion(axisOfRotation=Vector(2.0,-3.0,5.0), angle=2)
      self.rotQ2 = Quaternion(axisOfRotation=Vector(0.0,0.0,1.0), angle=pi/2)
      self.rotQ3 = Quaternion(axisOfRotation=Vector(0.0,1.0,0.0), angle=pi/2)
      self.rotQ4 = Quaternion(axisOfRotation=Vector(1.0,0.0,0.0), angle=pi/2)
      self.rotQ5 = Quaternion(axisOfRotation=Vector(0.0,0.0,1.0), angle=pi)
      self.rotQ6 = Quaternion(axisOfRotation=Vector(0.0,1.0,0.0), angle=pi)
      self.rotQ7 = Quaternion(axisOfRotation=Vector(1.0,0.0,0.0), angle=pi)

    #Test same method as print statement
    def test_str(self):
        self.assertEqual(str(self.q1), "<1.0, 2.0, 3.0, 4.0>")
        
    def test_scaleRotation(self):
        scaledQuat = self.q3.scaleRotation(0.5)
        definedQuat = Quaternion(axisOfRotation=Vector(1,0,0), angle=pi/4)
        assertQuaternionsAlmostEqual(self, scaledQuat, definedQuat, 14 )

    def test_slerp(self):
        testQ = self.q1.slerp(self.q3, 0)
        assertQuaternionsAlmostEqual(self, (self.q1 * testQ).normalize(), self.q1.normalize())

        testQ = self.q1.slerp(self.q3, 1)
        testQ = (self.q1 * testQ).normalize()
        assertQuaternionsAlmostEqual(self, testQ, self.q3.normalize())
        
        testQ = self.rotQ2.slerp(self.rotQ5, 0.5)
        testQ = (self.rotQ2 * testQ).normalize()
        assertQuaternionsAlmostEqual(self, testQ, Quaternion(axisOfRotation=Vector(0,0,1), angle=3*pi/4))

    #Test using the quaternion to rotate a vector
    #TODO: verify with matlab or other example
    def test_rotate(self):
        assertVectorsAlmostEqual(self, self.rotQ2.rotate(Vector(1,1,1)), Vector(-1,1,1))
        assertVectorsAlmostEqual(self, self.rotQ3.rotate(Vector(1,1,1)), Vector(1,1,-1))
        assertVectorsAlmostEqual(self, self.rotQ4.rotate(Vector(1,1,1)), Vector(1,-1,1))
        assertVectorsAlmostEqual(self, self.rotQ5.rotate(Vector(1,1,1)), Vector(-1,-1,1))
        assertVectorsAlmostEqual(self, self.rotQ6.rotate(Vector(1,1,1)), Vector(-1,1,-1))
        assertVectorsAlmostEqual(self, self.rotQ7.rotate(Vector(1,1,1)), Vector(1,-1,-1))
        
    def test_constructor(self):
        assertQuaternionsAlmostEqual(self, self.q1, Quaternion(components=[ 1.0,2.0,3.0,4.0 ]))
        expectedQ3 = Quaternion(components=[ sin(pi/4), sin(pi/4), 0, 0])
        assertQuaternionsAlmostEqual(self, self.q3, expectedQ3)

    #Test ==
    def test_equal(self):
        self.assertEqual(self.q1, Quaternion(components=[ 1,2,3,4 ]))
        self.assertNotEqual(self.q1, Quaternion(components=[ 1,2,3,5 ]))
        #Check that can't compare Quaterion to a scalar
        with self.assertRaises(AttributeError):
            self.q1 == 33

    #Test +
    def test_add(self):
        self.assertEqual(self.q1 + self.q2, Quaternion(components=[0,0,0,0]))
        self.assertEqual(self.q1 + self.q1, Quaternion(components=[2,4,6,8]))
        #Check that adding a scalar and quaternion is not allowed
        with self.assertRaises(AttributeError):
            self.q1 + 33
    
    #Test -
    def test_subtract(self):
        self.assertEqual(self.q1 - self.q1, Quaternion(components=[0,0,0,0]))
        self.assertEqual(self.q1 - self.q2, Quaternion(components=[2,4,6,8]))
        #Check that adding a scalar and quaternion is not allowed
        with self.assertRaises(AttributeError):
            self.q1 - 33

    #Test * scalar
    def test_scalarMult(self):
        self.assertEqual(self.q1 * 2, Quaternion(components=[ 2,4,6,8 ]))

    #Test * quaternion
    def test_quaternionMult(self):
        self.assertEqual(self.q4 * self.q5, Quaternion(components=[ 0.5,1.25,1.5,0.25 ]))
        self.assertEqual(self.q4 * self.q4, Quaternion(components=[ 0,0,2,0 ]))

        # Check that order of rotations is left to right
        rot1 = Quaternion(axisOfRotation=Vector(0,0,1), angle=(pi/2))
        rot2 = Quaternion(axisOfRotation=Vector(1,0,0), angle=(pi/2))
        totalRotation = rot1*rot2

        initZAxis = Vector(0,0,1)
        finalZAxis = totalRotation.rotate(initZAxis)
        initXAxis = Vector(1,0,0)
        finalXAxis = totalRotation.rotate(initXAxis)

        assertVectorsAlmostEqual(self, finalXAxis, Vector(0, 1, 0))
        assertVectorsAlmostEqual(self, finalZAxis, Vector(1, 0 ,0))

    def test_scalarDivision(self):
        self.assertEqual(self.q1.__truediv__(2), Quaternion(components=[ 0.5,1,1.5,2 ]))

    def test_quaternionDivision(self):
        testQ = self.q4.__truediv__(self.q5)
        expectedQ = Quaternion(components=[0.7273, 0.1212, 0.2424, -0.6061])
        assertQuaternionsAlmostEqual(self, testQ, expectedQ, 3)

    def test_norm(self):
        self.assertEqual(self.q1.norm(), sqrt(30))

    def test_normalize(self):
        testQ = self.q4.normalize()
        expectedQ = Quaternion(components=[0.7071, 0.0, 0.7071, 0.0])
        assertQuaternionsAlmostEqual(self, testQ, expectedQ, 3)

    def test_conjugate(self):
        assertQuaternionsAlmostEqual(self, self.q4.conjugate(), Quaternion(components=[ 1,0,-1,0 ]))

    def test_inverse(self):
        assertQuaternionsAlmostEqual(self, self.q4.inverse(), Quaternion(components=[ 0.5,0.0,-0.5,0.0 ]))

    def test_rotationAxis(self):
        axisResult = self.rotQ.rotationAxis()
        axis = Vector(2,-3,5).normalize()
        self.assertAlmostEqual(axisResult.X, axis.X)
        self.assertAlmostEqual(axisResult.Y, axis.Y)
        self.assertAlmostEqual(axisResult.Z, axis.Z)

    def test_rotationAngle(self):
        self.assertAlmostEqual(self.rotQ.rotationAngle(), 2)

    def test_plot(self):
        self.q1.plotRotation(showPlot=False)

#If the file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
