#Created by: Henry Stoldt
#January 2019

#To run tests:
#In this file: [test_Vector.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest
from math import acos, sqrt
from test.testUtilities import assertVectorsAlmostEqual


from MAPLEAF.Motion import Vector


class TestVector(unittest.TestCase):
    def setUp(self):
        #Define Vectors to be used in the checks below
        self.v1 = Vector(1.0,2.0,3.0)
        self.v2 = Vector(-1.0,-2.0,-3.0)
        self.v3 = Vector(2.0,2.0,2.0)
        self.v4 = Vector(1.0,-3.0,4.0)
        self.v5 = Vector(3.0,4.0,0.0)
        self.v6 = Vector(4.0,4.0,2.0)
        self.v7 = Vector(1.0,0.0,3.0)
        self.v8 = Vector(5.0,5.0,0.0)

    def test_format(self):
        formattedVector = "{:1.2f}".format(self.v1)
        self.assertEqual(formattedVector, "1.00 2.00 3.00")

    def test_getItem(self):
        self.assertEqual(self.v1[0], 1)
        self.assertEqual(self.v1[1], 2)
        self.assertEqual(self.v1[2], 3)

    def test_iter(self):
        x, y, z = self.v1
        self.assertEqual(x, 1)
        self.assertEqual(y, 2)
        self.assertEqual(z, 3)

    def test_constructorParsing(self):
        self.assertEqual(Vector("(1,2,3)"), self.v1)
        self.assertEqual(Vector("(1, 2, 3)"), self.v1)
        self.assertEqual(Vector("(1;2;3)"), self.v1)
        self.assertEqual(Vector("(1; 2; 3)"), self.v1)
        self.assertEqual(Vector("(1 2 3)"), self.v1)
        self.assertEqual(Vector(" (694.25 200 3) "), Vector(694.25, 200, 3))
        self.assertEqual(Vector(" (694.25,200,3) "), Vector(694.25, 200, 3))
        self.assertEqual(Vector("(0.0  0.0  0.0)"), Vector(0,0,0))

    def test_Multiplication(self):
        self.assertEqual(self.v1*0.0,Vector(0,0,0))
        self.assertEqual(self.v1*1.0, self.v1)
        self.assertEqual(self.v1*(-1), self.v2)

    def test_rMul(self):
        assertVectorsAlmostEqual(self, 0.0*self.v1,Vector(0,0,0))
        assertVectorsAlmostEqual(self, 1.0*self.v1, self.v1)
        assertVectorsAlmostEqual(self, (-1)*self.v1, self.v2)

    #Test == operator
    def test_equal(self):
        self.assertEqual(self.v1, Vector(1,2,3))
        self.assertNotEqual(self.v1, Vector(1,2,4))

    #Test + operator
    def test_add(self):
        self.assertEqual(self.v1 + self.v2, Vector(0,0,0))
        self.assertEqual(self.v1 + self.v3, Vector(3,4,5))
        self.assertEqual(self.v2 + self.v3, Vector(1,0,-1))

    #Test - operator
    def test_subtract(self):
        self.assertEqual(self.v1 - self.v2, Vector(2,4,6))
        self.assertEqual(self.v2 - self.v1, Vector(-2,-4,-6))
    
    def test_neg(self):
        self.assertEqual(-self.v1, Vector(-1,-2,-3))

    def test_str(self):
        self.assertEqual(str(self.v1), "(1.0 2.0 3.0)")
    
    def test_dotProduct(self):
        self.assertEqual(self.v1 * self.v2, -14)
        self.assertEqual(self.v2 * self.v1, -14)
        self.assertEqual(self.v1 * self.v3, 12)
        self.assertEqual(self.v2 * self.v3, -12)
        self.assertEqual(self.v5 * self.v6, 28)

    def test_scalarMult(self):
        self.assertEqual(self.v1 * 3, Vector(3,6,9))
        self.assertEqual(self.v3 * 2, Vector(4,4,4))

    #Test getting the magnitude of the vector
    def test_length(self):
        self.assertEqual(self.v1.length(), sqrt(14))
        self.assertEqual(self.v3.length(), sqrt(12))
        self.assertEqual(self.v5.length(), 5)
        self.assertEqual(self.v6.length(), 6)

    #Test turning vector into a unit vector
    def test_normalize(self):
        self.assertEqual(self.v4.normalize(), Vector(1/sqrt(26), -3/sqrt(26), 4/sqrt(26)))

    #Test getting the angle between two vectors
    def test_angle(self):
        self.assertEqual(self.v5.angle(self.v6), acos(14.0/15))
        self.assertEqual(self.v6.angle(self.v5), acos(14.0/15))
        self.assertEqual(self.v7.angle(self.v8), acos(0.1 * sqrt(5)))
        self.assertEqual(self.v8.angle(self.v7), acos(0.1 * sqrt(5)))

    def test_crossProduct(self):
        self.assertEqual(self.v1.crossProduct(self.v1), Vector(0,0,0))
        self.assertEqual(self.v1.crossProduct(self.v3), Vector(-2,4,-2))
        #Can only take the cross product with another vector
        with self.assertRaises(TypeError):
            self.v1.crossProduct(33)

    #Test / operator
    def test_Division(self):
        self.assertEqual(self.v1.__truediv__(2), Vector(0.5,1,1.5))
        self.assertEqual(self.v2.__truediv__(2), Vector(-0.5,-1,-1.5))

    #Example, could put cleanup code here
    def tearDown(self):
        pass

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
