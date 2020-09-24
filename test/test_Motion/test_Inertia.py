#Created by: Declan Quinn
#May 2019

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest

from MAPLEAF.Motion import Inertia
from MAPLEAF.Motion import Vector
from test.testUtilities import assertIterablesAlmostEqual


class TestInertia(unittest.TestCase):

    def test_InertiaAddition(self):
        mCG1 = Inertia(Vector(0,0,0), Vector(0,0,2), 10)
        mCG2 = Inertia(Vector(0,0,0), Vector(0,0,0), 20)

        m3 = mCG1.combineInertiasAboutPoint([ mCG2 ], Vector(0,0,0))
        self.assertEqual(m3.mass, 30)
        self.assertAlmostEqual(m3.CG.Z, 0.6666666666666)

    def test_combineInertiasAboutPoint_SlenderRod(self):
        # Test transforming from center to endpoint of slender rod
        rodMOI = Vector(1/12,1/12,0)
        rodCentroid = Vector(0,0,0)
        rodMass = 1
        rodCG = Vector(0,0,0)
        centroidInertia = Inertia(rodMOI, rodCentroid, rodMass, rodCG)

        rodEndPoint = Vector(0,0,-0.5)
        endPointInertia = centroidInertia.combineInertiasAboutPoint([], rodEndPoint)
        assertIterablesAlmostEqual(self, endPointInertia.MOI, Vector(1/3, 1/3, 0))

    def test_combineInertiasAboutPoint_TShapedBlock(self):
        # http://adaptivemap.ma.psu.edu/websites/A2_moment_intergrals/parallel_axis_theorem/pdf/ParallelAxis_WorkedProblem2.pdf
        # T-shaped block
        # Top block (1) dimensions are (4,2,0)
        # Bottom block (2) dimensions are (2,4,0)
        Ixx1 = 1/12 * 2 * 4**3
        Ixx2 = 1/12 * 4 * 2**3
        Iyy1 = Ixx2
        Iyy2 = Ixx1
        
        MOI1 = Vector(Ixx1, Iyy1, 0)
        centroid1 = Vector(0, 3, 0)
        mass1 = 8
        inertia1 = Inertia(MOI1, centroid1, mass1, centroid1)

        MOI2 = Vector(Ixx2, Iyy2, 0)
        centroid2 = Vector(0, 0, 0)
        mass2 = 8
        inertia2 = Inertia(MOI2, centroid2, mass2, centroid2)

        # Add them up at the overall centroid
        overallCentroid = Vector(0,1.5,0)
        combinedInertia = inertia1.combineInertiasAboutPoint([inertia2], overallCentroid)

        self.assertAlmostEqual(combinedInertia.MOI.X, 49.333333333333)
        self.assertAlmostEqual(combinedInertia.mass, 16)
        self.assertAlmostEqual(combinedInertia.CG, overallCentroid)
        self.assertAlmostEqual(combinedInertia.MOICentroidLocation, overallCentroid)

    def test_combineInertiasAboutPoint_Dumbbell(self):
        # http://adaptivemap.ma.psu.edu/websites/A2_moment_intergrals/parallel_axis_theorem/pdf/ParallelAxis_WorkedProblem3.pdf
        # Two 40kg 0.2m diameter weights at each end
        # One 20kg, 0.6m long bar between them
        # (0,0,0) is at the center of the rod between the dumbbells

        ballMOI = Vector(0.16,0.16,0.16)
        ballCentroid1 = Vector(-0.4, 0, 0)
        ballCentroid2 = Vector(0.4, 0, 0)
        ballMass = 40
        ball1Inertia = Inertia(ballMOI, ballCentroid1, ballMass, ballCentroid1)
        ball2Inertia = Inertia(ballMOI, ballCentroid2, ballMass, ballCentroid2)

        rodMOI = Vector(0,0.6,0.6)
        rodCentroid = Vector(0,0,0)
        rodMass = 20
        rodInertia = Inertia(rodMOI, rodCentroid, rodMass, rodCentroid)

        combinedInertia = ball1Inertia.combineInertiasAboutPoint([ball2Inertia, rodInertia], rodCentroid)
        self.assertAlmostEqual(combinedInertia.MOI.Y, 13.72)

    def test_combineInertias_Dumbbell(self):
        # http://adaptivemap.ma.psu.edu/websites/A2_moment_intergrals/parallel_axis_theorem/pdf/ParallelAxis_WorkedProblem3.pdf
        # Two 40kg 0.2m diameter weights at each end
        # One 20kg, 0.6m long bar between them
        # (0,0,0) is at the center of the rod between the dumbbells

        ballMOI = Vector(0.16,0.16,0.16)
        ballCentroid1 = Vector(-0.4, 0, 0)
        ballCentroid2 = Vector(0.4, 0, 0)
        ballMass = 40
        ball1Inertia = Inertia(ballMOI, ballCentroid1, ballMass, ballCentroid1)
        ball2Inertia = Inertia(ballMOI, ballCentroid2, ballMass, ballCentroid2)

        rodMOI = Vector(0,0.6,0.6)
        rodCentroid = Vector(0,0,0)
        rodMass = 20
        rodInertia = Inertia(rodMOI, rodCentroid, rodMass, rodCentroid)

        combinedInertia = ball1Inertia.combineInertias([ball2Inertia, rodInertia])
        self.assertAlmostEqual(combinedInertia.MOI.Y, 13.72)

    def test_Equality(self):
        inertia1 = Inertia(Vector(1,2,3), Vector(4,5,6), 7)
        inertia2 = Inertia(Vector(1,2,3), Vector(4,5,6), 7)
        self.assertEqual(inertia1, inertia2)

        inertia3 = Inertia(Vector(2,1,3), Vector(4,5,6), 7)
        self.assertNotEqual(inertia1, inertia3)

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
