# Created by: Henry Stoldt
# April 2020

import unittest

from MAPLEAF.Rocket.CompositeObject import CompositeObject
from MAPLEAF.Motion import Inertia
from MAPLEAF.Rocket import FixedMass
from test.testUtilities import assertIterablesAlmostEqual
from MAPLEAF.Motion import Vector


class FixedMassForCompositeObjectTest(FixedMass):
    def __init__(self, inertia):
        self.inertia = inertia

    def getInertia(self, _, __):
        return self.inertia

class TestCompositeObject(unittest.TestCase):

    def test_compositeObject_Dumbbell(self):
        # http://adaptivemap.ma.psu.edu/websites/A2_moment_intergrals/parallel_axis_theorem/pdf/ParallelAxis_WorkedProblem3.pdf
        # Two 40kg 0.2m diameter weights at each end
        # One 20kg, 0.6m long bar between them
        # (0,0,0) is at the center of the rod between the dumbbells

        ballMOI = Vector(0.16,0.16,0.16)
        ballCentroid1 = Vector(-0.4, 0, 0)
        ballCentroid2 = Vector(0.4, 0, 0)
        ballMass = 40
        ball1Inertia = Inertia(ballMOI, ballCentroid1, ballMass, ballCentroid1)
        ball1 = FixedMassForCompositeObjectTest(ball1Inertia)
        ball2Inertia = Inertia(ballMOI, ballCentroid2, ballMass, ballCentroid2)
        ball2 = FixedMassForCompositeObjectTest(ball2Inertia)

        rodMOI = Vector(0,0.6,0.6)
        rodCentroid = Vector(0,0,0)
        rodMass = 20
        rodInertia = Inertia(rodMOI, rodCentroid, rodMass, rodCentroid)
        rod = FixedMassForCompositeObjectTest(rodInertia)

        # Create composite object
        dumbbell = CompositeObject([ball1, ball2, rod])
        # Check that fixed mass caching is working properly
        self.assertEqual(len(dumbbell.fixedMassComponents), 3)

        # Get inertia and check it
        combinedInertia = dumbbell.getInertia(0, None)
        self.assertAlmostEqual(combinedInertia.MOI.Y, 13.72)
        self.assertAlmostEqual(combinedInertia.MOI.Z, 13.72)
        self.assertAlmostEqual(combinedInertia.MOI.X, 0.32)
        self.assertAlmostEqual(combinedInertia.mass, 100)
        self.assertAlmostEqual(combinedInertia.CG, Vector(0,0,0))

        # test the getMass function
        mass = dumbbell.getMass(0, None)
        self.assertAlmostEqual(100, mass)

        # test the getCG function
        CG = dumbbell.getCG(0, None)
        self.assertAlmostEqual(CG, Vector(0,0,0))

    def test_compositeObjectOfCompositeObjects(self):
        ballMOI = Vector(0.16,0.16,0.16)
        ballCentroid1 = Vector(-0.4, 0, 0)
        ballCentroid2 = Vector(0.4, 0, 0)
        ballCentroid3 = Vector(1.2, 0, 0)
        ballCentroid4 = Vector(2.0, 0, 0)
        ball1Inertia = Inertia(ballMOI, ballCentroid1, 10)
        ball2Inertia = Inertia(ballMOI, ballCentroid2, 20)
        ball3Inertia = Inertia(ballMOI, ballCentroid3, 30)
        ball4Inertia = Inertia(ballMOI, ballCentroid4, 40)
        Ball1 = FixedMassForCompositeObjectTest(ball1Inertia)
        Ball2 = FixedMassForCompositeObjectTest(ball2Inertia)
        Ball3 = FixedMassForCompositeObjectTest(ball3Inertia)
        Ball4 = FixedMassForCompositeObjectTest(ball4Inertia)

        innerCompObject1 = CompositeObject([Ball1, Ball2])
        innerCompObject2 = CompositeObject([Ball3, Ball4])

        completeCompositeObject = CompositeObject([Ball1, Ball2, Ball3, Ball4])
        outerCompObject = CompositeObject([innerCompObject1, innerCompObject2])

        # Check whether we get the same results from outer and complete composite objects
        innerMOI = outerCompObject.getInertia(0, None).MOI
        completeMOI = completeCompositeObject.getInertia(0, None).MOI

        assertIterablesAlmostEqual(self, innerMOI, completeMOI)


#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
