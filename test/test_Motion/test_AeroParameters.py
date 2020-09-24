#Created by: Declan Quinn
#May 2019

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import math
import unittest
from test.testUtilities import assertVectorsAlmostEqual

from MAPLEAF.ENV import Environment
from MAPLEAF.IO.Logging import removeLogger
from MAPLEAF.Motion import (AeroParameters, AngularVelocity, Quaternion,
                            RigidBodyState, Vector)


class TestAeroParameters(unittest.TestCase):
    def setUp(self):
        removeLogger()

        self.dummyVelocity1 = Vector(0, 0, 50)
        self.dummyVelocity2 = Vector(1, 0, 20)
        self.dummyVelocity3 = Vector(0, 0, -100)

        self.dummyOrientation1 = Quaternion(Vector(1, 0, 0), math.radians(2))
        self.dummyOrientation2 = Quaternion(Vector(1, 0, 0), math.radians(-2))
        self.dummyOrientation3 = Quaternion(Vector(0, 1, 0), math.radians(2))
        self.dummyOrientation4 = Quaternion(Vector(1, 0, 0), 0)
        self.dummyOrientation5 = Quaternion(Vector(1, 1, 0), math.radians(2))
        self.dummyOrientation6 = Quaternion(Vector(1,0,0), math.radians(90))
        
        self.environment = Environment(silent=True)
        self.currentConditions = self.environment.getAirProperties(Vector(0,0,200))

        self.rocketState1 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState3 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 500), Quaternion(Vector(1, 0, 0), math.radians(2)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState4 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, -200), Quaternion(Vector(1, 0, 0), math.radians(180)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState8 = RigidBodyState(Vector(0, 0, 200), Vector(20.04, -0.12, -52.78), Quaternion(Vector(0, 1, 0), math.radians(90)), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        self.correctDynamicPressure1 = self.currentConditions.Density * self.rocketState1.velocity.length()**2 / 2
        self.correctDynamicPressure2 = self.currentConditions.Density * self.rocketState3.velocity.length()**2 / 2

    def test_getTotalAOA(self):
        def getDummyState(vel, orientation):
            zeroVec = Vector(0,0,0)
            zeroAngVel = AngularVelocity(rotationVector=zeroVec)
            return RigidBodyState(zeroVec, vel, orientation, zeroAngVel)

        state1 = getDummyState(self.dummyVelocity1, self.dummyOrientation1)
        self.assertAlmostEqual(AeroParameters.getTotalAOA(state1, self.currentConditions), math.radians(2))
        
        state2 = getDummyState(self.dummyVelocity1, self.dummyOrientation2)
        self.assertAlmostEqual(AeroParameters.getTotalAOA(state2, self.currentConditions), math.radians(2))

        state3 = getDummyState(self.dummyVelocity2, self.dummyOrientation3)
        self.assertAlmostEqual(AeroParameters.getTotalAOA(state3, self.currentConditions), math.radians(0.862405226))

        state4 = getDummyState(self.dummyVelocity2, self.dummyOrientation4)
        self.assertAlmostEqual(AeroParameters.getTotalAOA(state4, self.currentConditions), math.radians(2.862405226))

        self.assertAlmostEqual(AeroParameters.getTotalAOA(self.rocketState4, self.currentConditions), 0)

        state5 = getDummyState(self.dummyVelocity3, self.dummyOrientation6)
        self.assertAlmostEqual(AeroParameters.getTotalAOA(state5, self.currentConditions), math.radians(90))

        # Prep rocket by calling getLocalFrameAirVel first to cache AOA result, then get cached result from getTotalAOA
        def testLocalFrameAirVelAOA(vel, orientation, expectedResult):
            state = RigidBodyState(Vector(0,0,0), vel, orientation, AngularVelocity(rotationVector=Vector(0,0,0)))
            self.assertAlmostEqual(AeroParameters.getTotalAOA(state, self.currentConditions), math.radians(expectedResult))
            
        testLocalFrameAirVelAOA(self.dummyVelocity1, self.dummyOrientation1, 2)
        testLocalFrameAirVelAOA(self.dummyVelocity1, self.dummyOrientation2, 2)
        testLocalFrameAirVelAOA(self.dummyVelocity2, self.dummyOrientation3, 0.862405226)
        testLocalFrameAirVelAOA(self.dummyVelocity2, self.dummyOrientation4, 2.862405226)
        testLocalFrameAirVelAOA(self.rocketState4.velocity, self.rocketState4.orientation, 0)
        testLocalFrameAirVelAOA(self.dummyVelocity3, self.dummyOrientation6, 90)

    def test_getRocketRollAngle(self):
        def getDummyState(vel, orientation):
            zeroVec = Vector(0,0,0)
            zeroAngVel = AngularVelocity(rotationVector=zeroVec)
            return RigidBodyState(zeroVec, vel, orientation, zeroAngVel)
        
        state1 = getDummyState(self.dummyVelocity1, self.dummyOrientation4)
        rollAngle1 = AeroParameters.getRollAngle(state1, self.currentConditions)

        state2 = getDummyState(self.dummyVelocity1, self.dummyOrientation1)
        rollAngle2 = AeroParameters.getRollAngle(state2, self.currentConditions)

        state3 = getDummyState(self.dummyVelocity1, self.dummyOrientation2)
        rollAngle3 = AeroParameters.getRollAngle(state3, self.currentConditions)

        state4 = getDummyState(self.dummyVelocity1, self.dummyOrientation5)
        rollAngle4 = AeroParameters.getRollAngle(state4, self.currentConditions)

        state5 = getDummyState(self.dummyVelocity2, self.dummyOrientation4)
        rollAngle5 = AeroParameters.getRollAngle(state5, self.currentConditions)

        state6 = getDummyState(self.dummyVelocity1, self.dummyOrientation3)
        rollAngle6 = AeroParameters.getRollAngle(state6, self.currentConditions)

        self.assertAlmostEqual(rollAngle1, 180)
        self.assertAlmostEqual(rollAngle2, 90)
        self.assertAlmostEqual(rollAngle3, 270)
        self.assertAlmostEqual(rollAngle4, 135)
        self.assertAlmostEqual(rollAngle5, 0)
        self.assertAlmostEqual(rollAngle6, 180)

        # Prep rocket by calling getLocalFrameAirVel first to cache AOA result, then get cached result from getTotalAOA
        def testLocalFrameAirVelRollAngle(vel, orientation, expectedResult):
            state = RigidBodyState(Vector(0,0,0), vel, orientation, AngularVelocity(rotationVector=Vector(0,0,0)))
            self.assertAlmostEqual(AeroParameters.getRollAngle(state, self.currentConditions), expectedResult)

        testLocalFrameAirVelRollAngle(self.dummyVelocity1, self.dummyOrientation4, 180)
        testLocalFrameAirVelRollAngle(self.dummyVelocity1, self.dummyOrientation1, 90)
        testLocalFrameAirVelRollAngle(self.dummyVelocity1, self.dummyOrientation2, 270)
        testLocalFrameAirVelRollAngle(self.dummyVelocity1, self.dummyOrientation5, 135)
        testLocalFrameAirVelRollAngle(self.dummyVelocity2, self.dummyOrientation4, 0)
        testLocalFrameAirVelRollAngle(self.dummyVelocity1, self.dummyOrientation3, 180)

    def test_getNormalAeroForceDirection(self):
        zeroAngVel = AngularVelocity(0,0,0)
        zeroPos = Vector(0,0,0)
        
        state1 = RigidBodyState(zeroPos, self.dummyVelocity1, self.dummyOrientation1, zeroAngVel)
        assertVectorsAlmostEqual(self, AeroParameters.getNormalAeroForceDirection(state1, self.currentConditions), Vector(0, -1, 0))

        state2 = RigidBodyState(zeroPos, self.dummyVelocity1, self.dummyOrientation2, zeroAngVel)
        assertVectorsAlmostEqual(self, AeroParameters.getNormalAeroForceDirection(state2, self.currentConditions), Vector(0, 1, 0))

        state3 = RigidBodyState(zeroPos, self.dummyVelocity1, self.dummyOrientation3, zeroAngVel)
        assertVectorsAlmostEqual(self, AeroParameters.getNormalAeroForceDirection(state3, self.currentConditions), Vector(1, 0, 0))

        state4 = RigidBodyState(zeroPos, self.dummyVelocity1, self.dummyOrientation4, zeroAngVel)
        assertVectorsAlmostEqual(self, AeroParameters.getNormalAeroForceDirection(state4, self.currentConditions), Vector(1, 0, 0))

        state5 = RigidBodyState(zeroPos, self.dummyVelocity1, self.dummyOrientation5, zeroAngVel)
        assertVectorsAlmostEqual(self, AeroParameters.getNormalAeroForceDirection(state5, self.currentConditions), Vector(0.707, -0.707, 0),3)

        assertVectorsAlmostEqual(self, AeroParameters.getNormalAeroForceDirection(self.rocketState8, self.currentConditions), Vector(-0.99999, 0.0022736, 0),3)

        # Prep rocket by calling getLocalFrameAirVel first to cache AOA result, then get cached result from getTotalAOA
        def testLocalFrameAirVelNormalForceDir(vel, orientation, expectedResult, precision=7):
            state = RigidBodyState(Vector(0,0,0), vel, orientation, AngularVelocity(0,0,0))
            assertVectorsAlmostEqual(self, AeroParameters.getNormalAeroForceDirection(state, self.currentConditions), expectedResult, precision)

        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation1, Vector(0, -1, 0))
        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation2, Vector(0, 1, 0))
        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation3, Vector(1, 0, 0))
        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation4, Vector(1, 0, 0))
        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation5, Vector(0.707, -0.707, 0), 3)
        testLocalFrameAirVelNormalForceDir(self.rocketState8.velocity, self.rocketState8.orientation, Vector(-0.99999, 0.0022736, 0),3)

    def test_dynamicPressure(self):
        self.assertAlmostEqual(AeroParameters.getDynamicPressure(self.rocketState1,self.currentConditions), self.correctDynamicPressure1)
        self.assertAlmostEqual(AeroParameters.getDynamicPressure(self.rocketState3,self.currentConditions), self.correctDynamicPressure2)

    def test_getMachNumber(self):
        environment = self.environment.getAirProperties(Vector(0,0,0))
        # Assumes gamma=1.4, R=287
        self.assertAlmostEqual(AeroParameters.getMachNumber(self.rocketState1, environment), 0.587781235)

    def test_getReynoldsNumber(self):
        environment = self.environment.getAirProperties(Vector(0,0,0))
        self.assertAlmostEqual(AeroParameters.getReynoldsNumber(self.rocketState1, environment, 1), 13701279.02972, 2)

    def test_getBeta(self):
        self.assertAlmostEqual(AeroParameters.getBeta(0.5), 0.86602540)
        self.assertAlmostEqual(AeroParameters.getBeta(1.5), 1.118033989)

    def test_getAOA(self):
        test1State = RigidBodyState(velocity=Vector(1, 0, 1))
        expectedAOA = math.radians(-45)
        computedAOA = AeroParameters.getAOA(test1State, self.currentConditions)
        self.assertAlmostEqual(expectedAOA, computedAOA)

    def test_getAOSS(self):
        test1State = RigidBodyState(velocity=Vector(0, 1, 1))
        expectedAOA = math.radians(45)
        computedAOA = AeroParameters.getAOSS(test1State, self.currentConditions)
        self.assertAlmostEqual(expectedAOA, computedAOA)

if __name__ == "__main__":
    unittest.main()
