#Created by: Declan Quinn
#May 2019

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import math
import unittest
from test.testUtilities import assertVectorsAlmostEqual

import MAPLEAF.Rocket.AeroFunctions as AeroFunctions
from MAPLEAF.ENV.Environment import Environment
from MAPLEAF.IO.Logging import removeLogger
from MAPLEAF.Main import SingleSimRunner
from MAPLEAF.Motion import (AngularVelocity, ForceMomentSystem, Quaternion,
                            RigidBodyState, Vector)
from MAPLEAF.Rocket import NoseCone


def createStateWithRe(reynoldsNumber, env, length=1):
    velocity = reynoldsNumber * env.DynamicViscosity / (env.Density * length)
    # Zero AOA state heading forward at the velocity required to match the Reynolds number
    return RigidBodyState(velocity=Vector(0,0,velocity))

class TestAeroFunctions(unittest.TestCase):
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
        self.assertAlmostEqual(AeroFunctions.getTotalAOA(state1, self.currentConditions), math.radians(2))
        
        state2 = getDummyState(self.dummyVelocity1, self.dummyOrientation2)
        self.assertAlmostEqual(AeroFunctions.getTotalAOA(state2, self.currentConditions), math.radians(2))

        state3 = getDummyState(self.dummyVelocity2, self.dummyOrientation3)
        self.assertAlmostEqual(AeroFunctions.getTotalAOA(state3, self.currentConditions), math.radians(0.862405226))

        state4 = getDummyState(self.dummyVelocity2, self.dummyOrientation4)
        self.assertAlmostEqual(AeroFunctions.getTotalAOA(state4, self.currentConditions), math.radians(2.862405226))

        self.assertAlmostEqual(AeroFunctions.getTotalAOA(self.rocketState4, self.currentConditions), 0)

        state5 = getDummyState(self.dummyVelocity3, self.dummyOrientation6)
        self.assertAlmostEqual(AeroFunctions.getTotalAOA(state5, self.currentConditions), math.radians(90))

        # Prep rocket by calling getLocalFrameAirVel first to cache AOA result, then get cached result from getTotalAOA
        def testLocalFrameAirVelAOA(vel, orientation, expectedResult):
            state = RigidBodyState(Vector(0,0,0), vel, orientation, AngularVelocity(rotationVector=Vector(0,0,0)))
            self.assertAlmostEqual(AeroFunctions.getTotalAOA(state, self.currentConditions), math.radians(expectedResult))
            
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
        rollAngle1 = AeroFunctions.getRollAngle(state1, self.currentConditions)

        state2 = getDummyState(self.dummyVelocity1, self.dummyOrientation1)
        rollAngle2 = AeroFunctions.getRollAngle(state2, self.currentConditions)

        state3 = getDummyState(self.dummyVelocity1, self.dummyOrientation2)
        rollAngle3 = AeroFunctions.getRollAngle(state3, self.currentConditions)

        state4 = getDummyState(self.dummyVelocity1, self.dummyOrientation5)
        rollAngle4 = AeroFunctions.getRollAngle(state4, self.currentConditions)

        state5 = getDummyState(self.dummyVelocity2, self.dummyOrientation4)
        rollAngle5 = AeroFunctions.getRollAngle(state5, self.currentConditions)

        state6 = getDummyState(self.dummyVelocity1, self.dummyOrientation3)
        rollAngle6 = AeroFunctions.getRollAngle(state6, self.currentConditions)

        self.assertAlmostEqual(rollAngle1, 180)
        self.assertAlmostEqual(rollAngle2, 90)
        self.assertAlmostEqual(rollAngle3, 270)
        self.assertAlmostEqual(rollAngle4, 135)
        self.assertAlmostEqual(rollAngle5, 0)
        self.assertAlmostEqual(rollAngle6, 180)

        # Prep rocket by calling getLocalFrameAirVel first to cache AOA result, then get cached result from getTotalAOA
        def testLocalFrameAirVelRollAngle(vel, orientation, expectedResult):
            state = RigidBodyState(Vector(0,0,0), vel, orientation, AngularVelocity(rotationVector=Vector(0,0,0)))
            self.assertAlmostEqual(AeroFunctions.getRollAngle(state, self.currentConditions), expectedResult)

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
        assertVectorsAlmostEqual(self, AeroFunctions.getNormalAeroForceDirection(state1, self.currentConditions), Vector(0, -1, 0))

        state2 = RigidBodyState(zeroPos, self.dummyVelocity1, self.dummyOrientation2, zeroAngVel)
        assertVectorsAlmostEqual(self, AeroFunctions.getNormalAeroForceDirection(state2, self.currentConditions), Vector(0, 1, 0))

        state3 = RigidBodyState(zeroPos, self.dummyVelocity1, self.dummyOrientation3, zeroAngVel)
        assertVectorsAlmostEqual(self, AeroFunctions.getNormalAeroForceDirection(state3, self.currentConditions), Vector(1, 0, 0))

        state4 = RigidBodyState(zeroPos, self.dummyVelocity1, self.dummyOrientation4, zeroAngVel)
        assertVectorsAlmostEqual(self, AeroFunctions.getNormalAeroForceDirection(state4, self.currentConditions), Vector(1, 0, 0))

        state5 = RigidBodyState(zeroPos, self.dummyVelocity1, self.dummyOrientation5, zeroAngVel)
        assertVectorsAlmostEqual(self, AeroFunctions.getNormalAeroForceDirection(state5, self.currentConditions), Vector(0.707, -0.707, 0),3)

        assertVectorsAlmostEqual(self, AeroFunctions.getNormalAeroForceDirection(self.rocketState8, self.currentConditions), Vector(-0.99999, 0.0022736, 0),3)

        # Prep rocket by calling getLocalFrameAirVel first to cache AOA result, then get cached result from getTotalAOA
        def testLocalFrameAirVelNormalForceDir(vel, orientation, expectedResult, precision=7):
            state = RigidBodyState(Vector(0,0,0), vel, orientation, AngularVelocity(0,0,0))
            assertVectorsAlmostEqual(self, AeroFunctions.getNormalAeroForceDirection(state, self.currentConditions), expectedResult, precision)

        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation1, Vector(0, -1, 0))
        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation2, Vector(0, 1, 0))
        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation3, Vector(1, 0, 0))
        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation4, Vector(1, 0, 0))
        testLocalFrameAirVelNormalForceDir(self.dummyVelocity1, self.dummyOrientation5, Vector(0.707, -0.707, 0), 3)
        testLocalFrameAirVelNormalForceDir(self.rocketState8.velocity, self.rocketState8.orientation, Vector(-0.99999, 0.0022736, 0),3)

    def test_dynamicPressure(self):
        self.assertAlmostEqual(AeroFunctions.getDynamicPressure(self.rocketState1,self.currentConditions), self.correctDynamicPressure1)
        self.assertAlmostEqual(AeroFunctions.getDynamicPressure(self.rocketState3,self.currentConditions), self.correctDynamicPressure2)

    def test_getMachNumber(self):
        environment = self.environment.getAirProperties(Vector(0,0,0))
        # Assumes gamma=1.4, R=287
        self.assertAlmostEqual(AeroFunctions.getMachNumber(self.rocketState1, environment), 0.587781235)

    def test_getReynoldsNumber(self):
        environment = self.environment.getAirProperties(Vector(0,0,0))
        self.assertAlmostEqual(AeroFunctions.getReynoldsNumber(self.rocketState1, environment, 1), 13701279.02972, 2)

    def test_getBeta(self):
        self.assertAlmostEqual(AeroFunctions.getBeta(0.5), 0.86602540)
        self.assertAlmostEqual(AeroFunctions.getBeta(1.5), 1.118033989)

    def test_BarrowmanCPLocation(self):
        # Check that for a cone, XCP = 0.666 Length
        length = 1
        baseArea = 1
        tipArea = 0
        volume = baseArea*length/3
        XCP_cone = AeroFunctions.Barrowman_GetCPLocation(length, tipArea, baseArea, volume)
        self.assertAlmostEqual(XCP_cone.Z, -0.666666666)

        # Check that for a tangent ogive nosecone, XCP = 0.466 Length
        SimRunner = SingleSimRunner("MAPLEAF/Examples/Simulations/test3.mapleaf", silent=True)
        rocket = SimRunner.prepRocketForSingleSimulation()
        rocketNosecone = rocket.stages[0].getComponentsOfType(NoseCone)[0]
        noseconeLength = rocketNosecone.length
        expectedCp = rocketNosecone.position + Vector(0,0,-0.46666*noseconeLength)
        actualCp = rocketNosecone.CPLocation
        assertVectorsAlmostEqual(self, expectedCp, actualCp, 2)

    def test_CPZ(self):
        force = Vector(0, 1, 0)
        location = Vector(0, 0, 0)
        moment = Vector(1, 0, 0)
        testForce = ForceMomentSystem(force, location, moment)

        expectedCPZ = -1
        calculatedCPZ = AeroFunctions._getCPZ(testForce)
        self.assertAlmostEqual(expectedCPZ, calculatedCPZ)

    def test_getDragToAxialForceFactor(self):
        # Should be 0 at 0, 1.3 at 17, and 0 at 90
        AOAsToTest = [ 0, 17, 90 ]
        AOAsToTest = [ math.radians(AOA) for AOA in AOAsToTest ]
        results = [ AeroFunctions.getDragToAxialForceFactor(AOA) for AOA in AOAsToTest ]
        expectedResults = [ 1, 1.3, 0 ]
        for i in range(3):
            self.assertAlmostEqual(results[i], expectedResults[i], 3)

    def test_getAOA(self):
        test1State = RigidBodyState(velocity=Vector(1, 0, 1))
        expectedAOA = math.radians(-45)
        computedAOA = AeroFunctions.getAOA(test1State, self.currentConditions)
        self.assertAlmostEqual(expectedAOA, computedAOA)

    def test_getAOSS(self):
        test1State = RigidBodyState(velocity=Vector(0, 1, 1))
        expectedAOA = math.radians(45)
        computedAOA = AeroFunctions.getAOSS(test1State, self.currentConditions)
        self.assertAlmostEqual(expectedAOA, computedAOA)

    def test_getSkinFrictionCoeff(self):
        env = self.currentConditions
        length = 1

        def checkSkinFriction(Re, roughness, Mach, expectedCf, fullyTurbulent=True):
            state = createStateWithRe(Re, env, length)
            coeff = AeroFunctions.getSkinFrictionCoefficient(state, env, length, Mach, roughness, fullyTurbulent)
            self.assertAlmostEqual(coeff, expectedCf)

        # Min Re
        checkSkinFriction(0, 0, 0, 0.0148)
        checkSkinFriction(9999, 0, 0, 0.0148)
        
        # Fully Turbulent Flow
        checkSkinFriction(10000, 0, 0, 0.014815997)
        checkSkinFriction(100000, 0, 0, 0.007343512274)

        # Transitional Flow
        checkSkinFriction(10000, 0, 0, 0.01328, False)
        checkSkinFriction(500001, 0, 0, 0.001641693326, False)
        checkSkinFriction(5000000, 0, 0, 0.002911385467, False)

        # Flow with roughness
        roughness = 0.00002
        Recrit = 3888660.338
        checkSkinFriction(3888500, 0.00002, 0, 0.002958676686, False) # Check transitional flow again
        checkSkinFriction(3888500, 0.00002, 0, 0.003395863262, True) # Check turbulent flow again

        checkSkinFriction(3888700, 0.00002, 0, 0.003675834736, False) # Check roughness-determined friction
        checkSkinFriction(3888700, 0.00002, 0, 0.003675834736, True) # Check roughness-determined friction
        
    def test_getSubsonicCompressibilityCorrectionFactor(self):
        smooth = True
        Mach = 0.5
        expected = 0.975
        self.assertAlmostEqual(AeroFunctions._subSonicSkinFrictionCompressiblityCorrectionFactor(Mach, smooth), expected)

        smooth = False
        Mach = 0.5
        expected = 0.97
        self.assertAlmostEqual(AeroFunctions._subSonicSkinFrictionCompressiblityCorrectionFactor(Mach, smooth), expected)

    def test_supersonicSkinFrictionCompressibilityCorrection(self):
        smooth = True
        Mach = 1.5
        expected = 0.844791628
        self.assertAlmostEqual(AeroFunctions._supersonicSkinFrictionCompressiblityCorrectionFactor(Mach, smooth), expected)

    def test_getCylindricalSkinFriction(self):
        env = self.currentConditions
        length = 1
        Mach = 0
        rough = 0
        Area = 1
        finenessRatio = 2
        turbulent = True

        def checkSkinFriction(Re, roughness, Mach, finenessRatio, expectedCf, fullyTurbulent=True):
            state = createStateWithRe(Re, env, length)
            coeff, rollDamping = AeroFunctions.getCylindricalSkinFrictionDragCoefficientAndRollDampingMoment(state, env, length, Mach, roughness, 1, 1, finenessRatio, fullyTurbulent)
            self.assertAlmostEqual(coeff, expectedCf)
            self.assertAlmostEqual(rollDamping.length(), 0)

        checkSkinFriction(9999, rough, Mach, finenessRatio, 0.0185)

    def test_BarrowmanGetCN(self):
        AOA = 0.1 #rad
        Aref = 1
        xArea_top = 1
        xArea_bottom = 2
        expected = 0.199666833
        self.assertAlmostEqual(AeroFunctions.Barrowman_GetCN(AOA, Aref, xArea_top, xArea_bottom), expected)

    def test_BarrowmanCPLocation(self):
        length = 1
        xArea_top = 1
        xArea_bottom = 2
        vol = 1.5

        expected = Vector(0,0,-0.5)
        assertVectorsAlmostEqual(self, expected, AeroFunctions.Barrowman_GetCPLocation(length, xArea_top, xArea_bottom, vol))

        xArea_top = 1
        xArea_bottom = 1
        expected = Vector(0,0,-0.5)
        assertVectorsAlmostEqual(self, expected, AeroFunctions.Barrowman_GetCPLocation(length, xArea_top, xArea_bottom, vol))

if __name__ == "__main__":
    unittest.main()
