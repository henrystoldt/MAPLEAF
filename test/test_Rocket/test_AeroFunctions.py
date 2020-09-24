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
from MAPLEAF.ENV import Environment
from MAPLEAF.IO.Logging import removeLogger
from MAPLEAF.Main import Simulation
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

    def test_BarrowmanCPLocation(self):
        # Check that for a cone, XCP = 0.666 Length
        length = 1
        baseArea = 1
        tipArea = 0
        volume = baseArea*length/3
        XCP_cone = AeroFunctions.Barrowman_GetCPLocation(length, tipArea, baseArea, volume)
        self.assertAlmostEqual(XCP_cone.Z, -0.666666666)

        # Check that for a tangent ogive nosecone, XCP = 0.466 Length
        SimRunner = Simulation("MAPLEAF/Examples/Simulations/test3.mapleaf", silent=True)
        rocket = SimRunner.createRocket()
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
