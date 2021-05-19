
#Created by: Declan Quinn
#May 2019

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest
from test.testUtilities import (assertInertiasAlmostEqual,
                                assertIterablesAlmostEqual)

from MAPLEAF.IO import SimDefinition, SubDictReader
from MAPLEAF.Motion import Inertia, Vector
from MAPLEAF.Rocket import Rocket, TabulatedMotor


class TestMotor(unittest.TestCase):
    def setUp(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/test2.mapleaf")
        rocketDictReader = SubDictReader("Rocket", simDef)
        self.motorRocket = Rocket(rocketDictReader)
    
    def test_MotorInitialization(self):
        motor = self.motorRocket.stages[0].getComponentsOfType(TabulatedMotor)[0]
        self.assertEqual(motor.initOxCG_Z, 2)
        self.assertEqual(motor.finalOxCG_Z, 2.5)
        self.assertEqual(motor.initFuelCG_Z, 3)
        self.assertEqual(motor.finalFuelCG_Z, 3)

        self.assertEqual(motor.times, [ 0, 0.01, 5])
        self.assertEqual(motor.thrustLevels, [0, 4000, 4000])
        self.assertEqual(motor.initialOxidizerWeight, 9.99)
        self.assertAlmostEqual(motor.initialFuelWeight, 1.4985)

    def test_MotorThrust(self):
        motor = self.motorRocket.stages[0].getComponentsOfType(TabulatedMotor)[0]

        def getThrust(time):
            return motor.getAppliedForce('fakeState', time, 'fakeEnv', 'fakeCG').force.Z

        self.assertEqual(getThrust(0), 0)
        self.assertEqual(getThrust(0.005), 2000)
        self.assertEqual(getThrust(0.01), 4000)
        self.assertEqual(getThrust(2), 4000)

    def test_MotorOxWeight(self):
        motor = self.motorRocket.stages[0].getComponentsOfType(TabulatedMotor)[0]

        def getOxWeight(time):
            return motor._getOxInertia(time).mass

        self.assertEqual(getOxWeight(0), 9.99)
        self.assertEqual(getOxWeight(5), 0)
        self.assertEqual(getOxWeight(0.01), 9.98)
        self.assertEqual(getOxWeight(0.02), 9.96)

    def test_MotorFuelWeight(self):
        motor = self.motorRocket.stages[0].getComponentsOfType(TabulatedMotor)[0]

        def getFuelWeight(time):
            return motor._getFuelInertia(time).mass

        self.assertAlmostEqual(getFuelWeight(0), 1.4985)
        self.assertAlmostEqual(getFuelWeight(5), 0)
        self.assertAlmostEqual(getFuelWeight(0.01), 1.497)
        self.assertAlmostEqual(getFuelWeight(0.02), 1.494)

    def test_MotorGetOxInertia(self):
        motor = self.motorRocket.stages[0].getComponentsOfType(TabulatedMotor)[0]
        
        assertInertiasAlmostEqual(self, motor._getOxInertia(0), Inertia(Vector(1,1,0.1), Vector(0,0,2), 9.99))
        assertInertiasAlmostEqual(self, motor._getOxInertia(5), Inertia(Vector(0,0,0), Vector(0,0,2.5), 0.0))

    def test_MotorGetFuelInertia(self):
        motor = self.motorRocket.stages[0].getComponentsOfType(TabulatedMotor)[0]
        
        assertInertiasAlmostEqual(self, motor._getFuelInertia(0), Inertia(Vector(0.15,0.15,0.02), Vector(0,0,3), 1.4985000000000002))
        assertInertiasAlmostEqual(self, motor._getFuelInertia(5), Inertia(Vector(0,0,0), Vector(0,0,3), 0))

    def test_MotorInertia(self):
        motorRocket = self.motorRocket
        motor = motorRocket.stages[0].getComponentsOfType(TabulatedMotor)[0]
        
        beginningInertia = Inertia(Vector(0.15,0.15,0.02), Vector(0,0,3), 1.4985000000000002) + Inertia(Vector(1,1,0.1), Vector(0,0,2), 9.99)
        finalInertia = Inertia(Vector(0,0,0), Vector(0,0,3), 0) + Inertia(Vector(0,0,0), Vector(0,0,0), 0.0)
        
        assertInertiasAlmostEqual(self, motor.getInertia(0, motorRocket.rigidBody.state), beginningInertia)
        assertInertiasAlmostEqual(self, motor.getInertia(5, motorRocket.rigidBody.state), finalInertia)
    
    def test_MotorMOI(self):
        motorRocket = self.motorRocket
        motor = self.motorRocket.stages[0].getComponentsOfType(TabulatedMotor)[0]

        ### Oxidizer MOI Tests ###
        def getOxMOI(time):
            return motor._getOxInertia(time).MOI

        ActualOxMOI = getOxMOI(0)
        expectedOxMOI = Vector(1.0, 1.0, 0.1)
        assertIterablesAlmostEqual(self, ActualOxMOI, expectedOxMOI)
        
        ActualOxMOI = getOxMOI(2.505)
        expectedOxMOI = Vector(0.5, 0.5, 0.05)
        assertIterablesAlmostEqual(self, ActualOxMOI, expectedOxMOI)

        ActualOxMOI = getOxMOI(5.0)
        expectedOxMOI = Vector(0, 0, 0)
        assertIterablesAlmostEqual(self, ActualOxMOI, expectedOxMOI)
        
        ActualOxMOI = getOxMOI(10)
        expectedOxMOI = Vector(0, 0, 0)
        assertIterablesAlmostEqual(self, ActualOxMOI, expectedOxMOI)

        ### Fuel MOI Tests ###
        def getFuelMOI(time):
            return motor._getFuelInertia(time).MOI

        ActualFuelMOI = getFuelMOI(0)
        expectedFuelMOI = Vector(0.15, 0.15, 0.02)
        assertIterablesAlmostEqual(self, ActualFuelMOI, expectedFuelMOI)
        
        ActualFuelMOI = getFuelMOI(2.505)
        expectedFuelMOI = Vector(0.075, 0.075, 0.01)
        assertIterablesAlmostEqual(self, ActualFuelMOI, expectedFuelMOI)
        
        ActualFuelMOI = getFuelMOI(5)
        expectedFuelMOI = Vector(0, 0, 0)
        assertIterablesAlmostEqual(self, ActualFuelMOI, expectedFuelMOI)
        
        ActualFuelMOI = getFuelMOI(10)
        expectedFuelMOI = Vector(0, 0, 0)
        assertIterablesAlmostEqual(self, ActualFuelMOI, expectedFuelMOI)

        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Jake1.mapleaf")
        rocketDictReader = SubDictReader("Rocket", simDef)
        jakeRocket = Rocket(rocketDictReader)

        motor = jakeRocket.stages[0].motor
        motorInertia = motor.getInertia(3,jakeRocket.rigidBody.state)        
        expectedInertiaAfterBurnout = Inertia(Vector(0,0,0), Vector(0,0,0), 0.0)
        assertInertiasAlmostEqual(self, motorInertia, expectedInertiaAfterBurnout)

    def test_getTotalImpulse(self):
        motor = self.motorRocket.stages[0].getComponentsOfType(TabulatedMotor)[0]
        impulse = motor.getTotalImpulse()
        expectedImpulse = 19980 # Ns
        self.assertAlmostEqual(expectedImpulse, impulse)

    def test_impulseAdjustFactor(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/test2.mapleaf")
        
        # Create normal motor
        rocketDictReader1 = SubDictReader("Rocket", simDef)
        motorRocket1 = Rocket(rocketDictReader1)
        motor1 = motorRocket1.stages[0].getComponentsOfType(TabulatedMotor)[0]

        impulseAdjustFactor = "1.14"

        # Create impulse-adjusted motor
        simDef.setValue("Rocket.Sustainer.Motor.impulseAdjustFactor", impulseAdjustFactor)
        rocketDictReader2 = SubDictReader("Rocket", simDef)
        motorRocket2 = Rocket(rocketDictReader2)
        motor2 = motorRocket2.stages[0].getComponentsOfType(TabulatedMotor)[0]

        # Check that total impulse has changed
        impulse1 = motor1.getTotalImpulse()
        impulse2 = motor2.getTotalImpulse()
        self.assertAlmostEqual(impulse2/impulse1, float(impulseAdjustFactor))

        # Check that burn time hasn't changed
        burnTime1 = motor1.times[-1]
        burnTime2 = motor2.times[-1]
        self.assertAlmostEqual(burnTime1, burnTime2)
        
    def test_burnTimeAdjustFactor(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/test2.mapleaf")
        
        # Create normal motor
        rocketDictReader1 = SubDictReader("Rocket", simDef)
        motorRocket1 = Rocket(rocketDictReader1)
        motor1 = motorRocket1.stages[0].getComponentsOfType(TabulatedMotor)[0]

        burnTimeAdjustFactor = "1.11"

        # Create impulse-adjusted motor
        simDef.setValue("Rocket.Sustainer.Motor.burnTimeAdjustFactor", burnTimeAdjustFactor)
        rocketDictReader2 = SubDictReader("Rocket", simDef)
        motorRocket2 = Rocket(rocketDictReader2)
        motor2 = motorRocket2.stages[0].getComponentsOfType(TabulatedMotor)[0]

        # Check that total impulse hasn't changed
        impulse1 = motor1.getTotalImpulse()
        impulse2 = motor2.getTotalImpulse()
        self.assertAlmostEqual(impulse1, impulse2)

        # Check that burn time has changed
        burnTime1 = motor1.times[-1]
        burnTime2 = motor2.times[-1]
        self.assertAlmostEqual(burnTime2/burnTime1, float(burnTimeAdjustFactor))

    def test_SolidMotor(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Jake1.mapleaf")
        rocketDictReader = SubDictReader("Rocket", simDef)
        motorRocket = Rocket(rocketDictReader)

        motor = motorRocket.stages[0].motor
        # Try getting thrust, if it works with 0 oxidizer flow rate, then we're good
        t = motor.getAppliedForce('fakeState', 0.12, 'fakeEnv', 'fakeCG').force.Z

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
