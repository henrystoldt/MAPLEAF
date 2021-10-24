import unittest

import numpy as np
import sys

from MAPLEAF.GNC import PIDController, TableScheduledGainPIDController, EquationScheduledGainPIDController


class TestPIDController(unittest.TestCase):
    def setUp(self):
        self.PID_scalar = PIDController(1, 2, 3)
        self.PID_vector = PIDController(np.array([1,2,3]), np.array([4,5,6]), np.array([7,8,9]), initialError=np.array([0,0,0]))

    def test_pidCreate_scalar(self):
        self.assertEqual(self.PID_scalar.P, 1)
        self.assertEqual(self.PID_scalar.I, 2)
        self.assertEqual(self.PID_scalar.D, 3)
        self.assertEqual(self.PID_scalar.lastError, 0)
        self.assertEqual(self.PID_scalar.errorIntegral, 0)
        self.assertEqual(self.PID_scalar.maxIntegralMagnitude, None)

    def test_pidCreate_vector(self):
        self.assertTrue(np.array_equal(self.PID_vector.P, np.array([1,2,3])))
        self.assertTrue(np.array_equal(self.PID_vector.I, np.array([4,5,6])))
        self.assertTrue(np.array_equal(self.PID_vector.D, np.array([7,8,9])))
        self.assertTrue(np.array_equal(self.PID_vector.lastError, np.array([0,0,0])))
        self.assertTrue(np.array_equal(self.PID_vector.errorIntegral, np.array([0,0,0])))
        self.assertEqual(self.PID_vector.maxIntegralMagnitude, None)

    def test_UpdatePIDCoefficients_vector(self):
        P2 = np.array([10, 11, 12])
        I2 = np.array([13, 14, 15])
        D2 = np.array([16, 17, 18])
        self.PID_vector.updateCoefficients(P2, I2, D2)

        self.assertTrue(np.array_equal(self.PID_vector.P, np.array([10,11,12])))
        self.assertTrue(np.array_equal(self.PID_vector.I, np.array([13,14,15])))
        self.assertTrue(np.array_equal(self.PID_vector.D, np.array([16,17,18])))
        self.assertTrue(np.array_equal(self.PID_vector.lastError, np.array([0,0,0])))
        self.assertTrue(np.array_equal(self.PID_vector.errorIntegral, np.array([0,0,0])))
        self.assertEqual(self.PID_vector.maxIntegralMagnitude, None)

    def test_getNewSetPoint(self):
        newError = 10
        dt = 1
        newSetPoint = self.PID_scalar.getNewSetPoint(newError, dt)
        self.assertEqual(newSetPoint, 50)

    def test_getNewSetPoint_vector(self):
        newError = np.array([10, 20, 30])
        dt = 1
        newSetPoint = self.PID_vector.getNewSetPoint(newError, dt)
        self.assertTrue(np.array_equal(newSetPoint, np.array([100, 250, 450])))

    def test_resetIntegral(self):
        newError = 10
        dt = 1
        newSetPoint = self.PID_scalar.getNewSetPoint(newError, dt)
        self.PID_scalar.resetIntegral()
        self.assertEqual(self.PID_scalar.errorIntegral, 0)
        
    def test_updateMaxIntegral_scalar(self):
        self.PID_scalar.updateMaxIntegral(1)

        newError = 10
        dt = 1
        newSetPoint = self.PID_scalar.getNewSetPoint(newError, dt)
        self.assertEqual(newSetPoint, 42)

    def test_updateMaxIntegral_vector(self):
        self.PID_vector.updateMaxIntegral(np.array([1,1,1]))

        newError = np.array([10, 20, 30])
        dt = 1
        newSetPoint = self.PID_vector.getNewSetPoint(newError, dt)
        self.assertTrue(np.array_equal(newSetPoint, np.array([84, 205, 366])))


class TestTableScheduledGainPIDController(unittest.TestCase):
    def setUp(self):
        self.TableScheduledGainPID = TableScheduledGainPIDController("MAPLEAF/Examples/TabulatedData/testPIDControlLaw.txt", 2, 2, 7)

    def test_getPIDCoeffs(self):
        MachNum, Alt = 0.15, 0
        expectedResult = np.array([ 7.5, 8.5, 9.5, 10.5, 11.5, 12.5 ])
        result = self.TableScheduledGainPID._getPIDCoeffs(MachNum, Alt)
        self.assertTrue(np.allclose(result, expectedResult))

        MachNum, Alt = 0.1, 0.5
        expectedResult = np.array([ 4, 5, 6, 7.5, 8.5, 9.5 ])
        result = self.TableScheduledGainPID._getPIDCoeffs(MachNum, Alt)
        self.assertTrue(np.allclose(result, expectedResult))

class TestEquationScheduledGainPIDController(unittest.TestCase):
  def setUp(self):
    coefficientList = [[1, 2, 3, 4, 5, 6],
                      [1, 2, 3, 4, 5, 6],
                      [1, 2, 3, 4, 5, 6]]

    parameterList = ["Mach Number", "Altitude"]

    equationOrder = 2

    self.equationScheduledGainPID = EquationScheduledGainPIDController(coefficientList, parameterList, equationOrder)

  def test_updateCoefficientsFromEquation(self):

    MachNum = 1
    Altitude = 10

    parameterValues = [MachNum, Altitude]
    ExpectedResult = 1 + 2*MachNum + 3*Altitude + 4*(MachNum**2) + 5*MachNum*Altitude + 6*(Altitude**2)

    self.equationScheduledGainPID.updateCoefficientsFromEquation(parameterValues)

    P = self.equationScheduledGainPID.P
    I = self.equationScheduledGainPID.I
    D = self.equationScheduledGainPID.D

    self.assertTrue(np.isclose(ExpectedResult,P))
    self.assertTrue(np.isclose(ExpectedResult,I))
    self.assertTrue(np.isclose(ExpectedResult,D))

  def test_EquationScheduledPIDOutput(self):

    Error = 1
    dt = 1

    MachNum = 1
    Altitude = 10

    parameterValues = [MachNum, Altitude]
    Gains = 1 + 2*MachNum + 3*Altitude + 4*(MachNum**2) + 5*MachNum*Altitude + 6*(Altitude**2)

    Derivative = (Error - 0)/dt
    Integral = Error*dt/2

    ExpectedResult = Gains*Error + Gains*Integral + Gains*Derivative

    self.equationScheduledGainPID.updateCoefficientsFromEquation(parameterValues)

    Output = self.equationScheduledGainPID.getNewSetPoint(Error,dt)

    self.assertTrue(np.isclose(ExpectedResult,Output))