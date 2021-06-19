import unittest

import numpy as np

from MAPLEAF.Motion import AeroParameters
from MAPLEAF.GNC import (FirstOrderActuator, FirstOrderSystem,
                           TableInterpolatingActuatorController)


class TestFirstOrderSystem(unittest.TestCase):
    def setUp(self):
        self.testSystem = FirstOrderSystem(1)

    def test_FirstOrderSystem(self):
        result = self.testSystem.getPosition(1, 0, 0, 1)
        self.assertAlmostEqual(result, 0.63212056)

    def test_FirstOrderSystem_Vector(self):
        currentTime = 1
        lastTime = 0
        lastPosition = np.array([0,0,0,0])
        lastTarget = np.array([1,2,3,4])
        newPos = self.testSystem.getPosition(currentTime, lastTime, lastPosition, lastTarget)
        self.assertTrue(np.allclose(newPos, np.array([0.63212056,0.63212056*2,0.63212056*3,0.63212056*4])))

class TestTableInterpActuatorControlAndPointAtTarget(unittest.TestCase):
    def setUp(self):
        self.actuatorList = []
        for i in range(4):
            actuator = FirstOrderActuator(0.1)
            self.actuatorList.append(actuator)

        keyFnVector = [ AeroParameters.getMachNumber, AeroParameters.getAltitude ]
        self.interpActuatorController = TableInterpolatingActuatorController("MAPLEAF/Examples/TabulatedData/testFinDeflectionLaw.txt", 5, keyFnVector, self.actuatorList)

    def test_interpDeflectionTargets(self):
        def getfakeMach(*args):
            return 0.5

        def getfakeAltitude(*args):
            return 0

        # Test Case 1
        keyFnVector = [ getfakeMach, getfakeAltitude ]
        self.interpActuatorController.keyFunctionList = keyFnVector
        self.interpActuatorController.setTargetActuatorDeflections(np.array([0, 0, 0]), "fakeState", "fakeEnv", 0)

        actuatorTargets = [ actuator.targetDeflection for actuator in self.actuatorList ]
        expectedResults = np.array([ 0.5, 3, 5.5, 8 ])
        self.assertTrue(np.allclose(actuatorTargets, expectedResults))

        # Test Case 2
        keyFnVector = [ getfakeMach, getfakeMach ]
        self.interpActuatorController.keyFunctionList = keyFnVector
        self.interpActuatorController.setTargetActuatorDeflections(np.array([0.5, 0.5, 0.5]), "fakeState", "fakeEnv", 0)

        actuatorTargets = [ actuator.targetDeflection for actuator in self.actuatorList ]
        expectedResults = np.array([ 7.5, 20, 32.5, 45 ])
        self.assertTrue(np.allclose(actuatorTargets, expectedResults))

    def test_NoNanValuesWhenInterpolatingOutOfBounds(self):

        def getFakeMach2(*args):
            return 10

        # Test that there are no NaN values when interpolating out of bounds
        keyFnVector = [ getFakeMach2, getFakeMach2 ]
        self.interpActuatorController.keyFunctionList = keyFnVector
        self.interpActuatorController.setTargetActuatorDeflections(np.array([0.5, 0.5, 0.5]), "fakeState", "fakeEnv", 0)

        actuatorTargets = [ actuator.targetDeflection for actuator in self.actuatorList ]
        self.assertFalse(np.isnan(actuatorTargets).any())

    
class TestFirstOrderActuator(unittest.TestCase):
    def setUp(self):
        self.actuator = FirstOrderActuator(1, maxDeflection=15, minDeflection=-15)

    def test_FirstOrderDeflection(self):
        self.actuator.setTargetDeflection(1, 0)
        deflAt1Second = self.actuator.getDeflection(1)
        self.assertAlmostEqual(deflAt1Second, 0.63212056)

    def test_DeflectionLimiting(self):
        self.actuator.setTargetDeflection(25, 0)
        self.assertEqual(self.actuator.targetDeflection, 15)

        self.actuator.setTargetDeflection(-25, 0)
        self.assertEqual(self.actuator.targetDeflection, -15)