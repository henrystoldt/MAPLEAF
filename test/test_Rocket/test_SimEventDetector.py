# Created by: Henry Stoldt
# April 2020

import unittest

from MAPLEAF.Main import Simulation
from MAPLEAF.Motion import Vector
from MAPLEAF.Rocket import EventTypes, SimEventDetector


class TestSimEventDetector(unittest.TestCase):

    def setUp(self):
        simRunner = Simulation("MAPLEAF/Examples/Simulations/EventDetector.mapleaf", silent=True)
        self.rocket = simRunner.createRocket()
        self.rocket.rigidBody.time = 30 # Set to arbitrary time > 1 sec

        self.calledCount = 0
        self.eventDetector = SimEventDetector(self.rocket)

    def exampleCallbackFunction(self):
        self.calledCount += 1

    def test_DetectApogee(self):
        # Subscribing with enum
        self.eventDetector.subscribeToEvent(EventTypes.Apogee, self.exampleCallbackFunction)

        self.rocket.rigidBody.state.position = Vector(0,0, 10000)
        self.rocket.rigidBody.state.velocity = Vector(0,0,0.00001) # Upwards velocity so we're not at apogee yet

        self._checkEventTriggering()

    def test_DetectAscendingThroughAltitude(self):
        # Subscribing with string
        self.eventDetector.subscribeToEvent("ascendingThroughAltitude", self.exampleCallbackFunction, eventTriggerValue=10000)

        self.rocket.rigidBody.state.position = Vector(0,0,9999.999)
        self.rocket.rigidBody.state.velocity = Vector(0,0,1)

        self._checkEventTriggering()
    
    def test_DetectDescendingThroughAltitude(self):
        self.eventDetector.subscribeToEvent(EventTypes.DescendingThroughAltitude, self.exampleCallbackFunction, eventTriggerValue=100)

        self.rocket.rigidBody.state.position = Vector(0,0,100.001)
        self.rocket.rigidBody.state.velocity = Vector(0,0,-1)

        self._checkEventTriggering()

    def test_DetectTimeReached(self):
        self.eventDetector.subscribeToEvent(EventTypes.TimeReached, self.exampleCallbackFunction, eventTriggerValue=30.01)

        self.rocket.rigidBody.state.position = Vector(0,0,9999.999)
        self.rocket.rigidBody.state.velocity = Vector(0,0,1)
        self._checkEventTriggering()

    def test_DelayedTriggering(self):
        self.eventDetector.subscribeToEvent(EventTypes.Apogee, self.exampleCallbackFunction, triggerDelay=0.03)

        self.rocket.rigidBody.state.position = Vector(0,0, 10000)
        self.rocket.rigidBody.state.velocity = Vector(0,0,-0.00001) # Upwards velocity so we're already at apogee

        # Even though we're at/past apogee, shouldn't trigger right away because of delay
        self._checkEventTriggering()

    def test_motorBurnout(self):
        self.eventDetector.subscribeToEvent(EventTypes.MotorBurnout, self.exampleCallbackFunction)
        self.rocket.rigidBody.time = 4.99
        self._checkEventTriggering()

    def test_eventRemovalOrder(self):
        # These should trigger on the 1st, 2nd, and 3rd timesteps, respectively
        # If triggers are being removed in the correct order, the call back function will only be called once per time step
        # Also checks that the correct condition values (times) remain in self.eventDetector.conditionValues after every time step

        # Register events
        self.rocket.simEventDetector = self.eventDetector
        self.eventDetector.subscribeToEvent(EventTypes.TimeReached, self.exampleCallbackFunction, eventTriggerValue=30.01)
        self.eventDetector.subscribeToEvent(EventTypes.TimeReached, self.exampleCallbackFunction, eventTriggerValue=30.06)
        self.eventDetector.subscribeToEvent(EventTypes.TimeReached, self.exampleCallbackFunction, eventTriggerValue=30.11)

        # Check all values are present
        self._checkValuesInConditionValues([30.01, 30.06, 30.11])

        ### Time Step 1 ###
        self.rocket.timeStep(0.05)
        self.eventDetector.triggerEvents()
        self.assertEqual(self.calledCount, 1)
        self._checkValuesInConditionValues([30.06,30.11])
        
        ### Time Step 2 ###
        self.rocket.timeStep(0.05)
        self.eventDetector.triggerEvents()
        self.assertEqual(self.calledCount, 2)
        self.assertEqual(self.eventDetector.conditionValues, [30.11])
        
        ### Time Step 3 ###
        self.rocket.timeStep(0.05)
        self.eventDetector.triggerEvents()
        self.assertEqual(self.calledCount, 3)
        self.assertEqual(self.eventDetector.conditionValues, [])

    def _checkValuesInConditionValues(self, valueList):
        for value in valueList:
            self.assertTrue(value in self.eventDetector.conditionValues)

    def _checkEventTriggering(self):
        # Runs for all of the above test cases

        # Check that event is not triggered
        self.eventDetector.triggerEvents()
        self.assertEqual(self.calledCount, 0)

        # Take time step to advance simulation past the trigger point
        self.rocket.timeStep(0.05)

        # Check that event has triggered
        self.eventDetector.triggerEvents()
        self.assertEqual(self.calledCount, 1)

        # Check that event doesn't trigger again
        self.eventDetector.triggerEvents()
        self.assertEqual(self.calledCount, 1)
