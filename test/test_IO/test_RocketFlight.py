#Created by: Henry Stoldt
#April 2020

#To run tests:
#In this file: [test_RocketFlight.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest

from MAPLEAF.Motion import RigidBodyState_3DoF
from MAPLEAF.IO import RocketFlight
from MAPLEAF.Motion import Vector

class TestRocketFlight(unittest.TestCase):
    def setUp(self):
        pos1 = Vector(0, 0, 10)
        vel1 = Vector(0, 0, -1)
        pos2 = Vector(1, 2, 5)
        vel2 = Vector(0, 0, -1)
        t1 = 0.0
        t2 = 1.0
        state1 = RigidBodyState_3DoF(pos1, vel1)
        state2 = RigidBodyState_3DoF(pos2, vel2)
        
        self.flight = RocketFlight()
        self.flight.rigidBodyStates = [ state1, state2 ]
        self.flight.times = [ t1, t2 ]
        self.flight.actuatorDefls = None

    def test_getApogee(self):
        self.assertAlmostEqual(self.flight.getApogee(), 10)

    def test_getFlightTime(self):
        self.assertAlmostEqual(self.flight.getFlightTime(), 1.0)

    def test_getMaxSpeed(self):
        self.assertAlmostEqual(self.flight.getMaxSpeed(), 1.0)

    def test_getMaxHorizVel(self):
        self.assertAlmostEqual(self.flight.getMaxHorizontalVel(), 0.0)

    def test_LandingLocation(self):
        self.assertAlmostEqual(self.flight.getLandingLocation(), Vector(2, 4, 0))