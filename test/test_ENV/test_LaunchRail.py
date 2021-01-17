import math
import unittest
from test.testUtilities import assertVectorsAlmostEqual

from MAPLEAF.ENV import Environment
from MAPLEAF.ENV import LaunchRail
from MAPLEAF.IO import SimDefinition
from MAPLEAF.Motion import AngularVelocity
from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import Vector
from MAPLEAF.Motion import ForceMomentSystem
from MAPLEAF.Motion import RigidBodyState


class TestLaunchRail(unittest.TestCase):
    def setUp(self):        
        zeroVec = Vector(0,0,0)
        zeroQuat = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        zeroAngVel = AngularVelocity(rotationVector=zeroVec)
        self.initState = RigidBodyState(zeroVec, zeroVec, zeroQuat, zeroAngVel)

        startPos = zeroVec
        direction = Vector(1,0,1)
        length = 5
        self.rail = LaunchRail(startPos, direction, length)

    def test_zeroSag(self):
        unadjustedForce = ForceMomentSystem(Vector(10,9,-10), moment=Vector(10,11,12))
        adjustedForce = self.rail.applyLaunchTowerForce(self.initState, 0, unadjustedForce)

        # Check that force is zero (don't let object sag/fall)
        assertVectorsAlmostEqual(self, adjustedForce.force, Vector(0,0,0))
        # Check moments are zero
        assertVectorsAlmostEqual(self, adjustedForce.moment, Vector(0,0,0))

    def test_alignForceWithRail(self):
        unadjustedForce = ForceMomentSystem(Vector(10,9,11), moment=Vector(10,11,12))
        adjustedForce = self.rail.applyLaunchTowerForce(self.initState, 0, unadjustedForce)

        # Check that force direction is now aligned with rail
        forceDirection = adjustedForce.force.normalize()
        assertVectorsAlmostEqual(self, self.rail.initialDirection, forceDirection)
        # Check moments are zero
        assertVectorsAlmostEqual(self, adjustedForce.moment, Vector(0,0,0))

    def test_LeaveRail(self):
        unadjustedForce = ForceMomentSystem(Vector(10,9,11), moment=Vector(10,11,12))
        self.initState.position = Vector(4,0,4) # Greater than 5m away
        adjustedForce = self.rail.applyLaunchTowerForce(self.initState, 0, unadjustedForce)

        # Check that forces/moments are unaffected
        assertVectorsAlmostEqual(self, adjustedForce.force, Vector(10,9,11))
        assertVectorsAlmostEqual(self, adjustedForce.moment, Vector(10,11,12))

    def test_normalizeDirection(self):
        self.assertAlmostEqual(self.rail.initialDirection.length(), 1.0)
        
    def test_init(self):
        # Modify file to include a launch rail
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/NASATwoStageOrbitalRocket.mapleaf")
        simDef.setValue("Environment.LaunchSite.railLength", "15")

        # Initialize environment with launch rail
        env = Environment(simDef)

        # Check that launch rail has been created
        self.assertTrue(env.launchRail != None)

        # Check initial launch rail position and direction
        r = 6378137 + 13.89622
        initPos = Vector(r, 0, 0)
        assertVectorsAlmostEqual(self, initPos, env.launchRail.initialPosition)
        
        initDir = Vector(math.cos(math.radians(34.78)), math.sin(math.radians(34.78)), 0).normalize()
        assertVectorsAlmostEqual(self, initDir, env.launchRail.initialDirection)

        self.assertAlmostEqual(env.launchRail.length, 15)

    def test_initPosition(self):
        # Modify file to include a launch rail
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/AdaptTimeStep.mapleaf")
        simDef.setValue("Environment.LaunchSite.railLength", "10")

        # Initialize environment with launch rail
        env = Environment(simDef)

        # Check that launch rail has been created
        self.assertTrue(env.launchRail != None)

        # Check initial launch rail position and direction
        initPos = Vector(0, 0, 15+755) # Rocket position + launch site elevation
        assertVectorsAlmostEqual(self, initPos, env.launchRail.initialPosition)

if __name__ == "__main__":
    unittest.main()
