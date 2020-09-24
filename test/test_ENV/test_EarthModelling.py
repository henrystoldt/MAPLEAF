#Created by: Henry Stoldt
# August 2020

import math
import unittest

from MAPLEAF.SimulationRunners import Simulation
from MAPLEAF.ENV import (FlatEarth, NoEarth, SphericalEarth,
                                    WGS84)
from MAPLEAF.Motion import Vector
from test.testUtilities import assertVectorsAlmostEqual
from MAPLEAF.IO import SimDefinition
from MAPLEAF.SimulationRunners import Simulation
from test.testUtilities import assertVectorsAlmostEqual


class TestEarthModels(unittest.TestCase):

    def setUp(self):
        self.WGS84 = WGS84()
        self.roundEarth = SphericalEarth()
        self.noEarth = NoEarth()
        self.flatEarth = FlatEarth()

    def checkCoordinateTransforms(self, lat, lon, h, x, y, z, earthModel, n=6):
        ''' 
            Provide a correct pair of lat/lon/h and x/y/z coordinates. This function checks that conversions
            work in both directions using the WGS84 model
        '''
        x2, y2, z2 = earthModel.geodeticToCartesian(lat, lon, h)

        # Compare to results obtained from pyproj
        self.assertAlmostEqual(x2, x, n)
        self.assertAlmostEqual(y2, y, n)
        self.assertAlmostEqual(z2, z, n)

        # Check conversion back to latitude and longitude
        lat2, lon2, h2 = earthModel.cartesianToGeodetic(x, y, z)

        self.assertAlmostEqual(lat, lat2, n)
        self.assertAlmostEqual(lon, lon2, n)
        self.assertAlmostEqual(h, h2, n)

    def test_WGS84coordinateConversions(self):
        ''' 
            Check that we can convert to cartesian, and back, getting the same parameters we started with 
            Comparison x, y, z results obtained from PROJ, through pyproj
            
            pyproj is not used in simulations because it is very slow for single requests compared the method currently implemented
        '''
        # University of Calgary
        lat, lon, h = 51.075339, -114.131767, 0
        x, y, z = -1641688.5729170945, -3664588.431914631, 4938814.914854015
        self.checkCoordinateTransforms(lat, lon, h, x, y, z, self.WGS84)

        # Greenland
        lat, lon, h = 81.136427, -34.544050, 500
        x, y, z = 812209.7821296552, -559136.2029726238, 6280831.753924148
        self.checkCoordinateTransforms(lat, lon, h, x, y, z, self.WGS84)

        # Antarctica
        lat, lon, h = -86.618022, 38.395443, 1000
        x, y, z = 295927.0631906039, 234510.58046597126, -6346605.447459776
        self.checkCoordinateTransforms(lat, lon, h, x, y, z, self.WGS84)

        # Netherlands
        lat, lon, h = 51.624678, 5.261977, -500
        x, y, z = 3950752.939746138, 363856.04428183084, 4976593.902843423
        self.checkCoordinateTransforms(lat, lon, h, x, y, z, self.WGS84)

    def test_RoundEarthCoordinateConversions(self):
        lat, lon, h = 50.887093455, -114.131767, -5762.965219

        # Position radius = 6365244.216
        x, y, z = -1641690.425, -3664592.566, 4938820.487
        self.checkCoordinateTransforms(lat, lon, h, x, y, z, self.roundEarth, n=3)

    def test_rotationRate(self):
        secondsInOneDay = 60 * 60 * 24 - 235.9095 # Sidereal day - noy exactly right but close
        secondsToRotate = 2*math.pi / self.WGS84.rotationRate
        self.assertAlmostEqual(secondsToRotate, secondsInOneDay, 1)

        secondsToRotate = 2*math.pi / self.roundEarth.rotationRate
        self.assertAlmostEqual(secondsToRotate, secondsInOneDay, 1)
    
    def test_getGravity_noEarth(self):
        self.assertEqual(Vector(0,0,0), self.noEarth.getGravityForce("asdf", "adf"))

    def test_Initialization_NASASphere(self):
        simRunner = Simulation("./MAPLEAF/Examples/Simulations/NASASphere.mapleaf")
        sphere = simRunner.createRocket()

        # Should be at an altitude of 9144 m above earth's surface
        distanceFromEarthCenter = sphere.rigidBody.state.position.length()
        expected = sphere.environment.earthModel.a + 9144
        self.assertAlmostEqual(distanceFromEarthCenter, expected)

        # Should have zero angular velocity in inertial frame
        angVel = sphere.rigidBody.state.angularVelocity.length()
        self.assertAlmostEqual(angVel, 0.0)

        # Velocity should be zero in earth-fixed frame
        rotationVel = distanceFromEarthCenter * sphere.environment.earthModel.rotationRate
        inertialVel = sphere.rigidBody.state.velocity
        assertVectorsAlmostEqual(self, inertialVel, Vector(0, rotationVel, 0))

    def test_Initialization_Velocity(self):
        # Zero velocity in launch tower frame
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/NASATwoStageOrbitalRocket.mapleaf", silent=True)
        simDef.setValue("Rocket.velocity", "(0 0 0)")
        simRunner = Simulation(simDefinition=simDef, silent=True)
        rocket = simRunner.createRocket()
        
        computedInitGlobalFrameVel = rocket.rigidBody.state.velocity
        expectdedVel = Vector(0, 465.1020982258931, 0) # Earth's surface velocity at 0 lat, 0 long
        assertVectorsAlmostEqual(self, computedInitGlobalFrameVel, expectdedVel)

        # Velocity in the +x (East) direction in the launch tower frame
        simDef.setValue("Rocket.velocity", "(1 0 0)")
        simRunner = Simulation(simDefinition=simDef, silent=True)
        rocket = simRunner.createRocket()
        
        computedInitGlobalFrameVel = rocket.rigidBody.state.velocity
        expectdedVel = Vector(0, 466.1020982258931, 0) # Earth's surface velocity at 0 lat, 0 long + 1m/s east
        assertVectorsAlmostEqual(self, computedInitGlobalFrameVel, expectdedVel)

        # Velocity in the +y (North) direction in the launch tower frame
        simDef.setValue("Rocket.velocity", "(0 1 0)")
        simRunner = Simulation(simDefinition=simDef, silent=True)
        rocket = simRunner.createRocket()
        
        computedInitGlobalFrameVel = rocket.rigidBody.state.velocity
        expectdedVel = Vector(0, 465.1020982258931, 1) # Earth's surface velocity at 0 lat, 0 long + 1m/s north
        assertVectorsAlmostEqual(self, computedInitGlobalFrameVel, expectdedVel)

        # Velocity in the +z (Up) direction in the launch tower frame
        simDef.setValue("Rocket.velocity", "(0 0 1)")
        simRunner = Simulation(simDefinition=simDef, silent=True)
        rocket = simRunner.createRocket()
        
        computedInitGlobalFrameVel = rocket.rigidBody.state.velocity
        expectdedVel = Vector(1, 465.1020982258931, 0) # Earth's surface velocity at 0 lat, 0 long + 1m/s up
        assertVectorsAlmostEqual(self, computedInitGlobalFrameVel, expectdedVel)
        
#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
