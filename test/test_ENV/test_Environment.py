#Created by: Henry Stoldt
#May 2019

import unittest
from test.testUtilities import assertIterablesAlmostEqual

from MAPLEAF.ENV import Environment, EnvironmentalConditions
from MAPLEAF.IO import SimDefinition
from MAPLEAF.Motion import Vector


class TestEnvironment(unittest.TestCase):
    def setUp(self):
        self.rocket = "fakeNewsRocket" # May have to convert this into a real Rocket at some point, but it's faster this way
        self.simDefinition = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        self.simDefinition.setValue("Environment.MeanWindModel", "Constant")
        self.simDefinition.setValue("Environment.ConstantMeanWind.velocity", "(0 0 0)")

    def test_constAtmosphere(self):
        # Set constant properties in SimDefinition
        self.simDefinition.setValue("Environment.AtmosphericPropertiesModel", "Constant")
        self.simDefinition.setValue("Environment.ConstantAtmosphere.temp", "20") # This should be converted to K -> 293.15 expected
        self.simDefinition.setValue("Environment.ConstantAtmosphere.pressure", "101325")
        self.simDefinition.setValue("Environment.ConstantAtmosphere.density", "1.225")
        self.simDefinition.setValue("Environment.ConstantAtmosphere.viscosity", "1.789e-05")

        env = Environment(self.simDefinition, silent=True)
        
        constAirProps1 = EnvironmentalConditions(100.0, 293.15, 101325, 1.225, 1.789e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        props1 = env.getAirProperties(Vector(0,0,100))
        
        constAirProps2 = EnvironmentalConditions(1000.0, 293.15, 101325, 1.225, 1.789e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        props2 = env.getAirProperties(Vector(0,0,1000))
        
        constAirProps3 = EnvironmentalConditions(10000.0, 293.15, 101325, 1.225, 1.789e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        props3 = env.getAirProperties(Vector(0,0,10000))

        self.assertEqual(constAirProps1, props1)
        self.assertEqual(constAirProps2, props2)
        self.assertEqual(constAirProps3, props3)

    def test_getAirProperties_TabulatedAtm(self):
        self.simDefinition.setValue("Environment.LaunchSite.elevation", "0")
        self.simDefinition.setValue("Environment.AtmosphericPropertiesModel", "TabulatedAtmosphere")
        self.simDefinition.setValue("Environment.TabulatedAtmosphere.filePath", "MAPLEAF/ENV/US_STANDARD_ATMOSPHERE.txt")
        stdAtm = Environment(self.simDefinition, silent=True)
        
        # Sea level
        calculatedSeaLevelProperties = stdAtm.getAirProperties(Vector(0,0,0))        
        expectedSeaLevelProperties = EnvironmentalConditions(0.0,  288.15, 101300, 1.225, 1.789e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        assertIterablesAlmostEqual(self, calculatedSeaLevelProperties, expectedSeaLevelProperties)

        # 1000 m ASL
        calculated1kProperties = stdAtm.getAirProperties(Vector(0,0,1000))
        expected1kProperties = EnvironmentalConditions( 1000.0, 8.5+273.15, 89880, 1.112, 1.758e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        assertIterablesAlmostEqual(self, calculated1kProperties, expected1kProperties)

        # 3750 m ASL
        calculated3750Properties = stdAtm.getAirProperties(Vector(0,0,3750))
        expected3570Properties = EnvironmentalConditions( 3750.0, -9.3575+273.15, 63775, 0.841875, 1.66925e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        assertIterablesAlmostEqual(self, calculated3750Properties, expected3570Properties)

    def test_getAirProperties_USSTDA(self):
        self.simDefinition.setValue("Environment.LaunchSite.elevation", "0")
        self.simDefinition.setValue("Environment.AtmosphericPropertiesModel", "USStandardAtmosphere")
        stdAtm = Environment(self.simDefinition, silent=True)

        # Sea level
        calculatedSeaLevelProperties = stdAtm.getAirProperties(Vector(0,0,0))        
        expectedSeaLevelProperties = EnvironmentalConditions(0.0,  288.15, 101325, 1.225, 1.789e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        assertIterablesAlmostEqual(self, calculatedSeaLevelProperties, expectedSeaLevelProperties, 5)

        # 1000 m ASL
        calculated1kProperties = stdAtm.getAirProperties(Vector(0,0,1000))
        expected1kProperties = EnvironmentalConditions( 1000.0, 8.501+273.15, 89876, 1.117, 1.758e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        assertIterablesAlmostEqual(self, calculated1kProperties, expected1kProperties, 0)

        # 3750 m ASL
        calculated3750Properties = stdAtm.getAirProperties(Vector(0,0,3750))
        expected3570Properties = EnvironmentalConditions( 3750.0, -9.361+273.15, 63693, 0.84115, 1.66925e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        assertIterablesAlmostEqual(self, calculated3750Properties, expected3570Properties, 0)

        # 10,550 m ASL
        calculatedProperties = stdAtm.getAirProperties(Vector(0,0,10550))
        expectedProperties = EnvironmentalConditions( 10550, 219.575, 24284, 0.38529, 1.457e-6, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        # assertIterablesAlmostEqual(self, calculatedProperties, expectedProperties, 0)

        # 85,500 m ASL
        calculatedProperties = stdAtm.getAirProperties(Vector(0,0,85500))
        expectedProperties = EnvironmentalConditions( 85500, 187.920, 0.40802, 0.0000075641, 1.25816e-5, Vector(0,0,0), Vector(0,0,0), Vector(0,0,0))
        assertIterablesAlmostEqual(self, calculatedProperties, expectedProperties, 1)

        # Check that above 86 km the density is zero
        calculatedProperties = stdAtm.getAirProperties(Vector(0,0,85500))

    def test_LayerBaseProperties(self):
        self.simDefinition.setValue("Environment.LaunchSite.elevation", "0")
        self.simDefinition.setValue("Environment.AtmosphericPropertiesModel", "USStandardAtmosphere")
        stdAtm = Environment(self.simDefinition, silent=True)

        baseHeights = [ -2000, 11000, 20000, 32000, 47000, 51000, 71000, 84852 ] # m
        dt_dh = [ -6.5e-3, 0, 1.0e-3, 2.8e-3, 0, -2.8e-3, -2.0e-3, 0 ] # K/m

        expectedTemps = [ 301.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.95 ]
        expectedPressures = [ 127774, 22632, 5474.8, 868.01, 110.90, 66.938, 3.9564, 0.3732]

        for i in range(len(expectedTemps)):
            self.assertAlmostEqual(expectedTemps[i], stdAtm.atmosphericModel.baseTemps[i],0)
            self.assertAlmostEqual(expectedPressures[i], stdAtm.atmosphericModel.basePressures[i], 0)


#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
