#Created by: Henry Stoldt
#April 2020

#To run tests:
#In this file: [test_TurbulenceModelling.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest
from test.testUtilities import assertVectorsAlmostEqual

from MAPLEAF.IO import SimDefinition
from MAPLEAF.IO import SubDictReader
from MAPLEAF.ENV import meanWindModelFactory
from MAPLEAF.Rocket import Rocket
from MAPLEAF.ENV import PinkNoiseGenerator, turbulenceModelFactory
from MAPLEAF.Motion import Vector


class TestTurbulenceModels(unittest.TestCase):
    def setUp(self):
        configFilePath = "MAPLEAF/Examples/Simulations/Wind.mapleaf"
        self.simDef = SimDefinition(configFilePath, silent=True)
        self.simDef.setValue("Environment.MeanWindModel", "Constant")
        self.simDef.setValue("SimControl.loggingLevel", "0")
        rocketDictReader = SubDictReader("Rocket", self.simDef)
        envReader = SubDictReader("Environment", self.simDef)
        self.rocket = Rocket(rocketDictReader, silent=True)
        
        self.mWM = meanWindModelFactory(envReader, self.rocket)

    def test_initPinkNoiseGenerator(self):
        png = PinkNoiseGenerator(alpha=5/3, nPoles=2)
        self.assertAlmostEqual(png.coeffs[0], -5/6)
        self.assertAlmostEqual(png.coeffs[1], -5/72)

    def test_pinkNoiseLinearInterpolation(self):
        png = PinkNoiseGenerator(alpha=5/3, nPoles=2, seed=63583, simulatedSamplingFreqHz=20)
        # Values 1 and 2 will be 0.7946604314895807 and 0.5874110050866364 at 0.05 and 0.1 seconds, respectively
        self.assertAlmostEqual(png.getValue(0.05), 0.7946604314895807)
        self.assertAlmostEqual(png.getValue(0.075), 0.691035718)
        self.assertAlmostEqual(png.getValue(0.09), 0.62886089)
        self.assertAlmostEqual(png.getValue(0.1), 0.5874110050866364)

    def test_pinkNoiseStdDev(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        simDef.setValue("Environment.TurbulenceModel", "PinkNoise1D")
        simDef.setValue("Environment.PinkNoiseModel.velocityStdDeviation", "1") 
        simDef.setValue("Environment.PinkNoiseModel.randomSeed1", "63583")       
        simDef.setValue("Environment.turbulenceOffWhenUnderChute", "False")   

        # Make sure turbulence intensity is not present since it overrides the given std deviation
        simDef.removeKey("Environment.PinkNoiseModel.turbulenceIntensity")
        envReader = SubDictReader("Environment", simDef)

        turbModel = turbulenceModelFactory(envReader)
        v1 = turbModel.getTurbVelocity(1, Vector(1,0,0), None)
        self.assertAlmostEqual(v1[0], 0.7946604314895807/2.26)
        v2 = turbModel.getTurbVelocity(1, Vector(1,0,0), None)
        self.assertAlmostEqual(v2[0], 0.5874110050866364/2.26)

    def test_pinkNoiseTurbIntensity(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        simDef.setValue("Environment.TurbulenceModel", "PinkNoise1D")
        simDef.setValue("Environment.PinkNoiseModel.turbulenceIntensity", "100") 
        simDef.setValue("Environment.PinkNoiseModel.randomSeed1", "63583")      
        simDef.setValue("Environment.turbulenceOffWhenUnderChute", "False")

        # Make sure std deviation is not present since it overrides the given turbulence intensity
        simDef.removeKey("Environment.PinkNoiseModel.velocityStdDeviation")
        envReader = SubDictReader("Environment", simDef)

        turbModel = turbulenceModelFactory(envReader)
        v1 = turbModel.getTurbVelocity(1, Vector(1,0,0), None)
        self.assertAlmostEqual(v1[0], 0.7946604314895807/2.26)
        v2 = turbModel.getTurbVelocity(1, Vector(1,0,0), None)
        self.assertAlmostEqual(v2[0], 0.5874110050866364/2.26)

    def test_customGust(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        simDef.setValue("Environment.TurbulenceModel", "customSineGust")
        simDef.setValue("Environment.CustomSineGust.startAltitude", "0")
        simDef.setValue("Environment.CustomSineGust.magnitude", "9")
        simDef.setValue("Environment.CustomSineGust.sineBlendDistance", "30")
        simDef.setValue("Environment.CustomSineGust.thickness", "200")
        simDef.setValue("Environment.CustomSineGust.direction", "(0 1 0)")
        envReader = SubDictReader("Environment", simDef)

        turbModel = turbulenceModelFactory(envReader)

        v1 = turbModel.getTurbVelocity(0, Vector(1,0,0), 0)
        assertVectorsAlmostEqual(self, v1, Vector(0,0,0))
        v1 = turbModel.getTurbVelocity(15, Vector(1,0,0), 0)
        assertVectorsAlmostEqual(self, v1, Vector(0,4.5,0))
        v1 = turbModel.getTurbVelocity(30, Vector(1,0,0), 0)
        assertVectorsAlmostEqual(self, v1, Vector(0,9,0))
        v1 = turbModel.getTurbVelocity(230, Vector(1,0,0), 0)
        assertVectorsAlmostEqual(self, v1, Vector(0,9,0))
        v1 = turbModel.getTurbVelocity(245, Vector(1,0,0), 0)
        assertVectorsAlmostEqual(self, v1, Vector(0,4.5,0))
        v1 = turbModel.getTurbVelocity(260, Vector(1,0,0), 0)
        assertVectorsAlmostEqual(self, v1, Vector(0,0,0))

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
