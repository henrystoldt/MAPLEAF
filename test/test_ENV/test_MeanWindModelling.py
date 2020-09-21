#Created by: Henry Stoldt
#Jan 2020

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest
from test.testUtilities import assertVectorsAlmostEqual

import numpy as np

from MAPLEAF.ENV import ConstantWind, InterpolatedWind, meanWindModelFactory
from MAPLEAF.ENV.MeanWindModelling import (
    RadioSondeDataSampler, _convertWindHeadingToXYPlaneWindDirection)
from MAPLEAF.IO import SimDefinition, SubDictReader
from MAPLEAF.Motion import Vector


class TestMeanWindModels(unittest.TestCase):
    def setUp(self):
        self.rocket = "fakeNewsRocket" # May have to convert this into a real Rocket at some point, but it's faster this way
        self.simDefinition = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        self.envReader = SubDictReader("Environment", self.simDefinition)

    def test_convertWindHeadingToXYPlaneWindDirection(self):
        assertVectorsAlmostEqual(self, _convertWindHeadingToXYPlaneWindDirection(0), Vector(0,-1,0))
        assertVectorsAlmostEqual(self, _convertWindHeadingToXYPlaneWindDirection(90), Vector(-1,0,0))
        assertVectorsAlmostEqual(self, _convertWindHeadingToXYPlaneWindDirection(180), Vector(0,1,0))
        assertVectorsAlmostEqual(self, _convertWindHeadingToXYPlaneWindDirection(270), Vector(1,0,0))
        assertVectorsAlmostEqual(self, _convertWindHeadingToXYPlaneWindDirection(360), Vector(0,-1,0))

    def test_readCustomWindProfile(self):
        windModel = InterpolatedWind(windFilePath="MAPLEAF/Examples/Wind/testWindProfile.txt")
        
        self.assertTrue(np.allclose(windModel.windAltitudes, np.array([ 2000, 6000, 7000 ])))
        self.assertTrue(np.allclose(windModel.winds, np.array([ [ 10, 0, 0 ],
            [ 25, 0, 0 ],
            [ -5, -4, -2 ] ])))

    def test_getMeanWind_CustomProfile(self):
        # Make sure custom profile model is used
        self.simDefinition.setValue("Environment.MeanWindModel", "CustomWindProfile")
        self.simDefinition.setValue("Environment.CustomWindProfile.filePath", "MAPLEAF/Examples/Wind/testWindProfile.txt")
        # Re-initialize mWM
        mWM = meanWindModelFactory(self.envReader, self.rocket)

        assertVectorsAlmostEqual(self, mWM.getMeanWind(8000), Vector(-5, -4, -2))
        assertVectorsAlmostEqual(self, mWM.getMeanWind(7000), Vector(-5, -4, -2))
        assertVectorsAlmostEqual(self, mWM.getMeanWind(6500), Vector(10, -2, -1))
        assertVectorsAlmostEqual(self, mWM.getMeanWind(1000), Vector(10, 0, 0))
        assertVectorsAlmostEqual(self, mWM.getMeanWind(0), Vector(10, 0, 0))

    def test_getMeanWind_Hellman(self):
        # Make sure custom profile model is used
        self.simDefinition.setValue("Environment.MeanWindModel", "Hellman")
        self.simDefinition.setValue("Environment.Hellman.alphaCoeff", "0.1429")
        self.simDefinition.setValue("Environment.Hellman.altitudeLimit", "1000")
        # Set ground velocity
        self.simDefinition.setValue("Environment.Hellman.groundWindModel", "Constant")
        self.simDefinition.setValue("Environment.ConstantMeanWind.velocity", "(5 1 0)")
        # Re-initialize mWM
        mWM = meanWindModelFactory(self.envReader, self.rocket)

        assertVectorsAlmostEqual(self, mWM.getMeanWind(8000), Vector(9.655394088, 1.931078818, 0))
        assertVectorsAlmostEqual(self, mWM.getMeanWind(2000), Vector(9.655394088, 1.931078818, 0))
        assertVectorsAlmostEqual(self, mWM.getMeanWind(1000), Vector(9.655394088, 1.931078818, 0))
        assertVectorsAlmostEqual(self, mWM.getMeanWind(500), Vector(8.744859132, 1.748971826, 0))
        assertVectorsAlmostEqual(self, mWM.getMeanWind(0), Vector(5, 1, 0))

    def test_getMeanWind_Constant(self):
        windModel = ConstantWind(Vector(1,2,3))
        for altitude in [ 0, 1000, 10000, 100000 ]:
            self.assertEqual(windModel.getMeanWind(altitude), Vector(1,2,3))

    def test_initSampledGroundWindDataModel(self):
        self.simDefinition.setValue("Environment.MeanWindModel", "SampledGroundWindData")
        mWM = meanWindModelFactory(self.envReader, silent=True)

    def test_initSampledRadioSondeModel(self):
        self.simDefinition.setValue("Environment.MeanWindModel", "SampledRadioSondeData")
        mWM = meanWindModelFactory(self.envReader, silent=True)

class TestRadioSondeDataReader(unittest.TestCase):
    def setUp(self):
        self.rocket = "fakeNewsRocket" # May have to convert this into a real Rocket at some point, but it's faster this way
        self.simDefinition = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf", silent=True)
        self.simDefinition.setValue("Environment.MeanWindModel", "Constant")
        envReader = SubDictReader("Environment", self.simDefinition)
        self.mWM = meanWindModelFactory(envReader, self.rocket)

    def test_readRadioSondeDataFile(self):
        reader = RadioSondeDataSampler()
        datasetStartLines, data = reader._readRadioSondeDataFile("MAPLEAF/Examples/Wind/RadioSondetestLocation_filtered.txt")
        self.assertEqual(datasetStartLines, [0])
        self.assertEqual(len(datasetStartLines), 1)
        self.assertEqual(data[1], " 1361   235    82\n")
        self.assertEqual(data[15], "32280   300   525")
        self.assertEqual(len(data), 16)

    def test_radioSondeMonthFiltering(self):
        filePath = "MAPLEAF/Examples/Wind/RadioSondetestLocation_filtered.txt"
        reader = RadioSondeDataSampler()
        datasetStartLines, data = reader._readRadioSondeDataFile(filePath, filterByMonth="Jan")
        self.assertEqual(len(datasetStartLines), 1)
        datasetStartLines, data = reader._readRadioSondeDataFile(filePath, filterByMonth="Feb")
        self.assertEqual(len(datasetStartLines), 0)

    def test_extractRadioSondeData(self):
        reader = RadioSondeDataSampler()
        datasetStartLines, data = reader._readRadioSondeDataFile("MAPLEAF/Examples/Wind/RadioSondetestLocation_filtered.txt")
        header, data2 = reader._extractRadioSondeDataSet(datasetStartLines, data, 0)

        self.assertEqual(header, "#CAM00071119 2000 01 01 00 2315   83 ncdc-gts ncdc-gts  535475 -1141083\n")
        self.assertEqual(data2[0], " 1361   235    82\n" )
        self.assertEqual(data2[14], "32280   300   525")

    def test_parseRadioSondeHeader(self):
        header = "#CAM00071119 2000 01 01 00 2315   83 ncdc-gts ncdc-gts  535475 -1141083\n"
        reader = RadioSondeDataSampler()
        stationID, year, month, day, hour = reader._parseRadioSondeHeader(header)
        self.assertEqual(stationID, "CAM00071119")
        self.assertEqual(year, 2000)
        self.assertEqual(month, 1)
        self.assertEqual(day, 1)
        self.assertEqual(hour, 00)

    def test_parseRadioSondeData(self):
        reader = RadioSondeDataSampler()

        datasetStartLines, data = reader._readRadioSondeDataFile("MAPLEAF/Examples/Wind/RadioSondetestLocation_filtered.txt")
        header, data2 = reader._extractRadioSondeDataSet(datasetStartLines, data, 0)
        altitudes, windVectors = reader._parseRadioSondeData(data2, 0)

        self.assertEqual(altitudes[0], 1361)
        self.assertEqual(altitudes[-1], 32280)
        assertVectorsAlmostEqual(self, Vector(*windVectors[0]), Vector(0.819152044, 0.573576436, 0)*8.2)
        assertVectorsAlmostEqual(self, Vector(*windVectors[-1]), Vector(0.866025403, -0.5, 0)*52.5)

        # Test ASL location adjustment
        altitudes, windVectors = reader._parseRadioSondeData(data2, 100)

        self.assertAlmostEqual(altitudes[0], 1261)
        self.assertAlmostEqual(altitudes[-1], 32180)
        assertVectorsAlmostEqual(self, Vector(*windVectors[0]), Vector(0.819152044, 0.573576436, 0)*8.2)
        assertVectorsAlmostEqual(self, Vector(*windVectors[-1]), Vector(0.866025403, -0.5, 0)*52.5)

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
