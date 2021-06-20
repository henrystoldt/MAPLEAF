import os
import re
import unittest
from copy import deepcopy
from test.testUtilities import assertVectorsAlmostEqual

from MAPLEAF.IO import SimDefinition, defaultConfigValues
from MAPLEAF.IO.simDefinition import (getAbsoluteFilePath, getImmediateSubKey,
                                      getKeyLevel, isSubKey)
from MAPLEAF.Motion import Vector


class TestSimDefinition(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fileName = "test/test_IO/textFileDefinition.mapleaf"
        cls.simDef = SimDefinition(cls.fileName, silent=True)
        cls._simDef = deepcopy(cls.simDef)

    @classmethod
    def resetSimDef(cls):
        cls.simDef = deepcopy(cls._simDef)

    def test_detectDuplicateValue(self):
        with self.assertRaises(ValueError):
            duplicateKeyWrangler = SimDefinition("test/test_IO/TestSimDefinition_duplicateKeyError.mapleaf", silent=True)

    def test_UpdateValue(self):
        self.setValueTest("Dictionary1.SubDictionary1.key3", "newValue")
        self.setValueTest("Dictionary1.key1", "1.2245")
        self.setValueTest("Dictionary2.subD2.subsubD3.keyZZ", "value5")
        self.resetSimDef()

    # Test called by the actual test_ function
    def setValueTest(self, key, newVal):
        self.simDef.setValue(key, newVal)
        self.assertEqual(self.simDef.getValue(key), newVal)
    
    # Also testing the constructor here
    def test_getAndConstructor(self):
        self.assertEqual(self.simDef.getValue("Dictionary1.key1"), "value1")
        self.assertEqual(self.simDef.getValue("Dictionary1.key2"), "value2")
        self.assertEqual(self.simDef.getValue("Dictionary1.SubDictionary1.key3"), "value3")
        self.assertEqual(self.simDef.getValue("Dictionary1.SubDictionary1.key4"), "value4")
        
        simConfig = SimDefinition("test/test_IO/testSimDefinition.mapleaf", silent=True)
        self.assertEqual(simConfig.getValue("SimControl.timeDiscretization"), "RK4")
        self.assertEqual(simConfig.getValue("SimControl.timeStep"), "0.01")
        self.assertEqual(simConfig.getValue("SimControl.plot"), "Position Velocity AngularVelocity FlightAnimation")
        self.assertEqual(simConfig.getValue("Environment.initAltitude"), "0")
        self.assertEqual(simConfig.getValue("Environment.windSpeed"), "( 0 0 0 )")

    def test_writeToFile(self):
        #### Check rocket-only autowriting ####
        self.simDef.writeToFile("test/test_IO/testWrite.mapleaf")

        # Get output
        file1 = open("test/test_IO/testWrite.mapleaf", 'r')
        f1 = file1.read()
        f1 = re.sub("# Autowritten on:.+?\\n", "", f1)
        file1.close()

        # Get Correct output
        file2 = open("test/test_IO/testWriteCorrectOutput.mapleaf", 'r')
        f2 = file2.read()
        f2 = re.sub("# Autowritten on:.+?\\n", "", f2)
        file2.close()

        # Compare
        self.assertEqual(f1, f2)

        #### Check simConfig autowriting ####
        simConfig = SimDefinition("test/test_IO/testSimDefinition.mapleaf", silent=True)
        testWritePath = "test/test_IO/testSimConfigWrite.mapleaf"
        simConfig.writeToFile(testWritePath)
        simConfig2 = SimDefinition(testWritePath, silent=True)
        self.assertEqual(simConfig, simConfig2)

    def test_setIfAbsent(self):
        self.simDef.setIfAbsent("Dictionary1.key1", "newValue")
        self.assertEqual(self.simDef.getValue("Dictionary1.key1"), "value1")
        self.simDef.setIfAbsent("Dictionary1.key3", "newValue")
        self.assertEqual(self.simDef.getValue("Dictionary1.key3"), "newValue")

        self.resetSimDef()

    def test_findKeysContaining(self):
        # List of keys
        res = self.simDef.findKeysContaining(["SubDictionary1"])
        self.assertEqual(res, ["Dictionary1.SubDictionary1.key3", "Dictionary1.SubDictionary1.key4", "Dictionary1.SubDictionary1.key5"])

        # Single string key
        res = self.simDef.findKeysContaining("SubDictionary1")
        self.assertEqual(res, ["Dictionary1.SubDictionary1.key3", "Dictionary1.SubDictionary1.key4", "Dictionary1.SubDictionary1.key5"])

    def test_removeKey(self):
        val1 = self.simDef.removeKey("Dictionary1.key1")
        self.assertEqual(val1, "value1")
        self.assertFalse("Dictionary1.key1" in self.simDef.dict)

        self.resetSimDef()

    def test_defaultValues(self):
        # Test that values not in the config file are returned from the defaults dictionary
        self.assertEqual(self.simDef.getValue("testValue.testDefaultValue1"), "asdf")
        self.assertEqual(self.simDef.getValue("testDefaultValue2"), "jkl;")
        self.assertEqual(self.simDef.getValue("Environment.TurbulenceModel"), "None")

        # Test that values not in the config file or in the default dict are not found
        with self.assertRaises(KeyError):
            self.simDef.getValue("NonexistentKeyasdf;lkjasdf;lkjasdfl;k")

    def test_classBasedDefaultValues(self):
        simConfig = SimDefinition("test/test_IO/testSimDefinition.mapleaf", silent=True)
        self.assertEqual(simConfig.getValue("Rocket.Sustainer.TailFins.Actuators.controller"), "TableInterpolating")
        self.assertEqual(simConfig.getValue("Rocket.Sustainer.TailFins.finCantAngle"), "0")

    def test_sampleDistribution_scalarValue(self):
        self.simDef.setValue("scalar1", "10")
        self.simDef.setValue("scalar1_stdDev", "1")
        self.simDef.rng.seed(1000)
        self.simDef.resampleProbabilisticValues()

        self.assertAlmostEqual(10.254632116649685, float(self.simDef.getValue("scalar1")))
        # Check does not change without resampling
        self.assertAlmostEqual(10.254632116649685, float(self.simDef.getValue("scalar1")))

        # Check different after resample
        self.simDef.resampleProbabilisticValues()
        self.assertNotAlmostEqual(10.254632116649685, float(self.simDef.getValue("scalar1")))

        self.resetSimDef()

    def test_sampleDistribution_vectorValue(self):
        self.simDef.setValue("vector1", "(10, 11, 12)")
        self.simDef.setValue("vector1_stdDev", "(1, 2, 3)")
        self.simDef.rng.seed(1000)
        self.simDef.resampleProbabilisticValues()        

        resultVec = Vector(self.simDef.getValue("vector1"))
        expectedVec = Vector(10.254632116649685, 8.066448705920658 ,14.27360999981685)
        assertVectorsAlmostEqual(self, expectedVec, resultVec)
        # Check does not change without resampling
        resultVec = Vector(self.simDef.getValue("vector1"))        
        assertVectorsAlmostEqual(self, expectedVec, resultVec)

        # Check different value after resampling
        self.simDef.resampleProbabilisticValues()        
        resultVec = Vector(self.simDef.getValue("vector1"))        
        for i in range(3):
            self.assertNotAlmostEqual(expectedVec[i], resultVec[i])

        self.resetSimDef()

    def test_isSubKey(self):
        self.assertTrue(isSubKey("Rocket", "Rocket.name"))
        self.assertFalse(isSubKey("Rocket", "SimControl"))
        self.assertTrue(isSubKey("", "Rocket"))
        self.assertTrue(isSubKey("Dictionary1", "Dictionary1.key1"))
        self.assertFalse(isSubKey("Case5", "Case5_0AOA"))

    def test_getKeyLevel(self):
        self.assertEqual(getKeyLevel("Rocket.name"), 1)
        self.assertEqual(getKeyLevel("Rocket"), 0)
        self.assertEqual(getKeyLevel("Rocket.name.asdfasdf.asdfasdf.asdfwerqt.4"), 5)

    def test_getImmediateSubkey(self):
        self.assertEqual(getImmediateSubKey("Rocket", "Rocket.Sustainer.name"), "Rocket.Sustainer")
        self.assertEqual(getImmediateSubKey("", "SimControl.timeStep"), "SimControl")

    def test_getImmediateSubkeys(self):
        result = self.simDef.getImmediateSubKeys("Dictionary1")
        expectedResult = [ "Dictionary1.key2", "Dictionary1.key1"  ]
        self.assertEqual(len(result), len(expectedResult))
        for i in range(len(result)):
            self.assertTrue(result[i] in expectedResult)

    def test_getSubKeys(self):
        result = self.simDef.getSubKeys("Dictionary1")
        expectedContents = [ "Dictionary1.SubDictionary1.key3", "Dictionary1.key2", "Dictionary1.key1", "Dictionary1.SubDictionary1.key4", "Dictionary1.SubDictionary1.key5" ]
        for key in expectedContents:
            self.assertTrue(key in result)

    def test_getImmediateSubDicts(self):
        result = self.simDef.getImmediateSubDicts("Dictionary1")
        expectedResult = [ "Dictionary1.SubDictionary1" ]
        self.assertEqual(result, expectedResult)

    def test_checkForOutOfDateDefaultValues(self):
        AllPossibleOptions = SimDefinition("SimDefinitionTemplate.mapleaf")

        for key in defaultConfigValues:
            # Can only perform this check for keys that aren't class-based (start with one of the root dictionary names)
            if "." not in key or key[:key.index('.')] not in [ "Rocket", "Environment", "SimControl", "MonteCarlo" ]:
                continue

            if not key in AllPossibleOptions.dict:
                raise ValueError("Key {} in default values, not in Sim Definition Template. Update which ever one is out of date".format(key))

    def test_ParsingDerivedDict(self):
        simDef = SimDefinition("test/test_IO/testDerivedDicts.mapleaf")
        simDef2 = SimDefinition("test/test_IO/testDerivedDictsFinal.mapleaf")
        self.assertEqual(simDef, simDef2)

    def test_contains(self):
        self.assertTrue("Dictionary1.key1" in self.simDef)
        self.assertTrue("Dictionary1.key2" in self.simDef)
        self.assertTrue("Dictionary1.key3" not in self.simDef)
        self.assertTrue("Dictionary1.SubDictionary1.key3" in self.simDef)
        self.assertTrue("Dictionary1.SubDictionary1.key6" not in self.simDef)

    def test_getAbsoluteFilePath(self):
        filePath = getAbsoluteFilePath("testDerivedDicts.mapleaf", "./test/test_IO/")
        self.assertTrue(os.path.isfile(filePath))

    def test_circularImport(self):
        with self.assertRaises(ValueError):
            circularSimDef = SimDefinition("test/test_IO/circImportTest1.mapleaf")

    def test_relativeFilepath(self):
        simulationDefinition = SimDefinition('test/test_IO/testRelativeMotorPath.mapleaf', silent=True)        
        motorFilePath = simulationDefinition.getValue('Rocket.Sustainer.Motor.path')
        self.assertTrue(os.path.isfile(motorFilePath))

        # Ensure relative path expansion does not corrupt plot line formats
        self.assertEqual(simulationDefinition.getValue('Rocket.Sustainer.Motor.lineFormat'), '.')

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
