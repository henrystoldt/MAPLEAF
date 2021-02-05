import unittest

import matplotlib.pyplot as plt

from MAPLEAF.IO import SimDefinition
from MAPLEAF.Main import isBatchSim
from MAPLEAF.SimulationRunners.Batch import _checkResult, BatchRun, _validate, CaseResult
from test.testUtilities import captureOutput


class TestBatchSim(unittest.TestCase):
    
    def setUp(self):
        self.batchRun = BatchRun("fakeSimDef")

    def test_isBatchSim(self):
        # Checks whether the batch sim detection function is working
        batchDefinition = SimDefinition("batchRunTemplate.mapleaf")
        self.assertTrue(isBatchSim(batchDefinition))

        normalDefinition = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf")
        self.assertFalse(isBatchSim(normalDefinition))

    def test_checkResult(self):
        caseResult = CaseResult('fakeCase', 0, 0, 0, [], [])
        origPassedTests = caseResult.testsPassed
        origFailedTests = caseResult.testsFailed

        # Should pass
        with captureOutput() as (out, err):
            _checkResult(self.batchRun, caseResult, "fakeCase", "FakeColumn", 1.125, 1.125)

        self.assertEqual(caseResult.testsPassed, origPassedTests+1)
        output = out.getvalue().strip()
        self.assertTrue(" ok " in output)
        self.assertTrue(" FAIL " not in output)

        
        # Should fail
        with captureOutput() as (out, err):
            _checkResult(self.batchRun, caseResult, "fakeCase", "FakeColumn", 1.1, 1.0)

        self.assertEqual(caseResult.testsFailed, origFailedTests+1)
        output = out.getvalue().strip()
        self.assertTrue(" ok " not in output)
        self.assertTrue(" FAIL " in output)

    def test_validate_1To1(self):
        x = [1,2]
        yMapleaf= [1.05, 2.3]
        yValidation = [1, 2]

        _validate(self.batchRun, [x], [yMapleaf], [yValidation], x, "Test.Case.Data")

        self.assertAlmostEqual(self.batchRun.validationErrors[0], 10)
        self.assertEqual(len(self.batchRun.validationErrors), 1)
        self.assertEqual(self.batchRun.validationDataUsed, ["Test.Case.Data"])

    def test_validate_2To1(self):
        x = [1,2]
        yMapleaf= [1.05, 2.3]
        yValidation = [1, 2]
        yValidation2 = [ 1.01, 2.02]

        _validate(self.batchRun, [x], [yMapleaf], [yValidation, yValidation2], x, "Test.Case.Data")

        # Expect to return average error across all comparison data sets
        self.assertAlmostEqual(self.batchRun.validationErrors[0], 9.4554455445545)
        self.assertEqual(len(self.batchRun.validationErrors), 1)
        self.assertEqual(self.batchRun.validationDataUsed, ["Test.Case.Data"])

    def test_validate_1To2(self):
        x = [1,2]
        yMapleaf= [1.05, 2.3]
        yMapleaf2 = [1, 2]
        yValidation2 = [ 1, 2]

        _validate(self.batchRun, [x, x], [yMapleaf, yMapleaf2], [yValidation2], x, "Test.Case.Data")

        # Expect to return average error across all comparison data sets
        self.assertAlmostEqual(self.batchRun.validationErrors[0], 0.0)
        self.assertEqual(len(self.batchRun.validationErrors), 1)
        self.assertEqual(self.batchRun.validationDataUsed, ["Test.Case.Data"])

    def test_getCasesToRun(self):
        regressionTestsDefinition = SimDefinition("MAPLEAF/Examples/BatchSims/regressionTests.mapleaf")
        batchRun = BatchRun(regressionTestsDefinition, include="ParametricFin", exclude="0AOA")
        casesToRun = sorted(batchRun.getCasesToRun())

        expectedCasesToRun = [ "ParametricFin1", "ParametricFin10", "ParametricFin12", "ParametricFin5", "ParametricFin9", ]
        self.assertEqual(casesToRun, expectedCasesToRun)