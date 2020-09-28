import unittest

import matplotlib.pyplot as plt

from MAPLEAF.IO import SimDefinition
from MAPLEAF.Main import isBatchSim
from MAPLEAF.SimulationRunners.Batch import _checkResult, BatchRun
from test.testUtilities import captureOutput


class TestBatchSim(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        cls.batchRun = BatchRun("fakeSimDef")

    def test_isBatchSim(self):
        # Checks whether the batch sim detection function is working
        batchDefinition = SimDefinition("batchRunTemplate.mapleaf")
        self.assertTrue(isBatchSim(batchDefinition))

        normalDefinition = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf")
        self.assertFalse(isBatchSim(normalDefinition))

    def test_checkResult(self):
        origPassedTests = self.batchRun.nTestsOk
        origFailedTests = self.batchRun.nTestsFailed

        # Check case that should pass
        with captureOutput() as (out, err):
            _checkResult(self.batchRun, "FakeColumn", 1.125, 1.125)

        self.assertEqual(self.batchRun.nTestsOk, origPassedTests+1)
        output = out.getvalue().strip()
        self.assertTrue(" ok " in output)
        self.assertTrue(" FAIL " not in output)

        
        # Check case that should fail
        with captureOutput() as (out, err):
            _checkResult(self.batchRun, "FakeColumn", 1.1, 1.0)

        self.assertEqual(self.batchRun.nTestsFailed, origFailedTests+1)
        output = out.getvalue().strip()
        self.assertTrue(" ok " not in output)
        self.assertTrue(" FAIL " in output)
