import unittest

import matplotlib.pyplot as plt

from MAPLEAF.IO import SimDefinition
from MAPLEAF.Main import isBatchSim
from MAPLEAF.SimulationRunners.Batch import _checkResult
from test.testUtilities import captureOutput


class TestBatchSim(unittest.TestCase):
    
    def test_isBatchSim(self):
        # Checks whether the batch sim detection function is working
        batchDefinition = SimDefinition("batchRunTemplate.mapleaf")
        self.assertTrue(isBatchSim(batchDefinition))

        normalDefinition = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf")
        self.assertFalse(isBatchSim(normalDefinition))

    def test_checkResult(self):
        # Check case that should pass
        with captureOutput() as (out, err):
            match, errorPercent = _checkResult("FakeColumn", 1.125, 1.125)

        self.assertTrue(match)
        self.assertAlmostEqual(errorPercent, 0.0)
        output = out.getvalue().strip()
        self.assertTrue(" ok " in output)
        self.assertTrue(" FAIL " not in output)

        
        # Check case that should fail
        with captureOutput() as (out, err):
            match, errorPercent = _checkResult("FakeColumn", 1.1, 1.0)

        self.assertFalse(match)
        self.assertAlmostEqual(errorPercent, 10)
        output = out.getvalue().strip()
        self.assertTrue(" ok " not in output)
        self.assertTrue(" FAIL " in output)
