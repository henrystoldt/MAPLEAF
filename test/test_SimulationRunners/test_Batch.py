import unittest

import matplotlib.pyplot as plt

from MAPLEAF.IO import SimDefinition
from MAPLEAF.Main import isBatchSim


class TestBatchSim(unittest.TestCase):
    
    def test_isBatchSim(self):
        ''' Checks whether the batch sim detection function is working '''
        batchDefinition = SimDefinition("batchRunTemplate.mapleaf")
        self.assertTrue(isBatchSim(batchDefinition))

        normalDefinition = SimDefinition("MAPLEAF/Examples/Simulations/Wind.mapleaf")
        self.assertFalse(isBatchSim(normalDefinition))
        
    def tearDown(self):
        plt.cla()
        plt.clf()
