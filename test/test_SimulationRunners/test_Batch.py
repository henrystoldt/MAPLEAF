#Created by: Henry Stoldt
#May 2020

#To run tests:
#In this file: [test_ Main.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

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
