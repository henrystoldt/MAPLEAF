#Created by: Henry Stoldt
#June 2019

#To run tests:
#In this file: [test_gridConvergenceFunctions.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import os
import shutil
import unittest

import MAPLEAF.IO.gridConvergenceFunctions as gridConvergenceFunctions


class TestGridConvergence(unittest.TestCase):
    def setUp(self):
        self.coarseVal = 0.961780
        self.medVal = 0.968540
        self.fineVal = 0.970500
        self.gridRefinemnentRatio = 2
        self.observedOrder = 1.78618479

    def test_orderOfConvergence(self):
        #Test From: https://www.grc.nasa.gov/WWW/wind/valid/tutorial/spatconv.html
        observedOrder = gridConvergenceFunctions.orderOfConvergence(self.coarseVal, self.medVal, self.fineVal, 2)
        self.assertAlmostEqual(observedOrder, self.observedOrder, 4)

        # Test negative
        observedOrder = gridConvergenceFunctions.orderOfConvergence(self.coarseVal, self.medVal, self.coarseVal, 2, minOrder=0.5)
        self.assertEqual(observedOrder, 0.5)

    def test_relError(self):
        self.assertEqual(gridConvergenceFunctions.relError(0.5, 0.75), -0.33333333333333333333)

    def test_GCI(self):
        #Test From: https://www.grc.nasa.gov/WWW/wind/valid/tutorial/spatconv.html
        testFineGCI = gridConvergenceFunctions.GCI(self.medVal, self.fineVal, 2, self.observedOrder, factorOfSafety=1.25)
        self.assertAlmostEqual(testFineGCI, 0.10308, 5)

        testCoarseGCI = gridConvergenceFunctions.GCI( self.coarseVal, self.medVal, 2, self.observedOrder, factorOfSafety=1.25)
        self.assertAlmostEqual(testCoarseGCI, 0.356249, 4)

    def test_asymptoticCheck(self):
         #Test From: https://www.grc.nasa.gov/WWW/wind/valid/tutorial/spatconv.html
        asymptCheck = gridConvergenceFunctions.asymptoticCheck(0.103083, 0.356249, 2, self.observedOrder)
        self.assertAlmostEqual(asymptCheck, 1.002019, 4)

    def test_richardson(self):
        rCVal = gridConvergenceFunctions.richardsonExtrap(self.fineVal, self.medVal, self.gridRefinemnentRatio, orderOfConvergence=self.observedOrder)
        self.assertAlmostEqual(rCVal, 0.97130, 4)

    def test_gridConvergence(self):
        observedOrder, GCI12, GCI23, asymptCheck, richardsonVal, uncertainty = gridConvergenceFunctions.checkConvergence(self.coarseVal, self.medVal, self.fineVal, self.gridRefinemnentRatio, orderTolerance=0.3)
        self.assertAlmostEqual(observedOrder, self.observedOrder, 4)
        self.assertAlmostEqual(GCI12, 0.10308, 5)
        self.assertAlmostEqual(GCI23, 0.356249, 4)
        self.assertAlmostEqual(asymptCheck, 1.002019, 4)
        self.assertAlmostEqual(richardsonVal, 0.97130, 4)

    def test_plotConvergence(self):
        # Smoke test
        X = [ 0, 1, 2, 3 ]
        coarseVals = [ 0, 1, 2, 3 ]
        medVals = [ 1, 2, 3, 4 ]
        fineVals = [ 1.5, 2.5, 3.5, 4.5 ]

        # Create temp directory for images
        tempDirName = "TempFigureDir"
        try:
            os.mkdir(tempDirName)
        except FileExistsError:
            pass
        
        # Create plot
        gridConvergenceFunctions.plotConvergence(X, coarseVals, X, medVals, X, fineVals, createZoomedInset=True, showPlot=False, saveToDirectory=tempDirName)

        # Check that images have been saved
        savedImages = os.listdir(tempDirName)
        self.assertTrue(len(savedImages), 9)

        # Delete directory and images in it
        shutil.rmtree(tempDirName)

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
