import os
import sys
import unittest
from test.testUtilities import assertIterablesAlmostEqual

import MAPLEAF.IO.Logging as Logging
from MAPLEAF.IO.Logging import (Logger, MonteCarloLogger,
                                findNextAvailableNumberedFileName,
                                getSystemInfo, removeLogger)
from MAPLEAF.IO.CythonLog import Log


class TestLogger(unittest.TestCase):

    def test_Logging(self):
        output = []
        sys.stdout = Logger(output, continueWritingToTerminal=False)
        testText = "testStatement"
        print(testText)
        self.assertEqual(output, [testText, '\n'])

    def test_removeLogging(self):
        # Log some text
        output = []
        sys.stdout = Logger(output, continueWritingToTerminal=False)
        testText = "testStatement"
        print(testText)
        # Make sure it worked
        self.assertEqual(output, [testText, '\n'])
        # Remove the logger
        removeLogger()
        # Log some more text, make sure it no longer shows up in the output array
        print(testText)
        self.assertEqual(len(output), 2)

    def test_changeLoggingTarget(self):
        # Log message to first target
        output = []
        sys.stdout = Logger(output, continueWritingToTerminal=False)
        testText = "testStatement"
        print(testText)
        self.assertEqual(output, [testText, '\n'])

        # Change target
        output2 = []
        sys.stdout.changeLoggingTarget(output2)

        # Print second message
        print(testText)
        # Make sure it went to output2, not output1
        self.assertEqual(output, [testText, '\n'])
        self.assertEqual(output2, [testText, '\n'])

    def test_LoggingByLine(self):
        output = []
        log = Logger(output, continueWritingToTerminal=False)
        sys.stdout = log
        print("Test1")
        self.assertEqual(output, ["Test1", "\n"])
        log.addToLine("Time: 1s")
        self.assertEqual(output, ["Test1", "\n"])
        log.addToLine("\tForce: 1N")
        self.assertEqual(output, ["Test1", "\n"])
        log.writeLine()
        self.assertEqual(output, ["Test1", "\n", "Time: 1s\tForce: 1N\n"])

    def tearDown(self):
        removeLogger()

class TestMonteCarloLogger(unittest.TestCase):

    def test_log(self):
        monteCarloLog = []
        mCLogger = MonteCarloLogger(monteCarloLog=monteCarloLog)

        mCLogger.log("TestOutput")

        sysInfoLength = len(getSystemInfo())
        self.assertEqual(len(monteCarloLog), sysInfoLength+1)
        self.assertEqual(monteCarloLog[-1], "TestOutput")

    def test_findNextAvailableNumberedFileName(self):
        newFileName = findNextAvailableNumberedFileName(fileBaseName="testfindNextAvailableNumberedFileNameFunction", extension=".txt")
        self.assertEqual(newFileName, 'testfindNextAvailableNumberedFileNameFunction1.txt')
        
        # Create a file:
        open('testfindNextAvailableNumberedFileNameFunction1.txt', 'w+').close()

        newFileName = findNextAvailableNumberedFileName(fileBaseName="testfindNextAvailableNumberedFileNameFunction", extension=".txt")
        self.assertEqual(newFileName, 'testfindNextAvailableNumberedFileNameFunction2.txt')

        # Create another file:
        open('testfindNextAvailableNumberedFileNameFunction2.txt', 'w+').close()
        open('testfindNextAvailableNumberedFileNameFunction4.txt', 'w+').close()
        newFileName = findNextAvailableNumberedFileName(fileBaseName="testfindNextAvailableNumberedFileNameFunction", extension=".txt")
        self.assertEqual(newFileName, 'testfindNextAvailableNumberedFileNameFunction3.txt')

        # Delete files
        os.remove('testfindNextAvailableNumberedFileNameFunction1.txt')
        os.remove('testfindNextAvailableNumberedFileNameFunction2.txt')
        os.remove('testfindNextAvailableNumberedFileNameFunction4.txt')

    def test_writeToFile(self):
        logArray = []
        mCLogger = MonteCarloLogger(monteCarloLog=logArray)
        mCLogger.log("line1")
        mCLogger.log("line2")
        mCLogger.writeToFile(filePath="test_MCLogger_writeToFileOutput12345.txt")

        self.assertTrue(os.path.isfile("test_MCLogger_writeToFileOutput12345.txt"))
        os.remove("test_MCLogger_writeToFileOutput12345.txt")

class TestOtherLogging(unittest.TestCase):
    def test_LogPostProcessing(self):
        import pandas as pd
        testLogPath = "./test/test_IO/sampleFakeForcesLog.csv"
        expandedLogPath = Logging.postProcessForceEvalLog(testLogPath, refArea=3.5, refLength=2.1)

        # Load new log and check values
        origLog = pd.read_csv(testLogPath)
        newLog = pd.read_csv(expandedLogPath)

        columnsToCheckMatch = {
            "AeroCFX": "CFX",
            "AeroCMY": "CMY",
            "Cd": "Cd",
            "Cl": "Cl",
            "CN": "CN",
        }
        
        for col in columnsToCheckMatch:
            calculatedColumnName = col
            expectedColumnName = columnsToCheckMatch[col]
            calculatedCol = list(newLog[calculatedColumnName])
            expectedCol = list(origLog[expectedColumnName])
            assertIterablesAlmostEqual(self, calculatedCol, expectedCol)

        # Delete the new expanded log file
        os.remove(expandedLogPath)

class TestLog(unittest.TestCase):

    def setUp(self):
        cols = [ "PositionX", "PositionY" ]
        log = Log(cols)

        log.newLogRow(0.0)
        log.logValue("PositionX", 0.1)
        log.logValue("PositionY", 0.2)
        self.log = log

    def test_basicLogging(self):
        log = self.log
        # Check values are in there properly
        x1 = log.getValue(0, "PositionX")
        y1 = log.getValue(0, "PositionY")
        self.assertAlmostEqual(x1, 0.1)
        self.assertAlmostEqual(y1, 0.2)

    def test_completeLastLine(self):
        log = self.log
        log.newLogRow(0.1)
        log.newLogRow(0.2)

        # Check that t==0.1 values where filled in with the fill value (0)
        x2 = log.getValue(0.1, "PositionX")
        y2 = log.getValue(0.1, "PositionY")
        self.assertAlmostEqual(x2, 0)
        self.assertAlmostEqual(y2, 0)

    def test_addColumn(self):
        log = self.log
        newCol = log.addColumn("newCol")
        self.assertEqual(newCol, [ 0 ])

    def test_deleteLastRow(self):
        log = self.log
        log.deleteLastRow()
        self.assertEqual(0, len(log.logColumns["Time(s)"]))

    def test_writeToCSV(self):
        log = self.log

        log.writeToCSV("test/test_IO/testCSVLogOutput.csv")

        with open("test/test_IO/testCSVLogOutput.csv") as outputtedFile:
            output = outputtedFile.read()

        with open("test/test_IO/correctCSVLogOutput.csv") as correctFile:
            expectedOutput = correctFile.read()

        self.assertEqual(output, expectedOutput)

    def test_getValue(self):
        log = self.log
        val = log.getValue(0, "PositionX")
        self.assertEqual(val, 0.1)

    def test_logValue(self):
        log = self.log
        log.newLogRow(0.1)        
        log.logValue("PositionX", 0.3)
        log.logValue("PositionY", 0.4)        
        
        val = log.getValue(0.1, "PositionX")
        self.assertEqual(val, 0.3)

        val = log.getValue(0.1, "PositionY")
        self.assertEqual(val, 0.4)


if __name__ == '__main__':
    unittest.main()
