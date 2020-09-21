'''
    Script to run a series of regression tests, defined below
'''

import argparse
import os
import sys
import time
from math import isnan

import matplotlib.pyplot as plt
import numpy as np

from MAPLEAF.IO import (Logging, Plotting, SimDefinition, SubDictReader,
                        gridConvergenceFunctions)
from MAPLEAF.Main import findSimDefinitionFile
from MAPLEAF.Motion import Vector
from MAPLEAF.SimulationRunners import Simulation, WindTunnelSimulation

# Make sure MAPLEAF/ is accessible - should run these statements BEFORE MAPLEAF imports below
mainDirectoryPath = os.path.abspath(os.getcwd())
sys.path.append(mainDirectoryPath)

warningCount = 0

#### OPTIONS ####
percentageErrorTolerance = 0.01 # %
#### END OPTIONS ####

#### Main ####
def batchRun(batchDefinition, caseNameSpec=None, recordAll=False, printStackTraces=False):
    '''
        Given a batchDefinition object (of type SimDefinition), will run all of the test cases defined in it, and print a summary of the results
    '''
    # Track how long running cases takes
    startTime = time.time()

    # Get all the regression test cases
    testCases = batchDefinition.getImmediateSubDicts("")
    nCases = 0
    nCasesOk = 0
    nTestsOk = 0
    nTestsFailed = 0
    totalSimError = 0
    nComparisonSets = 0
    casesWhoseExpectedResultsWereUpdated = []
    for testCase in testCases:
        if caseNameSpec == None or (caseNameSpec in testCase):
            nCases += 1
            nOk, nFail, newExpectedResultsRecorded, errorStats = _runSingleTestCase(testCase, batchDefinition, recordAll=recordAll, printStackTraces=printStackTraces)

            totalSimError += errorStats[0]
            nComparisonSets += errorStats[1]
            
            if newExpectedResultsRecorded:
                casesWhoseExpectedResultsWereUpdated.append(testCase)

            # Track number of case and test successes / failures
            if nFail == 0:
                nCasesOk += 1
                nTestsOk += nOk
            else:
                nTestsOk += nOk
                nTestsFailed += nFail


    runTime = time.time() - startTime

    # Output result summary
    print("----------------------------------------------------------------------")
    print("Time to run {} Case(s): {:>.2f} s".format(nCases, runTime))

    if nComparisonSets > 0:
        totalSimAvgError = totalSimError/nComparisonSets
        print("")
        print("Total Average Error for Entire Simulation: %6.2f %%\n" %(totalSimAvgError))
    
    if warningCount > 0:
        print("Errors/Warnings: {}".format(warningCount))

    if len(casesWhoseExpectedResultsWereUpdated) > 0:
        print("New expected results were recorded for the following cases: {}".format(", ".join(casesWhoseExpectedResultsWereUpdated)))
        _writeModifiedTestDefinitionFile()        

    if nCasesOk == nCases:
        print("{} Case(s) ok".format(nCases))
        print("")
        if warningCount == 0:
            print("OK")
        else:
            print("WARNING")
    else:
        nCasesFailed = nCases - nCasesOk
        nTests = nTestsOk + nTestsFailed
        print("{}/{} Case(s) Failed, {}/{} Parameter Comparison(s) Failed".format(nCasesFailed, nCases, nTestsFailed, nTests))
        print("")
        print("FAIL")

#### 1. Load / Run Sim ####
def _runSingleTestCase(caseName, batchDefinition, recordAll=False, printStackTraces=False):
    '''
        Runs a single regression tests case, compares the results to the expected results provided, and generates any desired plots.
            If no comparison data is provided, comparison data is recorded

        Inputs:
            caseName:                   (string) Name of current case / top-level dictionary
            batchDefinition:  (SimDefinition) Should have the batchDefinition file loaded

        Outputs:
            Returns:    numTestsOk(Int), numTestsFailed(Int), resultValuesRecorded(Bool)
                resultValuesRecorded is True if absent ExpectedResults were added to the regressionTestDefinition
                    Used to remember that an updated regressionTestDefinition shoudl be written to file at the end of all the test cases
            Modifies:   batchDefinition - records sim results is no expected results are provided
            Prints:     One line to introduce case, one more line for each expected results
    '''
    print("Running Case: {}".format(caseName))
    caseDictReader = SubDictReader(caseName, simDefinition=batchDefinition)

    newExpectedValuesRecorded = False
    
    #### Load Sim definition file ####
    simDefFilePath = caseDictReader.getString("simDefinitionFile")
    simDefinition = SimDefinition(simDefFilePath, silent=True)

    #### Parameter overrides ####
    _implementParameterOverrides(caseName, batchDefinition, simDefinition)

    #### Run simulation ####
    # Check whether simulation is a full flight sim or a parameter sweeping simulation
    caseSubDictionaries = caseDictReader.getImmediateSubDicts()
    errorStats = [0.0,0]
    if caseName + ".ParameterSweep" in caseSubDictionaries:
        logFilePaths, numTestsOk, numTestsFailed, newExpectedValuesRecorded, errorStats = _runParameterSweepCase(caseDictReader, simDefinition, batchDefinition, recordAll=recordAll, printStackTraces=printStackTraces)
    else:
        logFilePaths, numTestsOk, numTestsFailed, newExpectedValuesRecorded = _runFullFlightCase(caseDictReader, simDefinition, batchDefinition, recordAll=recordAll, printStackTraces=printStackTraces)

    #### Generate/Save plots ####
    if len(logFilePaths) > 0: # Don't generate plots of simulation crashed (didn't produce any log files)
        # Get all plot subdictionaries
        plotsToGeneratePath = ".".join([caseName, "PlotsToGenerate"])
        plotDicts = batchDefinition.getImmediateSubKeys(plotsToGeneratePath)
        for plotDict in plotDicts:
            plotDictReader = SubDictReader(plotDict, simDefinition=batchDefinition)
            _generatePlot(plotDictReader, logFilePaths)

    # Print blank line before next case or final summary
    print("")

    return numTestsOk, numTestsFailed, newExpectedValuesRecorded, errorStats

def _implementParameterOverrides(testCase, batchDefinition, simDefinition):
    '''
        Inputs:
            testCase:   (string) name of the current test case / top level dictionary
            batchDefinition:      (SimDefinition) The sim definition object that's loaded/parsed the testDefinitions.mapleaf file
            simDefinition:                  (SimDefinition) The sim definition object that's loaded/parsed the definition file for the current regression testing simulation

        Outputs:
            Modifies: simDefinition, according to the parameter overrides specified in the regression tests definition file
    '''
    #### Load and enact parameter overrides ####
    # Always disable plotting and enable logging
    simDefinition.setValue("SimControl.plot", "None")
    simDefinition.setValue("SimControl.loggingLevel", "3")
    simDefinition.setValue("SimControl.RocketPlot", "Off")

    # Look for other overrides in the definition file
    parameterOverridesDictKey = ".".join([testCase, "ParameterOverrides"])
    parameterOverrides = batchDefinition.getSubKeys(parameterOverridesDictKey)
    for paramOverrideKey in parameterOverrides:
        overridenKey = paramOverrideKey.replace(parameterOverridesDictKey+".", "")
        overrideValue = batchDefinition.getValue(paramOverrideKey)

        # Implement them
        simDefinition.setValue(overridenKey, overrideValue)

def _runParameterSweepCase(caseDictReader, simDefinition, batchDefinition, recordAll=False, printStackTraces=False):
    # Run parameter sweep simulation and check result
    print("  Parameter Sweep Case")

    # Get input data from text file
    ParameterSweeps = []
    expectedResultsDicts = []
    ParameterSweepDicts = caseDictReader.getImmediateSubDicts(caseDictReader.simDefDictPathToReadFrom + ".ParameterSweep")
    for SubDict in ParameterSweepDicts:
        if 'Results' not in SubDict: # any subdict in parameterSweep that is NOT a results dict, will be assumed to be a parameter sweep dict
            ParameterSweeps.append(SubDict)
        else:
            expectedResultsDicts.append(SubDict)

    sweptParameters = []
    parameterValues = []
    for parameter in ParameterSweeps:
        sweptParameters.append(batchDefinition.getValue(parameter + '.sweptParameter'))
        parameterValues.append(batchDefinition.getValue(parameter + '.parameterValues'))

    parameterValues = [ _parseParameterSweepValues(valString) for valString in parameterValues ]

    if not all(len(sub) == len(parameterValues[0]) for sub in parameterValues):
        print("parameterSweep values are not equal lengths.")
    
    # Check whether to run 10x the points to make the line smooth
    if len(parameterValues) < 25:
        smoothLineDefault = 'True'
    else:
        smoothLineDefault = 'False'
    smoothLine = caseDictReader.tryGetString('ParameterSweep.smoothLine', defaultValue=smoothLineDefault)

    # Run simulation
    simRunner = WindTunnelSimulation(parameterToSweepKey=sweptParameters, parameterValueList=parameterValues, fW=simDefinition, silent=True, smoothLine=smoothLine)
    try:
        logFilePaths = simRunner.runSweep()
    except:
        # Simulation Failed
        Logging.removeLogger()
        print("  ERROR: Simulation Crashed")
        _incrementWarningCount()

        if printStackTraces:
            import traceback
            tb = traceback.format_exc()
            print(tb)

        numTestsOk, numTestsFailed, newExpectedValuesRecorded = (0, 1, False)
        return [], numTestsOk, numTestsFailed, newExpectedValuesRecorded, [0, 0]
    else:
        Logging.removeLogger()

    # Check results
    numTestsOk = 0
    numTestsFailed = 0
    newExpectedValuesRecorded = False
    compareToSim = False
    errorStats = [0.0, 0]
    plotsSubDicts = caseDictReader.getImmediateSubDicts(caseDictReader.simDefDictPathToReadFrom + ".PlotsToGenerate") # access plot sub dicts
    for plotSubDict in plotsSubDicts: # loop through plot sub dicts
        comparisonResults = caseDictReader.getImmediateSubDicts(plotSubDict) # access the comparison data dicts
        for comparisonResult in comparisonResults: # loop through comparison data dicts
            if caseDictReader.tryGetString(comparisonResult + ".compareToSim") != None: # check for compareToSim key
                compareToSim = True # conditional variable needed to not divide by 0 later on in the error calcs
                expectedResultsDicts.append(comparisonResult) # if compareToSim exists, add that comparisonDataDict to the expectedResultsDict
                errorStats[1] +=1

    compCasesCount = 0 # initialize, final total will be equal to number of plotted comparison data sets included in error calcs
    totalAvgError = 0 # initialize, this variable tracks total avg error for all plotted comparison data that is elected to be included in error calcs
    for expectedResultsDict in expectedResultsDicts: # loop through expected results. Manually inputed values, as well as comparisonData in the plots


        if caseDictReader.tryGetString(expectedResultsDict + ".compareToSim") != None: # this checks if the expected results came from comparisonPlot data, or user inputted expectedResults
            showAvgError = True # do an average percent error calc for this data set
            compCasesCount += 1 # increase count for comparison data sets included in error calcs

            expectedResultsCol = caseDictReader.getString(expectedResultsDict + ".compareToSim") # get column header that contains sim results in log files
            compResultsCol = caseDictReader.getString(expectedResultsDict + ".columnsToPlot") # get column header that contains sim results in log files
            expectedResultsPath = caseDictReader.tryGetString(expectedResultsDict + ".file", defaultValue=None)
            expectedResults, compColNames = Plotting.getLoggedColumns(expectedResultsPath, compResultsCol, sep=',') # get expected results from .csv file
            expectedResults = expectedResults[0] # take sublist and make it main list
        else: # if the data comes from expected results, and not the comparison data dictionary, the above 'if' statement will be false, and this will be executed
            expectedResultsCol = caseDictReader.getString(expectedResultsDict + ".column") # get column header that contains results in log files
            expectedResults = caseDictReader.getString(expectedResultsDict + ".expectedValues").split(',') # get expected results values that will be compared against sim
            showAvgError = False # do not do a percent error calc for this data set

        # Get results to be checked
        resultData = []
        for logPath in logFilePaths:
            columnDataLists, columnNames = Plotting.getLoggedColumns(logPath, expectedResultsCol)

            if len(columnNames) == 1:
                resultData = columnDataLists[0]
            elif len(columnNames) > 1:
                print("  ERROR: Column Spec '{}' matched more than one column: {} in log file: '{}'".format(expectedResultsCol, columnNames, logPath))
                _incrementWarningCount()
                numTestsOk, numTestsFailed, newExpectedValuesRecorded = (0, 1, False)
                return numTestsOk, numTestsFailed, newExpectedValuesRecorded, [0, 0]

        # Error check (no result columns found)
        if len(resultData) == 0:
            print("  ERROR: Column Spec {} did not match any columns".format(expectedResultsCol))
            _incrementWarningCount()
            numTestsOk, numTestsFailed, newExpectedValuesRecorded = (0, 1, False)
            return numTestsOk, numTestsFailed, newExpectedValuesRecorded, [0, 0]

        # Record / Check Results
        if (len(expectedResults) == 1 and expectedResults[0].lower() == "record") or recordAll:
            # Record results for future runs
            for value in resultData:
                print("  {:<25} Recorded {:>15.7}".format(expectedResultsCol + ":", value))

            key = expectedResultsDict + ".expectedValues"
            resultData = [ str(x) for x in resultData ]
            value = ",".join(resultData)
            batchDefinition.setValue(key, value)
            newExpectedValuesRecorded = True
        else:
            # Check that there's the same number of parameter values and expected results
            # if len(parameterValues) != len(expectedResults):
            #     print("ERROR: Must have the same number of sweep values (currently {}) and expected results (currently {}).".format(len(parameterValues), len(expectedResults)))
            #     _incrementWarningCount()
            #     numTestsOk, numTestsFailed, newExpectedValuesRecorded = (0, 1, False)
            #     return numTestsOk, numTestsFailed, newExpectedValuesRecorded

            # Compare obtained results to expected ones
            totalCaseError = 0    # initalize, used to track total error in each individual case        

            if smoothLine == "True":
                for i in range(len(expectedResults)):
                    match, errorPercent = _checkResult(expectedResultsCol, resultData[i*10], float(expectedResults[i]))
                    totalCaseError += abs(errorPercent) # add individual instance of a parameter's error to total case error
                    if match:
                        numTestsOk += 1
                    else:
                        numTestsFailed += 1
            else:
                for i in range(len(expectedResults)):
                    match, errorPercent = _checkResult(expectedResultsCol, resultData[i], float(expectedResults[i]))
                    totalCaseError += abs(errorPercent) # add individual instance of a parameter's error to total case error
                    if match:
                        numTestsOk += 1
                    else:
                        numTestsFailed += 1  

            if showAvgError: # if the case came from plotted comparison data, display the total parameter error
                
                avgCaseError = totalCaseError/(i+1) # take avg of the total
                print("Total average error for the above parameter comparison:  %6.2f %%\n" %(avgCaseError))
                totalAvgError += avgCaseError # add total avg parameter error to total case error

    errorStats[0] = totalAvgError
    if compareToSim: # required in order to not divide by 0 for cases that do not contain the compareToSim key in any of their comparison data dicts
        totalAvgError /= compCasesCount # get average error for all parameters in this case (only for data that came from plotted comparison data)
        print("Total average error for all comparison data: %6.2f %%\n" %(totalAvgError)) # Well I am sure you know what this does

    return logFilePaths, numTestsOk, numTestsFailed, newExpectedValuesRecorded, errorStats

def _parseParameterSweepValues(parameterValues):
    '''
        Pass in the raw string read from the parameterValues field in a testDefinition.
        Returns a list of strings representing each parameter value to run at.
    '''
    # Check whether a range of parameter values is specified
    if ':' in parameterValues:
        # Create list of values from range
        initVal, numSteps, finalVal = parameterValues.split(':')
        numSteps = int(numSteps)

        try:
            # Range of Scalar values
            initVal = float(initVal) # This line will raise a ValueError if the values are vectors
            finalVal = float(finalVal)
            parameterValues = list(np.linspace(initVal, finalVal, num=numSteps))
            parameterValues = [ str(x) for x in parameterValues ] # Convert back to strings for WindTunnelSimRunner
        
        except ValueError:
            # Range of Vector values
            initVal = Vector(initVal)
            finalVal = Vector(finalVal)
            xVals = list(np.linspace(initVal.X, finalVal.X, num=numSteps))
            yVals = list(np.linspace(initVal.Y, finalVal.Y, num=numSteps))
            zVals = list(np.linspace(initVal.Z, finalVal.Z, num=numSteps))
            
            # Populate list with string values for WindTunnelSimRunner
            parameterValues = []
            for i in range(numSteps):
                interpolatedVector = Vector(xVals[i], yVals[i], zVals[i])
                parameterValues.append(str(interpolatedVector))
    else:
        # Regular list of values
        parameterValues = parameterValues.split(',')

    return parameterValues

def _runFullFlightCase(caseDictReader, simDefinition, batchDefinition, recordAll=False, printStackTraces=False):
    # Run regular full-flight simulation and check result
    print("  Full Flight Case")
    try:
        simRunner = Simulation(simDefinition=simDefinition, silent=True)
        _, logFilePaths = simRunner.run()
    except:
        # Simulation Failed
        Logging.removeLogger() # Make sure we can print to the console
        print("  ERROR: Simulation Crashed")
        _incrementWarningCount()

        if printStackTraces:
            import traceback
            tb = traceback.format_exc()
            print(tb)
            
        numTestsOk, numTestsFailed, newExpectedValuesRecorded = (0, 1, False)
        return [], numTestsOk, numTestsFailed, newExpectedValuesRecorded
    else:
        Logging.removeLogger()

    #### Compare and/or record numerical results, output pass/fail ####
    expectedResultsDictKey = ".".join([caseDictReader.simDefDictPathToReadFrom, "ExpectedResultsAtEndOfSim"])
    expectedResultKeys = batchDefinition.getSubKeys(expectedResultsDictKey)

    numTestsOk = 0
    newExpectedValuesRecorded = False
    if len(expectedResultKeys) == 0:
        # If no expected results are provided, record some
        _recordDefaultSimResults(batchDefinition, logFilePaths, expectedResultsDictKey)
        newExpectedValuesRecorded = True
    else:
        # If values were provided, check them
        numTestsOk, newValuesRecorded = _checkSimResults(batchDefinition, caseDictReader, logFilePaths, expectedResultsDictKey, expectedResultKeys, recordAll=recordAll)
        if newValuesRecorded:
            newExpectedValuesRecorded = True

    numTestsFailed = len(expectedResultKeys) - numTestsOk
    return logFilePaths, numTestsOk, numTestsFailed, newExpectedValuesRecorded

#### 2. Checking Expected Final Results ####
def _recordDefaultSimResults(batchDefinition, logFilePaths, expectedResultsDictKey):
    ## No expected values present ##
    print("  WARNING: No expected parameter values provided. Recording Position & Velocity values.")
    _incrementWarningCount()

    # Record Position, Velocity values for future regression test runs
    columnSpecs = [ "Position", "^Velocity" ]
    dataLists, names = Plotting.getLoggedColumns(logFilePaths[0], columnSpecs)
    for i in range(len(names)):
        print("  {:<25} Recorded {:>15.7}".format(names[i] + ":", dataLists[i][-1]))

        key = ".".join([expectedResultsDictKey, names[i]])
        observedValue = str(dataLists[i][-1])
        batchDefinition.setValue(key, observedValue)

def _checkSimResults(batchDefinition, caseDictReader, logFilePaths, expectedResultsDictKey, expectedResultKeys, recordAll=False):
    numTestsOk = 0
    resultValuesRecorded = False
    for resultKey in expectedResultKeys:
        # Get expected final value for this parameter
        logColumnSpec = resultKey.replace(expectedResultsDictKey+".", "")

        try:
            if recordAll:
                raise ValueError("Let's record a value! #Flow Control by exceptions")

            ## Regular Parameter Check ##
            expectedResult = caseDictReader.getFloat(resultKey)
            # Get and compare observed final value for this parameter to its expected value
            observedResult, columnName = _getSingleResultFromLogs(logFilePaths, logColumnSpec)
            match, errorPercent = _checkResult(columnName, observedResult, expectedResult)

            # Keep track of the number of successes (and implicitly the number failures)
            if match:
                numTestsOk += 1

        except ValueError:
            ## Check whether we're meant to record/save the current sim result for this parameter ##
            expectedResult = caseDictReader.getString(resultKey)
            if expectedResult.lower() == "record" or recordAll:
                # Get and save value of parameter from current simulation
                observedValue, colName = _getSingleResultFromLogs(logFilePaths, logColumnSpec)
                
                print("  {:<25} Recorded {:>15.7}".format(colName + ":", observedValue))

                key = ".".join([expectedResultsDictKey, logColumnSpec])
                value = str(observedValue)
                batchDefinition.setValue(key, value)

                numTestsOk += 1

                resultValuesRecorded = True
            else:
                ## Parsing error ##
                print("  ERROR: Expected value: {} for parameter: {} not numeric or 'Record'".format(expectedResult, resultKey))
                _incrementWarningCount()

    return numTestsOk, resultValuesRecorded

def _checkResult(columnName, observedResult, expectedResult):
    '''
        Inputs:
            logFilePaths:   (List (string)) List of paths to simulation log files
            logColumnSpec:  (string) Partial or Full log column name, or regex expression. Should match exactly 1 log column
            expectedResult: (numeric) Expected value of the data in the column identified by logColumnSpec, in the very last row of data

        Outputs:
            Returns: checkPassed(bool), columnName(string)

            Prints: 1 line, success or failure

    '''    
    if observedResult != None:
        error = abs(expectedResult - observedResult)
        if expectedResult != 0:
            errorPercent = abs(error * 100 / expectedResult)
        elif expectedResult == observedResult:
            errorPercent = 0
        else:
            errorPercent = 100

        global percentageErrorTolerance
        if errorPercent > percentageErrorTolerance or isnan(errorPercent):
            print("  {:<25} FAIL     {:>15.7}, Expected: {:>15.7}, Disagreement: {:>10.2f} %".format(columnName + ":", observedResult, expectedResult, errorPercent))
            return False, errorPercent
        else:
            print("  {:<25} ok       {:>15.7}".format(columnName + ":", expectedResult))
            return True, 0
    else:
        return False

def _getSingleResultFromLogs(logFilePaths, logColumnSpec):
    '''
        Will return the last value in the log column defined by logColumn Spec. Will search in each file in logFilePaths
    '''
    for logPath in logFilePaths:
        dataLists, columnNames = Plotting.getLoggedColumns(logPath, [ logColumnSpec ])

        if len(dataLists) > 1:
            print("  ERROR: Column Spec '{}' matched more than one column: {} in log file: '{}'".format(logColumnSpec, columnNames, logPath))
            _incrementWarningCount()
            return None, logColumnSpec

        if len(dataLists) == 1:
            columnName = columnNames[0]
            observedResult = dataLists[0][-1]
            return observedResult, columnName
    
    # No column was found
    print("  ERROR: Column Spec {} did not match any columns".format(logColumnSpec))
    _incrementWarningCount()
    return None, None

#### 3. Plotting ####
def _generatePlot(plotDictReader, logFilePaths):
    '''
        Inputs:
            plotDictReader:     (SubDictReader) Initialized to read from the subdirectory of PlotsToGenerate that defines the desired plot
            logFilePaths:       (list (string)) 

        Outputs:
            Saves png, pdf, and eps plots to the location specified by .saveLocation
    '''
    # Create plot
    fig, ax = plt.subplots(figsize=(6,4))

    #### Plot Data from current simulation ####
    # Get all entries in the PlotsToGenerate dictionary
    columnSpecs = plotDictReader.tryGetString("columnsToPlot", defaultValue="").split()
    nLinesToPlot = len(columnSpecs)
    if nLinesToPlot == 0:
        return

    lineFormat = plotDictReader.tryGetString("lineFormat", defaultValue="y--").split()
    while len(lineFormat) < nLinesToPlot:
        lineFormat.append("")

    legendLabel = plotDictReader.tryGetString("legendLabel", defaultValue=columnSpecs[0]).split(',')
    while len(legendLabel) < nLinesToPlot:
        legendLabel.append(columnSpecs[len(legendLabel)])

    scalingFactor = plotDictReader.tryGetFloat("scalingFactor", defaultValue=1.0)
    offset = plotDictReader.tryGetFloat('offset', defaultValue=0.0)

    xLim = plotDictReader.tryGetString("xLimits", defaultValue="False").split() # Expected length: 2
    yLim = plotDictReader.tryGetString("yLimits", defaultValue="False").split() # Expected length: 2

    lists = [ lineFormat, legendLabel, scalingFactor ]
    fillValues = [ "", "", "" ]

    # Make sure we're set to get the time/x column along with the y-data
    xColumnName = plotDictReader.tryGetString("xColumnName", defaultValue="Time(s)")
    if xColumnName not in columnSpecs:
        columnSpecs.append(xColumnName)

    # Plot all the requested data
    plottedColumns = []
    for logFilePath in logFilePaths:
        columnData, columnNames = Plotting.getLoggedColumns(logFilePath, columnSpecs, plottedColumns)
        if len(columnNames) > 1:
            # Only plot if we've found (at minimum) an X-column and a Y-column
            _plotData(ax, columnData, columnNames, xColumnName, lineFormat, legendLabel, scalingFactor, offset, xLim, yLim, adjustXaxisToFit=True)

            # Don't retrieve the same columns from another log file
            plottedColumns += columnNames

            # Remove xColumnName if it's present
            if xColumnName in plottedColumns:
                plottedColumns.remove(xColumnName)

    #### Plot comparison data ####
    # Load comparison data file
    compDataDictionaries = plotDictReader.simDefinition.getImmediateSubDicts(plotDictReader.simDefDictPathToReadFrom)
    for compDataDict in compDataDictionaries:
        compDataDictReader = SubDictReader(compDataDict, plotDictReader.simDefinition)
        _plotComparisonData(ax, compDataDictReader)
    
    #### Finalize Plot ####
    # Set x and y labels
    xLabel = plotDictReader.tryGetString("xLabel", defaultValue=xColumnName)
    yLabel = plotDictReader.tryGetString("yLabel", defaultValue=columnSpecs[0])
    ax.set_xlabel(latexLabelTranslation(xLabel))
    ax.set_ylabel(latexLabelTranslation(yLabel))

    ax.legend()
    
    if yLim == "False":
        ax.autoscale(axis='y', tight=True)
    else:
        pass

    fig.tight_layout()

    # Get save location
    saveFilePath = plotDictReader.getString("saveLocation")
    saveDirectory = os.path.dirname(saveFilePath)
    saveFileName = os.path.basename(saveFilePath)

    overwriteImages = plotDictReader.tryGetBool("overwriteImages", defaultValue=True)

    # Save plot
    gridConvergenceFunctions.saveFigureAndPrintNotification(saveFileName, fig, saveDirectory, overwrite=overwriteImages, epsVersion=False, pngVersion=False, printStatementPrefix="  ")

    # Close figure to avoid keeping them all in memory (Matplotlib gives warning about this - thank you Matplotlib developers!)
    plt.close(fig)

def _plotComparisonData(ax, compDataDictReader):
    compDataPath = compDataDictReader.tryGetString("file", defaultValue=None)
    compColumnSpecs = compDataDictReader.tryGetString("columnsToPlot", defaultValue="").split()
    xColumnName = compDataDictReader.tryGetString("xColumnName", defaultValue="Time(s)")
    lineFormat = compDataDictReader.tryGetString("lineFormat", defaultValue="k-").split()
    legendLabel = compDataDictReader.tryGetString("legendLabel", defaultValue="Label").split(',')
    scalingFactor = compDataDictReader.tryGetFloat("scalingFactor", defaultValue=1.0)

    # If comparison data entries found in the plot dictionary, load and plot the comparison data
    if compDataPath != None and len(compColumnSpecs) > 0:
        # Plot comparison data columns
        if xColumnName not in compColumnSpecs:
            compColumnSpecs.append(xColumnName)

        try:
            compColData, compColNames = Plotting.getLoggedColumns(compDataPath, compColumnSpecs, sep=',')

            if len(compColData) < len(compColumnSpecs):
                print("  ERROR: Found {} columns of comparison data: {} for {} column specs: {} in file: {}".format(len(compColData), compColNames, len(compColumnSpecs), compColumnSpecs, compDataPath))
                _incrementWarningCount()

            _plotData(ax, compColData, compColNames, xColumnName, lineFormat, legendLabel, scalingFactor)

        except FileNotFoundError:
            print("  ERROR: Comparison data file: {} not found".format(compDataPath))
            _incrementWarningCount()

def _plotData(ax, dataLists, columnNames, xColumnName, lineFormat, legendLabel, scalingFactor, offset=0, xLim=["False"], yLim=["False"], adjustXaxisToFit=False):
    '''
        ax:             (Matplotlib.Axes) to plot on
        dataLists:      (list (list (float))) each sub-list should a vector of x or y data
        columnNames:    (list (string)) list of column names, order matching that of dataLists
        xColumnName:    (string) Name of the column that will serve as the 'x' data. Every other column will be assumed to contain 'y' data
    '''
    # Extract the x-column data
    xData = []
    for i in range(len(columnNames)):
        if columnNames[i] == xColumnName:
            xData = dataLists.pop(i)
            columnNames.pop(i)

    if adjustXaxisToFit:
        # This will be True if we're plotting the current sim's output
        if xLim[0] != "False":
            xLowerLim = float(xLim[0])
            xUpperLim = float(xLim[1])
            ax.set_xlim([xLowerLim,xUpperLim])

        else:
            ax.set_xlim([xData[0], xData[-1]])

        if yLim[0] != "False":
            yLowerLim = float(yLim[0])
            yUpperLim = float(yLim[1])
            ax.set_ylim([yLowerLim,yUpperLim])

    # Scale data and apply offset:
    for i in range(len(dataLists)):
        for j in range(len(dataLists[i])):
            a = dataLists[i][j]
            dataLists[i][j] = scalingFactor*float(dataLists[i][j]) + offset

    # Plot obtained data
    for i in range(len(columnNames)):
        line = lineFormat[i]

        if len(xData) > 1:
            # Plot line
            if adjustXaxisToFit:
                ax.plot(xData, dataLists[i], line, label = legendLabel[i], linewidth=3)
            else:
                ax.plot(xData, dataLists[i], line, label = legendLabel[i])
        else:
            # Plot point
            if adjustXaxisToFit:
                ax.scatter(xData, dataLists[i],  label = legendLabel[i], linewidth=3)
            else:
                ax.scatter(xData, dataLists[i],  label = legendLabel[i])            

#### Utility functions ####
def _incrementWarningCount():
    global warningCount
    warningCount += 1

def _writeModifiedTestDefinitionFile():
    newTestDefinitionPath = "./test/regressionTesting/testDefinitions_newExpectedResultsRecorded.mapleaf"
    print("Writing new testDefinition file to: {}".format(newTestDefinitionPath))
    print("  If desired, use this file (or values from this file) to replace/update testDefinitions.mapleaf\n")
    batchDefinition.writeToFile(newTestDefinitionPath, writeHeader=False)

def latexLabelTranslation(labelInput):
    labelDict = {
        '$\alpha$': r'$\alpha$',
        '$C_l$'   : r'$C_l$',
        '$C_d$'   : r'$C_d$',
        '$C_n$'   : r'$C_n$',
        '$C_y$'   : r'$C_y$',
        '$C_N$'   : r'$C_N$',
        '$C_A$'   : r'$C_A$'
    }
    
    if labelInput in labelDict:
        return labelDict[labelInput]
    else:
        return labelInput

def buildParser():
    parser = argparse.ArgumentParser(description="Batch-run MAPLEAF simulations")
    parser.add_argument(
        "--recordAll", 
        action='store_true', 
        help="If present, re-records all expected results for cases that are run. Recorded data outputted to ./test/regressionTesting/testDefinitions_newExpectedResultsRecorded.mapleaf"
    )
    parser.add_argument(
        "--printStackTraces", 
        action='store_true', 
        help="If present, stack traces are printed for crashed simulations"
    )
    parser.add_argument(
        "--filter", 
        nargs=1, 
        default=[], 
        help="Provide a string.  Only cases whose name includes this string will be run."
    )
    parser.add_argument(
        "batchDefinitionFile", 
        nargs='?', 
        default=["./test/regressionTesting/testDefinitions.mapleaf"], 
        help="Path to a batch definition (.mapleaf) file. Default is ./test/regressionTesting/testDefinitions.mapleaf"
    )

    return parser

if __name__ == "__main__":
    # Load the test definition database
    if os.path.basename(os.getcwd()) == "regressionTesting":
        os.chdir("../..")
    elif os.path.basename(os.getcwd()) == "test":
        os.chdir("..")
    
    # Parse command line arguments
    parser = buildParser()
    args = parser.parse_args()

    # Load definition file
    batchDefinitionPath = findSimDefinitionFile(args.batchDefinitionFile[0])
    batchDefinition = SimDefinition(batchDefinitionPath, defaultDict={}, silent=True)

    # Filter cases by name if required
    if len(args.filter) > 0:
        caseNameSpec=args.filter[0] # Run specific case(s)
    else:
        caseNameSpec = None # Run all cases

    # Run Cases
    batchRun(batchDefinition, caseNameSpec=caseNameSpec, recordAll=args.recordAll, printStackTraces=args.printStackTraces)
