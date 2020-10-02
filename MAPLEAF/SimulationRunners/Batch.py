''' 
    Script to run a batch of simulations, defined in a batch definition file. Can be run directly from the command line. 
    Accessible as `mapleaf-batch` if MAPLEAF is installed through pip.
'''
import argparse
import os
import time
from distutils.util import strtobool
from math import isnan
from statistics import mean
from typing import Union

import matplotlib.pyplot as plt
import numpy as np
from MAPLEAF.IO import (Logging, Plotting, SimDefinition, SubDictReader,
                        gridConvergenceFunctions)
from MAPLEAF.Motion import Vector
from MAPLEAF.Motion.Interpolation import linInterp
from MAPLEAF.SimulationRunners import Simulation, WindTunnelSimulation

__all__ = [ "main", "run", "BatchRun" ]


class BatchRun():
    ''' Class to hold info about and results of a mapleaf-batch run '''
    def __init__(self, 
            batchDefinition: SimDefinition, 
            recordAll=False, 
            printStackTraces=False, 
            include=None, 
            exclude=None,
            percentErrorTolerance=0.01, 
            resultToValidate=None
        ):
        self.batchDefinition = batchDefinition
        self.recordAll = recordAll
        self.printStackTraces = printStackTraces
        self.include = include
        self.exclude = exclude

        self.casesRun = set()
        self.casesFailed = set()
        self.nTestsOk = 0
        self.nTestsFailed = 0
        self.totalSimErrors = 0
        self.nComparisonSets = 0
        self.casesWithNewRecordedResults = set()

        self.warningCount = 0
        self.percentErrorTolerance = percentErrorTolerance

        self.validationErrors = []
        self.validationDataUsed = []
        self.resultToValidate = resultToValidate

    def getCasesToRun(self):
        subDicts = self.batchDefinition.getImmediateSubDicts("")
        
        if self.include == None and self.exclude == None:
            # Run all cases
            return subDicts

        else:
            # Only run cases that include the include string AND do not contain the exclude string
            casesToRun = []
            for caseDictName in subDicts:
                if (self.include == None or self.include in caseDictName) and (self.exclude == None or self.exclude not in caseDictName):
                    casesToRun.append(caseDictName)

            return casesToRun
    
    def printResult(self, timeToRun=None) -> int:
        """ Outputs result summary """
        print("\n----------------------------------------------------------------------")
        print("BATCH RUN RESULTS")

        if timeToRun != None:
            print("Ran {} Case(s) in {:>.2f} s".format(len(self.casesRun), timeToRun))
        else:
            print("Ran {} Case(s)".format(len(self.casesRun)))

        if self.resultToValidate != None:
            if len(self.validationErrors) > 0:
                print("\nValidation Results for {}:".format(self.resultToValidate))
                print("Average disagreement with validation data across {} validation data sets: {:2.2f}%".format( len(self.validationDataUsed), mean(self.validationErrors)))
                print("Average magnitude of disgreement with validation data across {} validation data sets: {:2.2f}%".format( len(self.validationDataUsed), mean([abs(error) for error in self.validationErrors])))
                print("Data Sets Used:")
                for (dataSet, avgError) in zip(self.validationDataUsed, self.validationErrors):
                    print("{}: {:2.2f}%".format(dataSet, avgError))
                print("")
            else:
                self.warning("\nERROR: No comparison/validation data for {} found. Make sure there is a plot of {} and some comparison data, and that {} is included in the name of those plotting dictionaries\n".format(self.resultToValidate, self.resultToValidate, self.resultToValidate))
  
        if self.warningCount > 0:
            print("Errors/Warnings: {}".format(self.warningCount))

        if len(self.casesWithNewRecordedResults) > 0:
            recordedCaseList = ", ".join(self.casesWithNewRecordedResults)
            print("New expected results were recorded for the following cases: {}".format(recordedCaseList))
            _writeModifiedTestDefinitionFile(self.batchDefinition)

        if len(self.casesFailed) == 0:
            print("{} Case(s) ok".format(len(self.casesRun)))
            print("")
            if self.warningCount == 0:
                print("OK")
            else:
                print("WARNING")

            return 0
        else:
            nCasesFailed = len(self.casesFailed)
            nTests = self.nTestsOk + self.nTestsFailed
            print("{}/{} Case(s) Failed, {}/{} Parameter Comparison(s) Failed".format(nCasesFailed, len(self.casesRun), self.nTestsFailed, nTests))
            print("")
            print("Failed Cases:")
            for case in self.casesFailed:
                print(case)
            print("")
            print("FAIL")

            return 1

    def error(self, caseName, msg: str):
        ''' Currently, errors are used to indicated problems directly related to MAPLEAF simulations '''
        self.warningCount += 1
        self.nTestsFailed += 1
        print(msg)
        self.casesFailed.add(caseName)

    def warning(self, msg: str):
        ''' Currently, warnings are used when errors occur in processes not directly related to MAPLEAF simulations, like loading comparison data '''
        self.warningCount +=1
        print(msg)

#### Command Line Parsing ####
def main(argv=None):    
    # Parse command line arguments
    parser = _buildParser()
    args = parser.parse_args(argv)

    # Load definition file
    from MAPLEAF.Main import findSimDefinitionFile  # Delayed import here to avoid circular imports
    batchDefinitionPath = findSimDefinitionFile(args.batchDefinitionFile)
    batchDefinition = SimDefinition(batchDefinitionPath, defaultDict={}, silent=True)

    include = args.include[0] if len(args.include) > 0 else None
    exclude = args.exclude[0] if len(args.exclude) > 0 else None
    validate = args.validate[0] if len(args.validate) > 0 else None

    # Create batch run object containing settings and results
    batchRun = BatchRun(batchDefinition, args.recordAll, args.printStackTraces, include, exclude, resultToValidate=validate)

    # Run Cases
    return run(batchRun)


#### Main ####
def run(batchRun: BatchRun) -> int:
    ''' Given a batchRun object (of type `BatchRun`), will run all of its test cases, and print a summary of the results '''
    # Track how long running cases takes
    startTime = time.time()

    # Get all the regression test cases
    testCases = batchRun.getCasesToRun()

    for case in testCases:
        _runCase(case, batchRun)
        batchRun.casesRun.add(case)

    runTime = time.time() - startTime
    return batchRun.printResult(runTime)

#### 1. Load / Run Sim ####
def _runCase(caseName: str, batchRun: BatchRun):
    '''
        Runs a single regression tests case, compares the results to the expected results provided, and generates any desired plots.
            If no comparison data is provided, comparison data is recorded

        Inputs:
            
            *caseName:         (string) Name of current case / top-level dictionary  
            *batchDefinition:  (`MAPLEAF.IO.SimDefinition`) Should have the batchDefinition file loaded  

        Outputs:

            Returns:    numTestsOk(Int), numTestsFailed(Int), resultValuesRecorded(Bool)  
                resultValuesRecorded is True if absent ExpectedResults were added to the regressionTestDefinition  
                    Used to remember that an updated regressionTestDefinition shoudl be written to file at the end of all the test cases  
            Modifies:   batchDefinition - records sim results is no expected results are provided  
            Prints:     One line to introduce case, one more line for each expected results  
    '''
    print("\nRunning Case: {}".format(caseName))
    caseDictReader = SubDictReader(caseName, simDefinition=batchRun.batchDefinition)
    
    #### Load Sim definition file ####
    simDefFilePath = caseDictReader.getString("simDefinitionFile")
    simDefinition = SimDefinition(simDefFilePath, silent=True)

    #### Parameter overrides ####
    _implementParameterOverrides(caseName, batchRun.batchDefinition, simDefinition)

    #### Run simulation ####
    # Check whether simulation is a full flight sim or a parameter sweeping simulation
    caseSubDictionaries = caseDictReader.getImmediateSubDicts()
    if caseName + ".ParameterSweep" in caseSubDictionaries:
        logFilePaths = _runParameterSweepCase(batchRun, caseDictReader, simDefinition)
    else:
        logFilePaths = _runFullFlightCase(batchRun, caseDictReader, simDefinition)

    #### Generate/Save plots ####
    if len(logFilePaths) > 0: # Don't generate plots for crashed sims
        # Get all plot subdictionaries, create plot for each one
        plotDicts = caseDictReader.getImmediateSubDicts("PlotsToGenerate")
        for plotDict in plotDicts:
            plotDictReader = SubDictReader(plotDict, simDefinition=batchRun.batchDefinition)
            _generatePlot(batchRun, plotDictReader, logFilePaths)

def _implementParameterOverrides(caseName: str, batchDefinition: SimDefinition, caseSimDefinition: SimDefinition):
    '''
        Runs on each case before running any sims to implement desired modifications to simulation definition files

        Inputs:
            testCase:   (string) name of the current test case / top level dictionary
            batchDefinition:      (SimDefinition) The sim definition object that's loaded/parsed the testDefinitions.mapleaf file
            simDefinition:                  (SimDefinition) The sim definition object that's loaded/parsed the definition file for the current regression testing simulation

        Outputs:
            Modifies: simDefinition, according to the parameter overrides specified in the regression tests definition file
    '''
    #### Load and enact parameter overrides ####
    # Always disable plotting and enable logging
    caseSimDefinition.setValue("SimControl.plot", "None")
    caseSimDefinition.setValue("SimControl.loggingLevel", "3")
    caseSimDefinition.setValue("SimControl.RocketPlot", "Off")

    # Look for other overrides in the definition file
    parameterOverridesDictKey = ".".join([caseName, "ParameterOverrides"])
    parameterOverrides = batchDefinition.getSubKeys(parameterOverridesDictKey)
    for paramOverrideKey in parameterOverrides:
        overridenKey = paramOverrideKey.replace(parameterOverridesDictKey+".", "")
        overrideValue = batchDefinition.getValue(paramOverrideKey)

        # Implement them
        caseSimDefinition.setValue(overridenKey, overrideValue)

def _runParameterSweepCase(batchRun: BatchRun, caseDictReader: SubDictReader, simDefinition: SimDefinition):
    ''' Runs a parameter sweep / wind tunnel simulation, checks+plots results '''
    print("  Parameter Sweep Case")

    # Find dictionaries of expected results & parameter sweeps
    ParametersToSweepOver = []
    expectedResultsDicts = []
    ParameterSweepDicts = caseDictReader.getImmediateSubDicts(caseDictReader.simDefDictPathToReadFrom + ".ParameterSweep")
    for SubDict in ParameterSweepDicts:
        if 'Results' in SubDict: # any subdict in parameterSweep that is NOT a results dict, will be assumed to be a parameter sweep dict
            expectedResultsDicts.append(SubDict)
        else:
            ParametersToSweepOver.append(SubDict)

    # Parse parameter sweep values
    sweptParameters = []
    parameterValues = []
    for parameter in ParametersToSweepOver:
        sweptParameters.append(caseDictReader.getString(parameter + '.sweptParameter'))
        parameterValues.append(caseDictReader.getString(parameter + '.parameterValues'))
    parameterValues = [ _parseParameterSweepValues(valString) for valString in parameterValues ]
    
    # Check whether to add points for smoother plots
    smoothLineDefault = 'True' if len(parameterValues) < 25 else 'False'
    smoothLine = caseDictReader.tryGetString('ParameterSweep.smoothLine', defaultValue=smoothLineDefault)

    # Run simulation
    try:
        simRunner = WindTunnelSimulation(sweptParameters, parameterValues, simDefinition=simDefinition, silent=True, smoothLine=smoothLine)
        logFilePaths = simRunner.runSweep()
    except:
        _handleSimCrash(batchRun, caseDictReader.simDefDictPathToReadFrom)
    else:
        Logging.removeLogger()

    for expectedResultsDict in expectedResultsDicts: # loop through expected results. Manually inputed values, as well as comparisonData in the plots
        expectedResultsCol = caseDictReader.getString(expectedResultsDict + ".column") # get column header that contains results in log files
        expectedResults = caseDictReader.getString(expectedResultsDict + ".expectedValues").split(',') # get expected results values that will be compared against sim
        expectedResults = [ float(x) for x in expectedResults ] # Convert to floatS
            
        ### Get results to be checked ###
        for logPath in logFilePaths:
            columnDataLists, columnNames = Plotting.getLoggedColumns(logPath, expectedResultsCol)
            if len(columnNames) > 0:
                break # Stop looking on first column match

        if len(columnNames) != 1:
            batchRun.error(caseDictReader.simDefDictPathToReadFrom, "   ERROR: Did not find exactly one column matching spec: {} in log files: {}. Instead, found: {} matching columns {}".format(expectedResultsCol, logFilePaths, len(columnNames), columnNames))
            return
        else:
            resultData = columnDataLists[0]

        ### Record / Check Results ###
        if (len(expectedResults) == 1 and expectedResults[0].lower() == "record") or batchRun.recordAll:
            ## Record results ##
            key = expectedResultsDict + ".expectedValues"
            stringResults = ", ".join([ str(x) for x in resultData ])
            batchRun.batchDefinition.setValue(key, value)
            batchRun.casesWithNewRecordedResults.add(caseDictReader.simDefDictPathToReadFrom)

            # Tell user the values have been recorded
            for value in resultData:
                print("  {:<25} Recorded {:>15.7}".format(expectedResultsCol + ":", value))

        else:
            ## Chcek results ##
            resultDataStep = 10 if strtobool(smoothLine) else 1

            for i in range(len(expectedResults)):
                _checkResult(batchRun, caseDictReader.simDefDictPathToReadFrom, expectedResultsCol, resultData[i*resultDataStep], expectedResults[i])

    return logFilePaths

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

def _runFullFlightCase(batchRun: BatchRun, caseDictReader: SubDictReader, simDefinition: SimDefinition):
    ''' Run a regular MAPLEAF simulation based on this case dictionary, checks+plots results '''
    print("  Full Flight Case")
    try:
        simRunner = Simulation(simDefinition=simDefinition, silent=True)
        _, logFilePaths = simRunner.run()
    except:
        _handleSimCrash(batchRun, caseDictReader.simDefDictPathToReadFrom)
    else:
        Logging.removeLogger()

    #### Compare and/or record numerical results from final simulation state, output pass/fail ####
    expectedResultKeys = caseDictReader.getSubKeys("ExpectedFinalValues")

    if len(expectedResultKeys) == 0:
        # If no expected results are provided, record the default set
        _setUpDefaultResultRecording(batchRun, caseDictReader, logFilePaths)
    
    _checkSimResults(batchRun, caseDictReader, logFilePaths, expectedResultKeys)

    return logFilePaths

def _handleSimCrash(batchRun: BatchRun, caseName):
    # Simulation Failed
    Logging.removeLogger() # Make sure we can print to the console
    batchRun.error(caseName, "  ERROR: Simulation Crashed")

    if batchRun.printStackTraces:
        import traceback
        tb = traceback.format_exc()
        print(tb)
        
    return [] # No log file paths

#### 2. Checking Expected Final Results ####
def _setUpDefaultResultRecording(batchRun: BatchRun, caseDictReader: SubDictReader, logFilePaths):
    ''' If no expected results are provided, this adds some default position/velocity values to record for future runs '''
    print("  WARNING: No expected parameter values provided. Recording Position & Velocity values.")
    _incrementWarningCount()

    caseName = caseDictReader.simDefDictPathToReadFrom
    colsToRecord = [ "PositionX", "PositionY", "PositionZ", "VelocityX", "VelocityY", "VelocityZ"]

    for column in colsToRecord:
        batchRun.batchDefinition.setValue(caseName + ".ExpectedFinalValues." + column, "Record" )

def _checkSimResults(batchRun: BatchRun, caseDictReader: SubDictReader, logFilePaths, expectedResultKeys):
    ''' Checks every values in the expected results at end of sim dictionary '''
    for resultKey in expectedResultKeys:
        logColumnSpec = resultKey[resultKey.rfind(".")+1:] # From CaseName.ExpectedFinalValues.PositionX -> PositionX

        try:
            if batchRun.recordAll:
                raise ValueError("Let's record a value")

            ## Regular Parameter Check ##
            expectedResult = caseDictReader.getFloat(resultKey)
            observedResult, columnName = _getSingleResultFromLogs(batchRun, logFilePaths, logColumnSpec)
            _checkResult(batchRun, caseDictReader.simDefDictPathToReadFrom, columnName, observedResult, expectedResult)

        except ValueError:
            ## Record value for this parameter? ##
            expectedResult = caseDictReader.getString(resultKey)
            if expectedResult.lower() == "record" or batchRun.recordAll:
                # Get and save value of parameter from current simulation
                observedValue, colName = _getSingleResultFromLogs(batchRun, logFilePaths, logColumnSpec)
                batchRun.batchDefinition.setValue(resultKey, str(observedValue))
                print("  {:<25} Recorded {:>15.7}".format(colName + ":", observedValue))
                batchRun.casesWithNewRecordedResults.add(caseDictReader.simDefDictPathToReadFrom)
                
            else:
                ## Parsing error ##
                print("  ERROR: Expected value: {} for parameter: {} not numeric or 'Record'".format(expectedResult, resultKey))
                batchRun.warningCount += 1

def _checkResult(batchRun: BatchRun, caseName: str, columnName: str, observedResult: float, expectedResult: float):
    '''
        Checks whether the observed and expected results match to within the desired tolerance

        Inputs:
            logFilePaths:   (List (string)) List of paths to simulation log files
            logColumnSpec:  (string) Partial or Full log column name, or regex expression. Should match exactly 1 log column
            expectedResult: (numeric) Expected value of the data in the column identified by logColumnSpec, in the very last row of data

        Outputs:
            Returns: checkPassed(bool), columnName(string)
            Prints: 1 line, success or failure

    '''    
    if observedResult == None:
        # Could end up here if a result is not found in the log file - perhaps a column name has been mis-spelled in the batch definition file?
        batchRun.nTestsFailed += 1
        batchRun.casesFailed.add(caseName)
    
    else:
        # Compute error %
        if expectedResult != 0:
            error = abs(expectedResult - observedResult)
            errorPercent = abs(error * 100 / expectedResult)
        else:
            errorPercent = 0 if (expectedResult == observedResult) else 100

        # Print + Save Result
        if errorPercent > batchRun.percentErrorTolerance or isnan(errorPercent):
            print("  {:<25} FAIL     {:>15.7}, Expected: {:>15.7}, Disagreement: {:>10.2f} %".format(columnName + ":", observedResult, expectedResult, errorPercent))
            batchRun.nTestsFailed += 1
            batchRun.casesFailed.add(caseName)

        else:
            print("  {:<25} ok       {:>15.7}".format(columnName + ":", expectedResult))
            batchRun.nTestsOk += 1

def _getSingleResultFromLogs(batchRun: BatchRun, logFilePaths, logColumnSpec):
    ''' Returns the last value in the log column defined by logColumn Spec. Searches in each file in logFilePaths '''
    for logPath in logFilePaths:
        dataLists, columnNames = Plotting.getLoggedColumns(logPath, [ logColumnSpec ])

        if len(dataLists) > 1:
            batchRun.warning("  ERROR: Column Spec '{}' matched more than one column: {} in log file: '{}'".format(logColumnSpec, columnNames, logPath))
            return None, logColumnSpec

        if len(dataLists) == 1:
            columnName = columnNames[0]
            observedResult = dataLists[0][-1]
            return observedResult, columnName
    
    # No column was found
    batchRun.warning("  ERROR: Column Spec {} did not match any columns".format(logColumnSpec))
    return None, None

#### 3. Plotting ####
def _generatePlot(batchRun: BatchRun, plotDictReader: SubDictReader, logFilePaths):
    '''
        Called once for every plot dictionary. Handles plotting MAPLEAF's results and any provided comparison data. Saves plot.

        Inputs:
            plotDictReader:     (SubDictReader) Initialized to read from the subdirectory of PlotsToGenerate that defines the desired plot
            logFilePaths:       (list (string)) 

        Outputs:
            Saves png, pdf, and eps plots to the location specified by  [PlotDictionary].saveLocation in the batch definition file
    '''
    # Read info from plotDictReader, create figure, set x/y limits, axes labels, etc...
    fig, ax, columnSpecs, xColumnName, lineFormats, legendLabels, scalingFactor, offset, xLim, yLim = _setUpFigure(plotDictReader)

    #### Plot all the requested data from MAPLEAF's results ####
    mapleafCols = []
    mapleafX = []
    mapleafData = []
    for logFilePath in logFilePaths:
        columnData, columnNames = Plotting.getLoggedColumns(logFilePath, columnSpecs, columnsToExclude=mapleafCols)
        if len(columnNames) > 1:
            # Only plot if we've found (at minimum) an X-column and a Y-column (2 columns)
            adjustX = True if xLim == ["False"] else False
            xData = _plotData(ax, columnData, columnNames, xColumnName, lineFormats, legendLabels, scalingFactor, offset, linewidth=3, adjustXaxisToFit=adjustX)
            
            # Track the x-data for each column of y-data plotted
            for i in range(len(columnNames)):
                mapleafX.append(xData)

            # Avoid plotting columns twice!
            for i in range(len(columnNames)):
                if columnNames[i] != xColumnName:
                    mapleafCols.append(columnNames[i])
                    mapleafData.append(columnData[i])

    #### Plot comparison data ####
    compDataDictionaries = plotDictReader.simDefinition.getImmediateSubDicts(plotDictReader.simDefDictPathToReadFrom)
    for compDataDict in compDataDictionaries:
        compDataDictReader = SubDictReader(compDataDict, plotDictReader.simDefinition)
        valData, valCols, valX = _plotComparisonData(batchRun, ax, compDataDictReader)
        validationData = compDataDictReader.tryGetBool("validationData", defaultValue=True)

        if batchRun.resultToValidate != None:
            # Check whether we should validate this graph
            dictNameMatchesValidation = (batchRun.resultToValidate in compDataDict and len(valCols) == 1)
            columnNameMatchesValidation = (len(valCols) == 1 and batchRun.resultToValidate in valCols[0])
            mapleafColumnNameMatchesValidation = (len(mapleafCols) == 1 and batchRun.resultToValidate in mapleafCols[0])
            dataShouldBeUsedForCurrentValidation = validationData and any([dictNameMatchesValidation, columnNameMatchesValidation, mapleafColumnNameMatchesValidation])
            dataExists = len(valCols) > 0
            
            if dataShouldBeUsedForCurrentValidation and dataExists:
                _validate(batchRun, mapleafX, mapleafData, valData, valX, compDataDict)
    
    #### Finalize + Save Plot ####  
    if yLim == ["False"]:
        ax.autoscale(axis='y', tight=True)
    
    ax.legend()
    fig.tight_layout()

    # Get save location
    saveFilePath = plotDictReader.getString("saveLocation")
    saveDirectory = os.path.dirname(saveFilePath)
    saveFileName = os.path.basename(saveFilePath)
    overwrite = plotDictReader.tryGetBool("overwrite", defaultValue=True)

    # Save plot
    gridConvergenceFunctions.saveFigureAndPrintNotification(saveFileName, fig, saveDirectory, overwrite=overwrite, epsVersion=False, pngVersion=False, printStatementPrefix="  ")
    plt.close(fig) # Close figure to avoid keeping them all in memory (Matplotlib gives warning about this - thank you Matplotlib developers!)

def _setUpFigure(plotDictReader: SubDictReader):
    # Create plot
    fig, ax = plt.subplots(figsize=(6,4))

    #### Plot Data from current simulation ####
    # Get all entries in the PlotsToGenerate dictionary
    columnSpecs = plotDictReader.tryGetString("columnsToPlot", defaultValue="").split()
    nLinesToPlot = len(columnSpecs)
    if nLinesToPlot == 0:
        return

    # Make sure we're set to get the time/x column along with the y-data
    xColumnName = plotDictReader.tryGetString("xColumnName", defaultValue="Time(s)")
    if xColumnName not in columnSpecs:
        columnSpecs.append(xColumnName)

    lineFormats = plotDictReader.tryGetString("lineFormat", defaultValue="y--").split()
    while len(lineFormats) < nLinesToPlot:
        lineFormats.append("")

    legendLabels = plotDictReader.tryGetString("legendLabel", defaultValue=columnSpecs[0]).split(',')
    while len(legendLabels) < nLinesToPlot:
        legendLabels.append(columnSpecs[len(legendLabels)])

    scalingFactor = plotDictReader.tryGetFloat("scalingFactor", defaultValue=1.0)
    offset = plotDictReader.tryGetFloat('offset', defaultValue=0.0)

    ### Set Axes Limits
    xLim = plotDictReader.tryGetString("xLimits", defaultValue="False").split() # Expected length: 2
    if xLim[0] != "False":
        xLowerLim = float(xLim[0])
        xUpperLim = float(xLim[1])
        ax.set_xlim([xLowerLim,xUpperLim])
    yLim = plotDictReader.tryGetString("yLimits", defaultValue="False").split() # Expected length: 2
    if yLim[0] != "False":
        yLowerLim = float(yLim[0])
        yUpperLim = float(yLim[1])
        ax.set_ylim([yLowerLim,yUpperLim])
    
    # Set x and y labels
    xLabel = plotDictReader.tryGetString("xLabel", defaultValue=xColumnName)
    yLabel = plotDictReader.tryGetString("yLabel", defaultValue=columnSpecs[0])
    ax.set_xlabel(_latexLabelTranslation(xLabel))
    ax.set_ylabel(_latexLabelTranslation(yLabel))
    
    return fig, ax, columnSpecs, xColumnName, lineFormats, legendLabels, scalingFactor, offset, xLim, yLim

def _plotComparisonData(batchRun: BatchRun, ax, compDataDictReader):
    ''' Plot a single line of comparison data from a specified .csv file '''
    # Get line formatting info
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
                batchRun.warning("  ERROR: Found {} columns of comparison data: {} for {} column specs: {} in file: {}".format(len(compColData), compColNames, len(compColumnSpecs), compColumnSpecs, compDataPath))

            xData = _plotData(ax, compColData, compColNames, xColumnName, lineFormat, legendLabel, scalingFactor)
            return compColData, compColNames, xData

        except FileNotFoundError:
            batchRun.warning("  ERROR: Comparison data file: {} not found".format(compDataPath))

    else:
        batchRun.warning("  ERROR: Locating comparison data, file: {}, columns to plot: {}".format(compDataPath, compColumnSpecs))

    return [], [], xColumnName

def _plotData(ax, dataLists, columnNames, xColumnName, lineFormat, legendLabel, scalingFactor, offset=0, linewidth=1.5, adjustXaxisToFit=False):
    '''
        Adds MAPLEAF's results to the plot currently being created

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
        ax.set_xlim([xData[0], xData[-1]])

    # Scale data and apply offset:
    for i in range(len(dataLists)):
        for j in range(len(dataLists[i])):
            a = dataLists[i][j]
            dataLists[i][j] = scalingFactor*float(dataLists[i][j]) + offset

    # Plot data
    for i in range(len(columnNames)):
        if len(xData) > 1:
            # Line
            ax.plot(xData, dataLists[i], lineFormat[i], linewidth=linewidth, label=legendLabel[i])
        else:
            # Point
            ax.scatter(xData, dataLists[i], linewidth=linewidth, label=legendLabel[i])

    return xData

def _validate(batchRun: BatchRun, mapleafX, mapleafData, valData, validationX, validationDataPath: str) -> Union[float, None]:
    '''
        Returns the average percentage disagreement between the mapleaf results and the validation data

        Inputs:
            mapleafX:           (List[List[float]]) Mapleaf X-data
            mapleafData:        (List[List[float]]) Mapleaf data for each of the column names in mapleafCols (order should match)
            
            valData:            (List[List[float]]) Comparison data for each of the column names in valCols (order should match), also includes x-column data
            validationX:        (List[float]) x-column data for the values in valData

            validationDataPath: (str) Used to track the source of the data used

        Outputs:
            Computes average disagreement b/w linearly-interpolated mapleaf data and validation data, saves it in the batchRun object
    '''
    if len(mapleafX) != len(mapleafData):
        batchRun.warning("  ERROR: Can't validate data without matching number of X and Y MAPLEAF data sets. Current validation data set: {}".format(validationDataPath))
        return

    def getAvgError(MAPLEAFX, MAPLEAFY, valX, valY) -> float:
        def getInterpolatedMAPLEAFResult(x):
            # Interpolating MAPLEAF's results because we are assuming MAPLEAF's data is often denser than validation data, which decreases interpolation error
            return linInterp(MAPLEAFX, MAPLEAFY, x)
        
        interpolatedMAPLEAFResults = [ getInterpolatedMAPLEAFResult(x) for x in validationX ]

        # TODO: Provide/plot error distributions, not just averages?
        errorMagnitudes = [ (mY - vY)  for  (mY, vY)  in  zip(interpolatedMAPLEAFResults, valY) ]
        errorPercentages = [ ((error / vY) if vY != 0 else 100)  for  (error, vY)  in  zip(errorMagnitudes, valY) ]
        
        return mean(errorPercentages)

    if len(mapleafData) == 1 and len(valData) == 1:
        # One set of mapleaf data, one set of comparison data -> straightforward
        avgError = getAvgError(mapleafX[0], mapleafData[0], validationX, valData[0])
    
    elif len(mapleafData) == 1 and len(valData) > 1:
        # One set of mapleaf data, multiple sets of comparison data -> compare each to the mapleaf data, return mean error across all curves
        avgErrors = [ getAvgError(mapleafX[0], mapleafData[0], validationX, validationY) for validationY in valData ]
        avgError = mean(avgErrors)

    elif len(mapleafData) > 1 and len(valData) == 1:
        # Multiple sets of mapleaf data, one set of comparison data -> compare comparison data to the mapleaf line that matches it most closely
        avgErrors = [ getAvgError(mapleafX[i], mapleafData[i], validationX, valData[0]) for i in range(len(mapleafData)) ]
        avgError = min(avgErrors)

    else:
        batchRun.warning("  WARNING: Unclear which set of MAPLEAF results should be validated by which set of comparison data")
        avgError = None

    if avgError != None:
        batchRun.validationDataUsed.append(validationDataPath)
        batchRun.validationErrors.append(avgError*100)


#### Utility functions ####
def _writeModifiedTestDefinitionFile(batchDefinition: SimDefinition):
    ''' If new expected final values were recorded during the present batch run, this function will be called to write those values to a new file, [originalFileName]_newExpectedResultsRecorded.mapleaf '''
    origFilePath = batchDefinition.fileName
    newTestDefinitionPath = origFilePath.replace(".mapleaf", "_newExpectedResultsRecorded.mapleaf")

    print("Writing new testDefinition file to: {}".format(newTestDefinitionPath))
    print("  If desired, use this file (or values from this file) to replace/update testDefinitions.mapleaf\n")

    batchDefinition.writeToFile(newTestDefinitionPath, writeHeader=False)

def _latexLabelTranslation(labelInput: str) -> str:
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

def _buildParser() -> argparse.ArgumentParser:
    ''' Builds the argparse parser for command-line arguments '''
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description="""
    Batch-run MAPLEAF simulations.
    Expects batch run to be defined by a batch definition file like MAPLEAF/Examples/Simulations/regressionTests.mapleaf
    See ./batchRunTemplate.mapleaf for definition of all possible options.
    """)

    parser.add_argument(
        "--recordAll", 
        action='store_true', 
        help="If present, re-records all expected results for cases that are run. Recorded data outputted to [batchDefinitionFile]_newExpectedResultsRecorded.mapleaf"
    )
    parser.add_argument(
        "--printStackTraces", 
        action='store_true', 
        help="If present, stack traces are printed for crashed simulations"
    )
    parser.add_argument(
        "--include", 
        nargs=1, 
        default=[], 
        help="Only cases whose name includes this string will be run."
    )
    parser.add_argument(
        "--exclude",
        nargs=1,
        default=[],
        help="Exclude cases whose name includes this string. Takes precedence over --include"
    )
    parser.add_argument(
        "--validate",
        nargs=1,
        default=[],
        help="The average disagreement between MAPLEAF's results and plotted comparison data will be computed for the parameter provided. Parameter must be found in one or more of: a) name of comparison data dictionary name, b) comparison data column name, c) the MAPLEAF column name."
    )
    parser.add_argument(
        "batchDefinitionFile", 
        nargs='?', 
        default="MAPLEAF/Examples/Simulations/regressionTests.mapleaf", 
        help="Path to a batch definition (.mapleaf) file. Default = MAPLEAF/Examples/Simulations/regressionTests.mapleaf"
    )

    return parser



if __name__ == "__main__":
    main()
