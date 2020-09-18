'''
Classes and functions for creating simulation logs for regular simulations (Logger) and Monte Carlo simulations (MonteCarloLogger)
'''

import os
import sys

# TODO: When logging, keep track of messages containing 'error' or 'warning' -> reprint those at the end of the simulation?

class Logger():
    '''
        Class intended to capture calls to print() and copy their contents to a list of strings, while still (optionally) printing them to the console

        Ex:
            logger = Logger(stringResultList)
            sys.stdout = logger

        Now anything passed into print() will be printed to the console and stored in stringResultArray
    '''

    def __init__(self, stringListToCopyTo, continueWritingToTerminal=True):
        self.terminal = sys.__stdout__
        self.log = stringListToCopyTo
        self.currentMessage = ""
        self.continueWritingToTerminal = continueWritingToTerminal

    def write(self, msg):
        if self.continueWritingToTerminal:
            self.terminal.write(msg)
        self.log.append(msg)

    def flush(self):
        self.terminal.flush()

    def writeLine(self, msg=None):
        if msg == None:
            msg = self.currentMessage + "\n"
            self.currentMessage = ""

        if self.continueWritingToTerminal:
            self.terminal.write(msg)
        self.log.append(msg)

    def addToLine(self, msg):
        self.currentMessage += msg

    def changeLoggingTarget(self, newTarget):
        self.log = newTarget

    def getvalue(self):
        return self.terminal.getvalue()

    def writeLogToFile(self, filePath, overwrite=False):
        if overwrite or not os.path.exists(filePath):
            with open(filePath, 'w+') as file:
                file.writelines(self.log)

class MonteCarloLogger():
    '''    log function write lines to the console/mainSimulation Log, and to the monteCarloLog    '''
    def __init__(self, monteCarloLog=None):
        if monteCarloLog == None:
            self.monteCarloLog = []
        else:
            self.monteCarloLog = monteCarloLog
        self.monteCarloLog += getSystemInfo()

    def log(self, string):
        self.monteCarloLog.append(string)
        print(string)

    def writeToFile(self, fileBaseName="monteCarloLog", filePath=None):
        '''
            Pass in fileBaseName OR filePath.
            If a filePath is provided, the file will be written to that path. Anything already there will be overwritten.
            If a fileBaseName is provided and no filePath is provided, the log will be written to a fileName found by calling findNextAvailableFileName(), no files will be overwritten.
        '''
        if filePath == None:
            filePath = findNextAvailableNumberedFileName(fileBaseName=fileBaseName)

        with open(filePath, 'w+') as file:
            for line in self.monteCarloLog:
                if len(line) == 0 or line[-1] != "\n":
                    line = line + "\n"
                file.write(line)

        return filePath

def removeLogger():
    sys.stdout = sys.__stdout__

def findNextAvailableNumberedFileName(fileBaseName="monteCarloLog", extension=".txt"):
    ''' 
        If fileBaseName is simLog, returns the first of: simLog1, simLog2, simLog3, etc... that isn't already a file.
        Returns a string of the form fileBaseName + Number + extension
    '''
    fileNumber = 0
    filePath = None
    while filePath == None or os.path.exists(filePath):
        fileNumber += 1
        filePath = fileBaseName + str(fileNumber) + extension

    return filePath

def getSystemInfo(printToConsole=False):
    ''' Returns string array containing info about github status, machine type, date, etc... '''
    
    from datetime import datetime
    from subprocess import check_output
    from platform import platform
    from os import getlogin
    
    result = []

    try:
        # Add current git status
        currentCommit = check_output(['git', 'rev-parse', 'HEAD']).decode()[:-1]
        currentBranch = check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).decode()[:-1]
        result.append("# MAPLEAF, branch: {}, latest commit: {}".format(currentBranch, currentCommit))
    except:
        result.append("# ERROR: Could not obtain current branch/commit info from git. Ensure command-line version of git is installed: https://git-scm.com/downloads")
    
    # Add date/time
    now = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    result.append("# {}".format(now))

    # Platform info
    try:
        user = getlogin()
        result.append("# User: {}".format(user))    
        operatingsystem = platform()
        result.append("# OS: {}".format(operatingsystem))
    except OSError:
        result.append("Unable to run os.getInfo() - raising OSError")
        pass # Probably running on a platform like Github Actions, which doesn't allow this command

    if printToConsole:
        for line in result:
            print(line)

    return result

def getSimDefinitionAndDefaultValueDictsForOutput(simDefinition, printToConsole=True):
    ''' Returns a string array '''

    stringResultArray = []

    # Add config file path
    print("# Using sim definition file: {}".format(simDefinition.fileName))
    
    # Add config file
    stringResultArray.append("\n---- Start Sim Definition File ----\n")
    stringResultArray += str(simDefinition)
    stringResultArray.append("\n---- End Sim Definition File ----\n\n")
    
    # Add default value dict
    from pprint import pformat
    from MAPLEAF.IO import defaultConfigValues
    
    stringResultArray.append("\n---- Start Default Value Dictionary ----\n")
    stringResultArray.append(pformat(defaultConfigValues))
    stringResultArray.append("\n---- End Default Value Dictionary ----\n\n")

    if printToConsole:
        for line in stringResultArray:
            print(line)

    return stringResultArray

def postProcessForceEvalLog(logFilePath, refArea=1, refLength=1):
    '''
        Calculates additional columns from those already present in the force evaluation log, writes to new file.

        Inputs:
            logFilePath:    (string) path to a forceEvaluationLog
            refArea:        (numeric) m^2 area - used to calculate force/moment coefficients - traditionally the rocket's cross-sectional area
            refLength:      (numeric) m length - used to calculate moment coefficients - traditionally the rocket's diameter

        Outputs:
            Writes new file to logFilePath_expanded.csv
    '''
    #TODO: Logged moments are defined about the CG, which can move. Need to compensate for this
    # Import statements here to avoid making this whole module dependent on pandas & numpy
    import pandas as pd
    import numpy as np
    import MAPLEAF.IO.Plotting as Plotting

    print("Post Processing (Calculating aerodynamic coefficients) log file: {}".format(logFilePath))

    # Load log file into a dataframe
    with open(logFilePath, 'r') as file:
        lines = file.readlines()
    skipRows, skipFooter = Plotting._getSkipRowsAndFooters_forceEvalLog(lines)
    forcesLog = pd.read_csv(logFilePath, sep="\s+", skiprows=skipRows, dtype=np.float64, skipfooter=skipFooter, engine='python')

    # Check if the file is already post-processed
    if "AeroCFX(N)" in forcesLog.columns:
        raise ValueError("Log file: {} already contains 'AeroCFX(N)'. Has likely already been post-processed.".format(logFilePath))

    # Compute airspeed components
    for dir in [ 'X', 'Y', 'Z' ]:
        forcesLog['AirVel{}(m/s)'.format(dir)] = forcesLog['Velocity{}(m/s)'.format(dir)] - forcesLog['Wind{}(m/s)'.format(dir)]

    # Compute square of the airspeed magnitude, the airspeed magnitude, and the air speed unit vector components
    forcesLog['AirVel^2'] = forcesLog['AirVelX(m/s)']**2 + forcesLog['AirVelY(m/s)']**2 + forcesLog['AirVelZ(m/s)']**2
    forcesLog['AirVelMag'] = forcesLog['AirVel^2']**0.5
    forcesLog['AirVelUnitX'] = forcesLog['AirVelX(m/s)'] / forcesLog['AirVelMag']
    forcesLog['AirVelUnitY'] = forcesLog['AirVelY(m/s)'] / forcesLog['AirVelMag']
    forcesLog['AirVelUnitZ'] = forcesLog['AirVelZ(m/s)'] / forcesLog['AirVelMag']

    forcesLog['DynamicP(Pa)'] = forcesLog['AirVel^2'] * forcesLog['AirDensity(kg/m^3)'] * 0.5

    forcesLog['nonDimConstant'] = forcesLog['DynamicP(Pa)'] * refArea

    # Compute force coefficients
    for col in forcesLog.columns:
        isForcesColumn = "FX(N)" in col or "FY(N)" in col or "FZ(N)" in col
        isMomentColumn = "MX(Nm)" in col or "MY(Nm)" in col or "MZ(Nm)" in col
        if isForcesColumn:
            # Avoid simply replacing 'F' with 'CF' in case names contain 'F' elsewhere
            coordDir = col[-4]
            newColName = col.replace("F{}(N)".format(coordDir), "CF{}".format(coordDir))
            # CFX = FX / (qA)
            forcesLog[newColName] = forcesLog[col] / forcesLog['nonDimConstant']

        elif isMomentColumn:
            coordDir = col[-5]
            newColName = col.replace("M{}(Nm)".format(coordDir), "CM{}".format(coordDir))
            # CMX = MX / (qAL)
            forcesLog[newColName] = forcesLog[col] / ( forcesLog['nonDimConstant'] * refLength)

    # Compute lift, drag, and normal force coefficients
    # Note that motor thrust is included in these calculations - so the coefficients are only truly accurate when the motor is off (Especially Cd)
        # Drag force component is the component aligned to the flow (localAirVel): (AeroForce \dot localAirvel) / (q * A_ref)
    forcesLog['dragForce'] = forcesLog["AeroFX(N)"]*forcesLog["AirVelUnitX"] + forcesLog["AeroFY(N)"]*forcesLog["AirVelUnitY"] + forcesLog["AeroFZ(N)"]*forcesLog["AirVelUnitZ"]
    forcesLog['Cd'] = -1*forcesLog['dragForce'] / forcesLog['nonDimConstant']
        # Lift force is the component perpendicular to the drag force
    forcesLog['liftForce'] = (forcesLog["AeroFX(N)"]**2 + forcesLog["AeroFY(N)"]**2 + forcesLog["AeroFZ(N)"]**2 - forcesLog["dragForce"]**2)**0.5
    forcesLog['Cl'] = forcesLog['liftForce'] / forcesLog['nonDimConstant']
        # Normal force is X and Y components combined
    forcesLog['normalForce'] = (forcesLog["AeroFX(N)"]**2 + forcesLog["AeroFY(N)"]**2)**0.5
    forcesLog['CN'] = forcesLog['normalForce'] / forcesLog['nonDimConstant']

    # Drop intermediate columns used in calculations, but not required to be outputted
    columnsToDrop = [
        "AirVel^2",
        "AirVelMag",
        "AirVelUnitX",
        "AirVelUnitY",
        "AirVelUnitZ",
    ]
    forcesLog.drop(columns=columnsToDrop)

    # Write expanded log to file
    newLogFilePath = logFilePath.replace(".txt", "_expanded.txt")
    forcesLog.to_csv(newLogFilePath, sep='\t', index=False)

    print("Writing expanded level 3 log to: {}".format(newLogFilePath))

    return newLogFilePath

