import csv
from bisect import bisect_left
from MAPLEAF.Motion import Vector

__all__ = [ "Log", "TimeStepLog" ]

# TODO: Log class should replace Logger and RocketFlight objects
# TODO: Log class should be able to re-populate itself from .csv file


cdef class Log():
    '''
        Class manages logs for any number of states that need to be logged.
        Each state/parameter that needs to be logged is given its own column in the log (represented as a python list)
        Each row holds values from a single time.
        When a new row is added to the log, any empty values in the previous row will be filled with the fill value.

        Internally, the log is represented as a dict containing a list for each log column
    '''
    cdef public dict logColumns
    cdef public dict fillValues

    def __init__(self, columnNames=None, defaultFillValue=0):
        ''' 
            columnNames should be a list of strings 
            To get references to the log columns, initialize the log as empty and then use the addColumn(s) functions.
                Those return the list(s) they add to the log for direct access
        '''
        self.logColumns = {}
        self.fillValues = {}
        self.fillValues["default"] = defaultFillValue

        # Each column starts out as an empty list
        if columnNames != None:
            for colName in columnNames:
                if " " in colName:
                    # TODO: Start writing logs in csv format so spaces will be allowed
                    raise ValueError("Log column names must not contain spaces. Name with space: {}".format(colName))

                self.logColumns[colName] = []

        # There is always a time column
        self.logColumns["Time(s)"] = []

    cpdef _completeLastLine(self):
        ''' Fill empty values in the current last row with self.fillValue. Called right before creating a new line '''
        cdef int expectedNVals = len(self.logColumns["Time(s)"])
        cdef int nVals

        for col in self.logColumns:
            nVals = len(self.logColumns[col])
            if nVals == expectedNVals:
                continue
            elif nVals < expectedNVals:
                try:
                    self.logColumns[col].append(self.fillValues[col])
                except KeyError:
                    self.logColumns[col].append(self.fillValues["default"])
            else:
                raise ValueError("More values than expected in log column: {} at time step: {}".format(col, self.logColumns["Time(s)"][-1]))

    cpdef newLogRow(self, currentTime):
        ''' Start a new row in the log. Typically called after a time step or force evaluation is completed '''
        # Fill empty values in the previous row with the fill value
        self._completeLastLine()

        # Check that time is always increasing (assumed by the getValue function, enables binary search for indices)
        if len(self.logColumns["Time(s)"]) != 0 and currentTime < self.logColumns["Time(s)"][-1]:
            lastTime = self.logColumns["Time(s)"][-1]
            raise ValueError("Times in log rows must be increasing sequentially. New Row Time: {}, Last Row Time: {}".format(currentTime, lastTime))

        # Create new row
        self.logColumns["Time(s)"].append(currentTime)

    cpdef deleteLastRow(self):
        nVals = len(self.logColumns["Time(s)"])

        if nVals > 0:
            for col in self.logColumns:
                # If column has as many items in it as expected, remove the last one
                # Don't do complete error checking here, assume that will be done in self.newLogRow()
                if len(self.logColumns[col]) == nVals:
                    self.logColumns[col].pop()

    cpdef addColumn(self, colName, fillValue=None):
        ''' Returns reference to the log column (list) '''
        if colName in self.logColumns:
            raise ValueError("Column {} already exists".format(colName))

        newCol = []
        self.logColumns[colName] = newCol

        if fillValue is not None:
            self.fillValues[colName] = fillValue

        # Add fillValue for any existing rows
        nRows = len(self.logColumns["Time(s)"])
        if fillValue is not None:
            for i in range(nRows):
                newCol.append(fillValue)
        else:
            for i in range(nRows):
                newCol.append(self.fillValues["default"])

        return newCol

    cpdef addColumns(self, colNames):
        ''' Returns list of references to the log columns (list[list]) '''
        newCols = []
        for colName in colNames:
            newCol = self.addColumn(colName)
            newCols.append(newCol)
        
        return newCols        
    
    cpdef logValue(self, colName, value):
        ''' Throws KeyError if colName is not a valid column name '''
        self.logColumns[colName].append(value)

    cpdef getValue(self, time, colName):
        # Binary search for the correct time
        desiredRow = bisect_left(self.logColumns["Time(s)"], time)

        # Retrieve value
        return self.logColumns[colName][desiredRow]

    cpdef expandIterableColumns(self):
        ''' Look for columns containing vector values, convert them into a triplet of values '''
        columnNames = list(self.logColumns.keys())
        
        for column in columnNames:
            if len(self.logColumns[column]) == 0:
                continue

            sampleItem = self.logColumns[column][0]
            if isinstance(sampleItem, Vector):
                # Expand the column into three (x,y,z components)
                # First compute the new column names
                columnSuffixes = [ 'X', 'Y', 'Z' ]
            else:
                try:
                    if isinstance(sampleItem, str):
                        continue

                    testIterator = iter(sampleItem)
                    columnSuffixes = [ str(x) for x in range(len(sampleItem)) ]

                except TypeError:
                    # Item is not iterable, skip
                    continue

                
            for i in range(3):
                if '(' in column:
                    newColumnName = column.replace('(', columnSuffixes[i]+'(')
                else:
                    newColumnName = column + columnSuffixes[i]
                
                self.logColumns[newColumnName] = [ x[i] for x in self.logColumns[column] ]
            
            # Remove the original column
            self.logColumns.pop(column)


    cpdef writeToCSV(self, fileName):
        # Fill any unfilled values on last line
        # Returns true if a file is written, false if it is not (empty log)
        self._completeLastLine()
        self.expandIterableColumns()

        nRows = len(self.logColumns["Time(s)"])

        if nRows > 0:
            print("Writing log file: " + fileName)

            with open(fileName, 'w', newline='') as file:
                writer = csv.writer(file)
                
                # Print out headers
                colNames = sorted(self.logColumns.keys())
                colNames.remove("Time(s)")
                colNames.insert(0, "Time(s)")
                writer.writerow(colNames)

                for i in range(nRows):
                    # Assemble the row
                    row = [ self.logColumns[col][i] for col in colNames ]
                    writer.writerow(row)

            return True
        
        return False

cdef class TimeStepLog(Log):
    '''
        Adds functionality for post processing specific to time step logs for mapleaf
    '''
    
    cpdef _calculateTimeStepSizes(self):
        timeStepSizes = []
        times = self.logColumns["Time(s)"]

        nRows = len(times)
        if nRows > 1:
            for i in range(nRows-1):
                # Calculate the time step sizes
                timeStepSizes.append(times[i+1] - times[i])

            # Make the time step column as long as the others by adding a fake last value (repeat the second last value)
            timeStepSizes.append(timeStepSizes[-1])

        self.logColumns["TimeStep(s)"] = timeStepSizes

    # TODO: Compute Euler angles as a post processing step as well?

    cpdef writeToCSV(self, fileName):
        self._calculateTimeStepSizes()
        return Log.writeToCSV(self, fileName)