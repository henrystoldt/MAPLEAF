import csv
from bisect import bisect_left

__all__ = [ "Log" ]

# TODO: Log class should replace Logger and RocketFlight objects
# TODO: Log class should be able to re-populate itself from .csv file
# TODO: need to determine exactly which objects should own log files
    # a single log for a time steps. or a single log for each rocket stage


cdef class Log():
    '''
        Class manages logs for any number of states that need to be logged.
        Each state/parameter that needs to be logged is given its own column in the log
        Each row holds values from a single time.
        When a new row is added to the log, any empty values in the previous row will be filled with the fill value.

        Internally, the log is represented as a dict containing a list for each log column
    '''
    cdef public dict logColumns
    cdef public double fillValue

    def __init__(self, columnNames=None, fillValue=0):
        ''' 
            columnNames should be a list of strings 
            To get references to the log columns, initialize the log as empty and then use the addColumn(s) functions.
                Those return the list(s) they add to the log for direct access
        '''
        self.logColumns = {}
        self.fillValue = fillValue

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
        expectedNVals = len(self.logColumns["Time(s)"])
        for col in self.logColumns:
            nVals = len(self.logColumns[col])
            if nVals == expectedNVals:
                continue
            elif nVals < expectedNVals:
                self.logColumns[col].append(self.fillValue)
            else:
                raise ValueError("More values than expected in log column: {} at time step: {}".format(col, self.logColumns["Time(s)"][-1]))

    cpdef newLogRow(self, currentTime):
        ''' Start a new row in the log. Typically called after a time step or force evaluation is completed '''
        # Fill empty values in the previous row with the fill value
        self._completeLastLine()

        # Check that time is always increasing (assumed by the getValue function, enables binary search for indices)
        if len(self.logColumns["Time(s)"]) != 0 and currentTime <= self.logColumns["Time(s)"][-1]:
            lastTime = self.logColumns["Time(s)"][-1]
            raise ValueError("Times in log rows must be increasing sequentially. New Row Time: {}, Last Row Time: {}".format(currentTime, lastTime))

        # Create new row
        self.logColumns["Time(s)"].append(currentTime)

    cpdef deleteLastRow(self):
        nVals = len(self.logColumns["Time(s)"])

        for col in self.logColumns:
            # If column has as many items in it as expected, remove the last one
            # Don't do complete error checking here, assume that will be done in self.newLogRow()
            if len(self.logColumns[col]) == nVals:
                self.logColumns[col].pop()

    cpdef addColumn(self, colName):
        ''' Returns reference to the log column (list) '''
        newCol = []
        self.logColumns[colName] = newCol

        # Add fillValue for any existing rows
        nRows = len(self.logColumns["Time(s)"])
        for i in range(nRows):
            newCol.append(self.fillValue)

        return newCol

    cpdef addColumns(self, colNames):
        ''' Returns list of references to the log columns (list[list]) '''
        newCols = []
        for colName in colNames:
            newCol = self.addColumn(colName)
            newCols.append(newCol)
        
        return newCols        
    
    cpdef logValue(self, colName, value):
        self.logColumns[colName].append(value)

    cpdef getValue(self, time, colName):
        # Binary search for the correct time
        desiredRow = bisect_left(self.logColumns["Time(s)"], time)

        # Retrieve value
        return self.logColumns[colName][desiredRow]

    cpdef writeToCSV(self, fileName):
        # Fill any unfilled values on last line
        self._completeLastLine()

        with open(fileName, 'w', newline='') as file:
            writer = csv.writer(file)
            
            # Print out headers
            colNames = sorted(self.logColumns.keys())
            colNames.remove("Time(s)")
            colNames.insert(0, "Time(s)")
            writer.writerow(colNames)

            nRows = len(self.logColumns["Time(s)"])
            for i in range(nRows):
                # Assemble the row
                row = [ self.logColumns[col][i] for col in colNames ]
                writer.writerow(row)

