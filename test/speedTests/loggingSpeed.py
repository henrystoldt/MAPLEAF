from timeit import timeit

nTests=1000

print("Each operation performed {} times".format(nTests))


print("\nString Logging")

setup = '''
from MAPLEAF.IO.Logging import Logger

log = []
logger = Logger(log, continueWritingToTerminal=False)

time = [ 0 ]

def logLine(times):
    times[0] += 1
    logString = '{} {} {}'.format(times[0], 0, 0)
    logString += ' {} {} {}'.format(0, 0, 0)
    logString += ' {} {}'.format(0, 0)
    logger.write(logString)
    
'''

print(timeit("logLine(time)", setup=setup, number=nTests))





print("\nPython Logging Module")
setup = '''
import logging
logger = logging.getLogger("testLogger")

time = [ 0 ]

def logLine(times):
    times[0] += 1
    logger.info('{} {} {} {} {} {} {} {}'.format(times[0], 0, 0, 0, 0, 0, 0, 0))
'''

print(timeit("logLine(time)", setup=setup, number=nTests))



print("\nCython Log class")

setup = '''
from MAPLEAF.IO.CythonLog import Log

log = Log()
logCols = log.addColumns(["X", "Y", "Z", "1", "2", "3", "4"])

time = [ 0 ]

def logLine(times):
    times[0] += 1
    log.newLogRow(times[0])
    for col in logCols:
        col.append(0)
'''

print(timeit("logLine(time)", setup=setup, number=nTests))




print("\n Pandas Dataframe")

setup = '''
import pandas as pd

cols = [ "X", "Y", "Z", "1", "2", "3", "4"]
log = pd.DataFrame(columns=cols)

time = [ 0 ]

def logLine(times):
    times[0] += 1
    for col in cols:
        log.loc[times[0]] = [ 0, 0, 0, 0, 0, 0, 0 ]
'''

print(timeit("logLine(time)", setup=setup, number=nTests))

