"""
    This file generates python modules in ./MAPLEAF/V&V which will be turned into verification and validation website by pdoc3 every time the documentation is built
    One module/webpage is generated per folder in ./test/V&V/. All .pdf files in the folders are displayed on the page.
"""
import os
from pathlib import Path
import shutil
import sys

from MAPLEAF.IO import SimDefinition
from MAPLEAF.IO.Logging import Logger, removeLogger
from MAPLEAF.SimulationRunners.Batch import BatchRun, run


try:
    shutil.rmtree('./MAPLEAF/V&V')
except:
    pass

# Load the simulation definition file that defines all of the regression verification and validation cases
batchDefinition = SimDefinition('./MAPLEAF/Examples/BatchSims/regressionTests.mapleaf')
batchRun = BatchRun(batchDefinition)

# Run all the cases to generate the result pdfs
run(batchRun)
resultSummary = []
sys.stdout = Logger(resultSummary)
batchRun.printResult()
removeLogger()

# Create the verification and validation folder, make it a python module
MAPLEAFPath = Path(__file__).parent.parent.parent.absolute()
fakeModuleDirectory = MAPLEAFPath / 'MAPLEAF' / 'V&V'
regressionTestingDirectory = MAPLEAFPath / 'test' / 'V&V'

os.mkdir(fakeModuleDirectory)
with open(regressionTestingDirectory / '__init__.py', "r") as f:
    templateText = f.read()

with open(fakeModuleDirectory / "__init__.py", "w+") as f:
    indentedSummary = ''.join([ '    ' + x for x in resultSummary ])
    text = templateText.replace('{INSERT RESULTS HERE}', '\n\n## Console Output:  ' + indentedSummary)
    f.write(text)


# Create all of the submodules, one for each case
for caseResult in batchRun.casesRun:    
    newFolderPath = fakeModuleDirectory / caseResult.name
    os.mkdir(newFolderPath)

    initPath = newFolderPath / '__init__.py'
    with open(initPath, 'w+') as f:
        lines = [ '"""' ]

        # We can only display .png plots, ignore the rest
        plots = []
        for plotPath in caseResult.plotPaths:
            if '.png' in plotPath:
                plots.append(plotPath)

        passed = caseResult.testsPassed
        failed = caseResult.testsFailed
        nTests = passed + failed
        lines.append('\n{}/{} tests passed'.format(passed, nTests))

        nPlots = len(plots)
        if nPlots > 1:
            lines.append('\n{} plots generated'.format(len(plots)))
        else:
            lines.append('\n{} plot generated'.format(len(plots)))

        lines.append('\n\n## Plots:  ')

        for plotPath in plots:
            plotFileName = Path(plotPath).absolute()
            plotName = str(plotFileName.name).replace('.png', '')
            lines.append('\n{}'.format(plotName))

            # Copy the plot to the fake module directory
            newPlotPath = newFolderPath / plotFileName.name
            shutil.copy(plotPath, newPlotPath)

            # FOR LOCAL USE
            # lines.append('\n<div><embed width="600" height="480" src="{}"/></div>'.format(newPlotPath.as_uri())) 
            
            # FOR ONLINE
            onlinePath = "https://raw.githubusercontent.com/henrystoldt/MAPLEAF/documentation/V%26V/{}/{}?raw=true".format(caseResult.name, plotFileName.name)
            lines.append('\n<div><img width="600" height="480" src="{}"/></div>'.format(onlinePath)) 

        lines.append('\n\n## Console Output:  ')
        indentedOutput = [ '    ' + x for x in caseResult.consoleOutput ]
        if '\nR' in indentedOutput[0]:
            # Make sure the first row is also indented
            indentedOutput[0] = indentedOutput[0].replace('\nR', '\n    R')
        lines.append(''.join(indentedOutput))

        lines.append('\n"""')

        f.writelines(lines)