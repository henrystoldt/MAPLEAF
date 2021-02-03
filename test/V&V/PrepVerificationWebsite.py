"""
    This file generates python modules in ./MAPLEAF/V&V which will be turned into verification and validation website by pdoc3 every time the documentation is built
    One module/webpage is generated per folder in ./test/V&V/. All .pdf files in the folders are displayed on the page.
"""

import os
import shutil

from MAPLEAF.IO import SimDefinition
from MAPLEAF.SimulationRunners.Batch import BatchRun, run


shutil.rmtree('./MAPLEAF/V&V')

# Load the simulation definition file that defines all of the regression verification and validation cases
batchDefinition = SimDefinition('./MAPLEAF/Examples/BatchSims/regressionTests.mapleaf')
batchRun = BatchRun(batchDefinition)

# Run all the cases to generate the result pdfs
# run(batchRun)

# Create the verification and validation folder, make it a python module
os.mkdir('./MAPLEAF/V&V')
with open("./MAPLEAF/V&V/__init__.py", "w+") as f:
    f.write("''' Testing page '''")

# Create all of the submodules
filesAndFolders = os.listdir('./test/V&V')
for folder in filesAndFolders:
    completePath = os.path.join('./test/V&V', folder)
    
    if os.path.isdir(completePath):
        newFolderPath = os.path.join('./MAPLEAF/V&V', folder)
        os.mkdir(newFolderPath)

        # Find all of the .pdfs for this case
        allFiles = os.listdir(completePath)
        pdfs = []
        for file in allFiles:
            if len(file) > 3 and file[-4:].lower() == '.pdf':
                pdfs.append(os.path.join(completePath, file))

        initPath = os.path.join(newFolderPath, '__init__.py')
        with open(initPath, 'w+') as f:
            lines = [ '"""' ]
            for pdf in pdfs:
                pdf = pdf.replace('\\', '/') 
                pdf = pdf[2:]
                lines.append('\n    <div><embed width="600" height="480" src="../../../../{}"/></div>'.format( pdf ))

            lines.append('\n"""')

            f.writelines(lines)