# Responsible for moving plots from ./MAPLEAF/V&V/* to ./docs/MAPLEAF/V&V/*

from pathlib import Path
import os
import shutil

topLevelFrom = Path('./MAPLEAF/V&V')
topLevelTo = Path('./docs/MAPLEAF/V&V')

folders = os.listdir(topLevelFrom)

for folder in folders:
    # Find directories
    completePath = topLevelFrom / folder
    if completePath.is_dir():
        files = os.listdir(completePath)

        for file in files:
            completeFilePath = completePath / file
            
            if '.png' in file:
                # We've found a figure that we need to move to the documentation directory
                pathAfterMove = topLevelTo / folder / file
                shutil.copy(completeFilePath, pathAfterMove)