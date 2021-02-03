"""
    This file generates python modules in ./MAPLEAF/V&V which will be turned into verification and validation website by pdoc3 every time the documentation is built
    One module/webpage is generated per folder in ./test/V&V/. All .pdf files in the folders are displayed on the page.
"""

import os

# Create the verification and validation folder
os.mkdir('./MAPLEAF/V&V')

# 