'''
Input/Output functionality:

* Reading/Writing Simulation Definition Files
* Plotting/Logging results

MAPLEAF.IO Relies on MAPLEAF.Motion to implement a few convenience / parsing functions.

.. image:: https://upload.wikimedia.org/wikipedia/commons/thumb/f/f8/Laptop-hard-drive-exposed.jpg/1280px-Laptop-hard-drive-exposed.jpg
'''
# Make the classes in all submodules importable directly from MAPLEAF.Rocket
from .rocketFlight import *
from .simDefinition import *
from .subDictReader import *

subModules = [ rocketFlight, simDefinition, subDictReader ]

__all__ = []

for subModule in subModules:
    __all__ += subModule.__all__