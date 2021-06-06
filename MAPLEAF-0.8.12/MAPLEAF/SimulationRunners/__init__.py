'''
Defines functions and classes that manage simulations.  
Includes some (`Simulation`) that run a single simulation, and others (`OptimizingSimRunner`) that run many simulations at once.

.. image:: https://storage.needpix.com/rsynced_images/important-1705212_1280.png
'''

# Make the classes in all submodules importable directly from MAPLEAF.Rocket
from .SingleSimulations import *
from .MonteCarlo import *
from .Convergence import *
from .Optimization import *
from .Batch import *

# For some reason CythonVector and company don't exist down here, so they won't import when running from MAPLEAF.Motion import *
subModules = [ SingleSimulations, MonteCarlo, Convergence, Optimization, Batch ]

__all__ = [ ]

for subModule in subModules:
    __all__ += subModule.__all__