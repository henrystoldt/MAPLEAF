'''
Environmental modelling: main class is `MAPLEAF.ENV.Environment`.
`MAPLEAF.ENV.Environment` wraps the atmospheric models, gravity models, mean wind models, and turbulence models.

.. image:: https://www.goodfreephotos.com/albums/astrophotography/sunrise-over-the-earth.jpg
'''
# Make the classes in all submodules importable directly from MAPLEAF.Rocket
from .launchRail import *
from .MeanWindModelling import *
from .EarthModelling import *
from .AtmosphereModelling import *
from .TurbulenceModelling import *
from .environment import *

subModules = [ environment, launchRail, MeanWindModelling, EarthModelling, AtmosphereModelling, TurbulenceModelling ]

__all__ = []

for subModule in subModules:
    __all__ += subModule.__all__