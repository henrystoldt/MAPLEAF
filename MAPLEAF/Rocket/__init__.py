r'''
Code specific to modelling Rockets.  
Main class is `Rocket`.

Other files define models for specific rocket components or stages.  
In general the responsibility of rocket component classes is to model:

1. The forces/moments (aerodynamic or otherwise (excluding gravitational forces)) applied to the rocket by that rocket component
2. The inertia of that rocket component

The interface all rocket components are expected to implement is defined by the abstract base class `MAPLEAF.Rocket.RocketComponent`. All rocket components inherit from this base class.

.. image:: https://airandspace.si.edu/sites/default/files/images/NASAJSC2002-01598h.jpg

MAPLEAF.Rocket relies on (in order of coupling) `MAPLEAF.Motion`, `MAPLEAF.ENV`, `MAPLEAF.IO`, and `MAPLEAF.GNC`

### Approximate Material Roughnesses
| Material                           | Roughness ( \(\mu m\) )                                        |
|------------------------------------|-------------------------------------------------------|
| Mirror:                            | 0                                                     |
| Glass:                             | 0.1                                                   |
| Finished/Polished surface:         | 0.5                                                   |
| Aircraft-type sheet-metal surface: | 2                                                     |
| Optimum paint-sprayed surfaces:    | 5                                                     |
| Planed wooden boards:              | 15                                                    |
| Paint in aircraft mass production: | 20                                                    |
| Steel plating: bare:               | 50                                                    |
| Smooth cement:                     | 50                                                    |
| Surface with asphalt-type coating: | 100                                                   |
| Dip-galvanized metal surface:      | 150                                                   |
| Incorrectly sprayed paint:         | 200                                                   |
| Natural cast-iron surface:         | 250                                                   |
| Raw wooden boards:                 | 500                                                   |
| Average concrete:                  | 1000                                                  |

(From Barrowman, 1967, Table 4-1)  

'''
# Make the classes in all submodules importable directly from MAPLEAF.Rocket
from .RocketComponents import *
from .simEventDetector import *
from .boatTail import *
from .bodyTube import *
from .Fins import *
from .Propulsion import *
from .Recovery import *
from .noseCone import *
from .RocketComponentFactory import *
from .stage import *
from .rocket import *

subModules = [ RocketComponents, simEventDetector, boatTail, bodyTube, Fins, Propulsion, Recovery, noseCone, stage, rocket, RocketComponentFactory ]

__all__ = [ ]

for subModule in subModules:
    __all__ += subModule.__all__