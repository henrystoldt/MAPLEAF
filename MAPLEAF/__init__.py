'''
<p align="center">
  <img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/DraftLogo.png" alt="Logo"
    title="MAPLEAF" height=125 style="padding-right: 10px;"/>
</p>

<p align="center">
<a align="center" href="https://github.com/henrystoldt/mapleaf/actions"><img alt="Tests" src="https://github.com/henrystoldt/mapleaf/workflows/Tests/badge.svg"></a>
  <a align="center" href="https://github.com/henrystoldt/mapleaf/actions"><img alt="Linting" src="https://github.com/henrystoldt/mapleaf/workflows/Linting/badge.svg"></a>
  <a align="center" href="https://henrystoldt.github.io/MAPLEAF/"><img alt="Docs" src="https://github.com/henrystoldt/mapleaf/workflows/Docs/badge.svg"></a>
  <a align="center" href="https://codecov.io/gh/henrystoldt/mapleaf"><img alt="Coverage" src="https://codecov.io/gh/henrystoldt/mapleaf/branch/master/graph/badge.svg"></a>
  <img alt="Python3" src="https://img.shields.io/badge/python-3.6+-brightgreen">
  <a align="center" href="https://lbesson.mit-license.org/"><img alt="MIT License" src="https://img.shields.io/badge/License-MIT-blue.svg"></a>
</p>

Simulation entry point: `MAPLEAF.Main.main`.  
`MAPLEAF.Main.main` will initialize one of the classes in `MAPLEAF.SimulationRunners` to drive/manage the simulation.
The simulation runner will then initialize an instance of `MAPLEAF.Rocket.Rocket`, which coordinates the modelling of a Rocket.

See [README.md](https://github.com/henrystoldt/MAPLEAF/blob/master/README.md) for info about:  

* Installation/setup  
* Running simulations 

See [SimulationDefinitionTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/SimDefinitionTemplate.mapleaf) for details about all available options

See [README_Dev.md](https://github.com/henrystoldt/MAPLEAF/blob/master/README_Dev.md) for info about:  

* Running unit tests  
* Running regression/validation tests  
* Recompiling Cython Code
'''

__pdoc__ = {
    'Examples': False
}