<p align="center">
  <img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/Draft2Logo.png" alt="Logo"
    title="MAPLEAF" height=150 style="padding-right: 10px;"/>
  <h1 align="center">Modular Aerospace Prediction Lab for Engines and Aero Forces</h1>
</p>

<p align="center">
<a align="center" href="https://github.com/henrystoldt/mapleaf/actions"><img alt="Tests" src="https://github.com/henrystoldt/mapleaf/workflows/Tests/badge.svg"></a>
  <a align="center" href="https://github.com/henrystoldt/mapleaf/actions"><img alt="Linting" src="https://github.com/henrystoldt/mapleaf/workflows/Linting/badge.svg"></a>
  <a align="center" href="https://henrystoldt.github.io/MAPLEAF/"><img alt="Docs" src="https://github.com/henrystoldt/mapleaf/workflows/Docs/badge.svg"></a>
  <a align="center" href="https://codecov.io/gh/henrystoldt/mapleaf"><img alt="Coverage" src="https://codecov.io/gh/henrystoldt/mapleaf/branch/master/graph/badge.svg"></a>
  <img alt="Python3" src="https://img.shields.io/badge/python-3.6+-brightgreen">
  <a align="center" href="https://lbesson.mit-license.org/"><img alt="MIT License" src="https://img.shields.io/badge/License-MIT-blue.svg"></a>
</p>

## Install:
Prerequisites:

- [Python 3.6+](https://www.python.org/downloads/) (**[64-bit](https://stackoverflow.com/questions/1842544/how-do-i-detect-if-python-is-running-as-a-64-bit-application)**)
  - Linux/WSL: install python3.X-dev package (includes python header files for Cython)
- C/C++ compilers OR an existing installation of [Cython](https://cython.readthedocs.io/en/latest/src/quickstart/install.html)
  - Windows: install visual studio build tools or MinGW
- (Recommended) A Python [virtual environment](https://towardsdatascience.com/virtual-environments-104c62d48c54). See [README_Dev.md](https://github.com/henrystoldt/MAPLEAF/blob/master/README_Dev.md#create-python-virtual-environment-with-virtualenvwrapper)

`$ pip install MAPLEAF`

The (optional) dependencies used to enable 3D flight path renders (mayavi) and parallelization (ray) are not installed by default because they're often tricky to get running. Run the [installOptionalPackages.py](https://github.com/henrystoldt/MAPLEAF/blob/master/installOptionalPackages.py) script to try installing them if you want to use those capabilities. See ray and mayavi github pages/documentation for more help getting them installed.

### Test your installation
Unit tests:  
`$ python -m unittest -v`  
Regression tests:  
`$ mapleaf-batch`  

See [README_Dev.md](https://github.com/henrystoldt/MAPLEAF/blob/master/README_Dev.md) for more information, or the [verification and validation section](https://henrystoldt.github.io/MAPLEAF/V&V/index.html) of the code documentation for the expected results of the regression tests.

## Developers
Contributions are welcome.
To learn about the code, have a look at [README_Dev.md](https://github.com/henrystoldt/MAPLEAF/blob/master/README_Dev.md), and the [code documentation website](https://henrystoldt.github.io/MAPLEAF/)

## Running a Simulation
`$ mapleaf path/to/SimDefinitionFile.mapleaf`  

Sample simulation definitions: [MAPLEAF/Examples/Simulations](https://github.com/henrystoldt/MAPLEAF/tree/master/MAPLEAF/Examples/Simulations)  
Example cases be run with just the case name: `$ mapleaf NASATwoStageOrbitalRocket`  
This is the same as running: `$ mapleaf MAPLEAF/Examples/Simulations/NASATwoStageOrbitalRocket.mapleaf` from MAPLEAF's install location

Help: `$ mapleaf -h`

### Simulation Definition Files
Brief overview below. **More info, and definitions of all possible options in:** [SimDefinitionTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/SimDefinitionTemplate.mapleaf)

Format is a simple key-value syntax similar to JSON or YAML.  
Dictionaries can be nested arbitrarily deeply and are brace-delimited.  
Keys and values in a dictionary are separated by the first whitespace in their line  
No multiline values

Example:
```  
SimControl{
  timeDiscretization    RK45Adaptive

  TimeStepAdaptation{
    controller          PID
    PID.coefficients    -0.01 -0.001 0
    targetError         0.0001
  }
}
```

[Code folding](https://code.visualstudio.com/docs/editor/codebasics#_folding) and [syntax highlighting](https://marketplace.visualstudio.com/items?itemName=MAPLEAF.MAPLEAF) are helpful in maintaining a file overview:

![Folded Sim Definition](https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/SimDefinitionFile_Folded.PNG?raw=true)

The possible top level dictionaries are 'Optimization', 'MonteCarlo', 'SimControl', 'Environment', and 'Rocket'.
Of these, only the 'Rocket' dictionary is strictly required to run a simulation, and defines the rocket's initial position/velocity and the inertial/aerodynamic/control models used to simulate it.  
The rocket is defined by nested subdictionaries, where the first level of nesting defines the rocket's stage(s) and the second level defines the component(s) in each stage:

![Rocket Definition](https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/SimDefinitionFile_Rocket.PNG?raw=true)

Default values from the defaultConfigValues dictionary in [MAPLEAF/IO/SimDefinition.py](https://github.com/henrystoldt/MAPLEAF/blob/master/MAPLEAF/IO/SimDefinition.py) will fill in for omitted keys.
Most defaults match the values in [SimDefinitionTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/SimDefinitionTemplate.mapleaf).

## Simulation Outputs
Depending on the options specified in the `SimControl` dictionary, MAPLEAF will output:
- Detailed tabulated simulation position, component force, aerodynamic coefficient and control logs (see SimControl.loggingLevel):
![Sample section of log file](https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/LogSample.PNG?raw=true)

- Flight animations (see SimControl.plot)  
<img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/FlightAnimation.gif?raw=true" alt="Flight Animation"
  title="MAPLEAF" height=225 style="padding-right: 10px;"/>

- Flight path visualizations (see SimControl.plot - [Mayavi](https://github.com/enthought/mayavi) is required to render these ones showing the Earth)  
<img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/EarthOrbit.png?raw=true" alt="Earth Orbit"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

- Plots of any logged parameter (see SimControl.plot or --plotFromLog command line option)  
<img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/PlottingFromLogs.png?raw=true" alt="Plotting from logs"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

## Monte Carlo Simulations
Monte Carlo simulations propagate uncertainties in simulation inputs through to simulation outputs.  
Any scalar or vector parameter in simulation definition files can be made probabilistic by adding a second parameter with `_stdDev` appended to the name:

![Monte Carlo Parameter](https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/SimDefinition_MonteCarlo.png?raw=true)

To execute a batch run of this now-probabilistic simulation, create the top-level 'Monte Carlo' dictionary (see [SimDefinitionTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/SimDefinitionTemplate.mapleaf))

Then, MAPLEAF can produce distributions of outputs like landing locations:  
<img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/LandingLocationPlot.png?raw=true" alt="Landing Location Plot"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

## Design Optimization
MAPLEAF uses [pyswarms](https://github.com/ljvmiranda921/pyswarms) to optimize arbitrary scalar parameters according to arbitrary cost functions using Particle Swarm Optimization.
To define an optimization problem, include the top-level 'Optimization' dictionary (see [SimDefinitionTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/SimDefinitionTemplate.mapleaf)):

<img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/SimDefinition_Optimization.PNG?raw=true" alt="Optimization Dict"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

Pyswarms will produce a plot of the optimization results after completion:

<img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/OptimizationResult.png?raw=true" alt="Optimization Result"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

## Batch Simulations
Automates running and post-processing groups of cases.  
To run: `$ mapleaf-batch path/to/batchDefinitionFile`  
For help: `$ mapleaf-batch -h`

![Batch Definition](Resources/BatchDefinition.png)

Provides ability to automate production of detailed plots of MAPLEAF's results compared to verification/validation data, and to run Wind-Tunnel style aero-sweep simulations:

<img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/CantedFinRocketPlot.png?raw=true" alt="Canted Fin Roll Rate"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

<img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/ParametricFin5CAvsMach.png?raw=true" alt="Wind Tunnel Comparison"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

Used for regression and verification/validation testing in MAPLEAF.  
See [batchRunTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/batchRunTemplate.mapleaf) for complete batch interface definition and [regressionTests.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/MAPLEAF/Examples/BatchSims/regressionTests.mapleaf) for examples.

To run the regression/verification/validation test suite, check out [README_Dev.md](https://github.com/henrystoldt/MAPLEAF/blob/master/README_Dev.md)

## Parallelization
To run **Monte Carlo** or **Optimization** simulations in parallel:
`mapleaf --parallel MonteCarlo.mapleaf`

MAPLEAF's parallelization is implemented using [ray](https://github.com/ray-project/ray).

## Cite MAPLEAF
[Stoldt, H., Quinn, D., Kavanagh, J., Johansen, C. (2021). MAPLEAF: A Compact, Extensible, Open-Source, 6-
Degrees-of-Freedom Rocket Flight Simulation Framework. In 2021 AIAA Propulsion & Energy Forum.](https://arc.aiaa.org/doi/abs/10.2514/6.2021-3267)