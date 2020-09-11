

<p align="center">
  <img src="Diagrams/DraftLogo.png?raw=true" alt="Logo"
    title="MAPLEAF" height=125 style="padding-right: 10px;"/>
  <h1 align="center">MAPLEAF: Modular Aerospace Prediction Lab for Engines and Aero Forces</h1>
  <p align="center">6-DOF Rocket Flight Simulation Framework </p>
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
`pip install MAPLEAF`
    
## Usage:

### Running a Simulation
`mapleaf path/to/SimDefinitionFile.mapleaf`  

Example config files are in the ./MAPLEAF/Examples/Simulations folder

### Simulation Definition Files
More info, and descriptions of all possible options in: [SimDefinitionTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/SimDefinitionTemplate.mapleaf)

Format is a simple key-value syntax similar to JSON or YAML.  
Dictionaries can be nested arbitrarily deeply and are brace-delimited.  
Keys and values in a dictionary are separated by the first whitespace in their line  
No multiline values

Example segment of a Sim Definition file:
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

[Code folding](https://code.visualstudio.com/docs/editor/codebasics#_folding) is very helpful in maintaining an overview of these files:

![Folded Sim Definition](Diagrams/SimDefinitionFile_Folded.PNG?raw=true)

The possible top level dictionaries are 'Monte Carlo', 'SimControl', 'Environment', and 'Rocket'.
Of these, only the 'Rocket' dictionary is strictly required to run a simulation, and defines the rocket's initial position/velocity and the inertial/aerodynamic/control models used to simulate it.  
The rocket is defined by nested subdictionaries, where the first level of nesting defines the rocket's stage(s) and the second level defines the rocket component(s) in each stage:

![Rocket Definition](Diagrams/SimDefinitionFile_Rocket.PNG?raw=true)

Default values from the defaultConfigValues dictionary in [MAPLEAF/IO/SimDefinition.py](https://github.com/henrystoldt/MAPLEAF/blob/master/MAPLEAF/IO/SimDefinition.py) will fill in when keys are undefined.
Default values mostly match those in the [SimDefinitionTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/SimDefinitionTemplate.mapleaf) file.

### Simulation Outputs
Depending on the options specified in the `SimControl` dictionary, MAPLEAF simulations can output:
- Detailed tabulated simulation position, component force, aerodynamic coefficient and control logs (see SimControl.loggingLevel):
![Sample section of log file](Diagrams/LogSample.PNG?raw=true)

- Flight animations (see SimControl.plot)  
<img src="Diagrams/FlightAnimation.gif?raw=true" alt="Flight Animation"
  title="MAPLEAF" height=225 style="padding-right: 10px;"/>

- Flight path visualizations (see SimControl.plot)  
<img src="Diagrams/EarthOrbit.png?raw=true" alt="Earth Orbit"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>
<img src="Diagrams/FlightPaths.png?raw=true" alt="Flight Paths"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

- Plots of any logged parameter (see SimControl.plot)  
<img src="Diagrams/PlottingFromLogs.png?raw=true" alt="Plotting from logs"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

### Monte Carlo Simulations
Monte Carlo simulations propagate uncertainties in simulation inputs through to simulation outputs.  
Any scalar or vector parameter in simulation definition files can be made probabilistic by adding a second parameter with `_stdDev` appended to the name:

![Monte Carlo Parameter](Diagrams/SimDefinition_MonteCarlo.png?raw=true)

To execute a batch run of this now-probabilistic simulation, create the top-level 'Monte Carlo' dictionary (see [SimDefinitionTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/))

From that, you can obtain distributions of outputs like flight paths or landing locations:

<img src="Diagrams/FlightPathsPlot.png?raw=true" alt="Flight Paths Plot"
  title="MAPLEAF" height=350 style="padding-right: 10px;"/>
<img src="Diagrams/LandingLocationPlot.png?raw=true" alt="Landing Location Plot"
  title="MAPLEAF" height=350 style="padding-right: 10px;"/>

### Developers
To extend MAPLEAF, re-use its libraries or otherwise work with the code, have a look at [README_Dev.md](https://github.com/henrystoldt/MAPLEAF/blob/master/README_Dev.md) the [code/api documentation website](https://henrystoldt.github.io/MAPLEAF/)
