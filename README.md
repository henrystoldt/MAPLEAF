

<p align="center">
  <img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/DraftLogo.png" alt="Logo"
    title="MAPLEAF" height=125 style="padding-right: 10px;"/>
  <h1 align="center">MAPLEAF: Modular Aerospace Prediction Lab for Engines and Aero Forces</h1>
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
    
## Running a Simulation
`$ mapleaf path/to/SimDefinitionFile.mapleaf`  

Sample simulation definitions: [MAPLEAF/Examples/Simulations](https://github.com/henrystoldt/MAPLEAF/tree/master/MAPLEAF/Examples/Simulations)  
Example cases be run with just the case name: `$ mapleaf NASATwoStageOrbitalRocket`  
This is the same as running: `$ mapleaf MAPLEAF/Examples/Simulations/NASATwoStageOrbitalRocket.mapleaf` from MAPLEAF's install location

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

[Code folding](https://code.visualstudio.com/docs/editor/codebasics#_folding) is very helpful in maintaining a file overview:

![Folded Sim Definition](https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/SimDefinitionFile_Folded.PNG?raw=true)

The possible top level dictionaries are 'Monte Carlo', 'SimControl', 'Environment', and 'Rocket'.
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

To execute a batch run of this now-probabilistic simulation, create the top-level 'Monte Carlo' dictionary (see [SimDefinitionTemplate.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/))

Then, MAPLEAF can produce distributions of outputs like landing locations:  
<img src="https://raw.githubusercontent.com/henrystoldt/MAPLEAF/master/Resources/LandingLocationPlot.png?raw=true" alt="Landing Location Plot"
  title="MAPLEAF" height=300 style="padding-right: 10px;"/>

## Developers
Contributions are welcome.
To learn about the code, have a look at [README_Dev.md](https://github.com/henrystoldt/MAPLEAF/blob/master/README_Dev.md), and the [code/api documentation website](https://henrystoldt.github.io/MAPLEAF/)