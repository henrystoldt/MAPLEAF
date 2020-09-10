# MAPLEAF - Modular Aerospace Prediction Lab for Engines and Aero Forces
Open-source 6-DOF Rocket Flight Simulation

![Unit Tests](https://github.com/henrystoldt/mapleaf/workflows/Tests/badge.svg)
![Linting](https://github.com/henrystoldt/mapleaf/workflows/Linting/badge.svg)
[![Documentation](https://github.com/henrystoldt/mapleaf/workflows/Generate%20documentation/badge.svg)](https://henrystoldt.github.io/MAPLEAF/)
[![codecov](https://codecov.io/gh/henrystoldt/mapleaf/branch/master/graph/badge.svg)](https://codecov.io/gh/henrystoldt/mapleaf)
![python](https://img.shields.io/badge/python-3.6+-brightgreen)
[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://lbesson.mit-license.org/)

## Install:
`pip install MAPLEAF`

## Install from source:
1. Clone this repository
2. Install Python 3.6+
    - (Linux only): Install corresponding Python3.X-dev package
    - (Optional, recommended) - create virtual Python environment (Skip step 3):  
    a. Install virtualenvwrapper: https://virtualenvwrapper.readthedocs.io/en/latest/install.html (other versions available for Windows/Powershell)  
    b. Create environment: `$ mkvirtualenv -a ~/Documents/rocketSimulator -r ~/Documents/rocketSimulator/requirements_Dev.txt MAPLEAF`  
    c. Environment will be automatically activated. To re-activate in the future: `$ workon MAPLEAF`  
    d. To deactivate: `$ deactivate`  
    e. To delete: `$ rmvirtualenv MAPLEAF`  
3. Install Python dependencies: `python3 -m pip install -r /path/to/rocketSimulator/requirements_Dev.txt`  
    a. If the mayavi installation fails, try running: `python3 -m pip install git+https://github.com/enthought/mayavi.git` instead
4. Compile Cython code: `python3 ./setup.py build_ext --inplace`
5. Run the test suite (see instructions below) to ensure everything is functioning properly
    
## Usage:
### Running a Simulation
`python3 Main.py /path/to/SimDefinitionFile.mapleaf`  
All possible options in the SimConfigFile are defined in SimDefinitionTemplate.mapleaf  
Example config files are in the ./test/simDefinitions folder  

### Running Unit Tests
Unit Testing Framework Info:
https://docs.python.org/3/library/unittest.html

Navigate to the project root directory in the command line (Where this readme is located).
To run all tests:  
`python3 -m unittest discover -v`  
To run a single test:  
`python3 -m unittest -v test.[Test Module Name]`  
Example:  
`python3 -m unittest -v test.test_Vector`  

### Running Regression Testing / Validation Suite
`python3 test/regressionTesting/runCases.py`

### Directory Structure
| Directory  | Subdirectories           | Contents                                                                                     |
|------------|--------------------------|----------------------------------------------------------------------------------------------|
| ./         |                          | Documentation/Examples                                                                       |
| ./.vscode: |                          | Config files for Visual Studio Code                                                          |
| ./.github: |                          | Code to autorun unit tests on GitHub                                                         |
|            |                          |                                                                                              |
| ./MAPLEAF:     |                          | The rocket simulator code (main sim program is Main.py)                                      |
|            | ./MAPLEAF/ENV:               | Environmental Modelling code                                                                 |
|            | ./MAPLEAF/GNC:               | Guidance, Navigation and Control Modelling code                                              |
|            | ./MAPLEAF/IO:                | Code to do with reading/writing simulation definition files, log files, and generating plots |
|            | ./MAPLEAF/MisdatSource:      | Missile Datcom Source code (Fortran)                                                         |
|            | ./MAPLEAF/Motion:            | Rigid Body Motion Code, including basic data types like Vector & Quaternion                  |
|            | ./MAPLEAF/NotcomInterface:   | Missile Datcom-Python interface                                                              |
|            | ./MAPLEAF/Rocket:            | Rocket Modelling Code - Rocket, Stage, Rocket Components                                     |
|            |                          |                                                                                              |
| ./test:    |                          | Unit testing code and cases for the rocket simulator                                         |
|            | ./test/motorDefinitions: | Motor definition files                                                                       |
|            | ./test/regressionTesting | All regression testing / validation code, test cases, comparison data, etc...                |
|            | ./test/simDefinitions:   | Simulation definition files                                                                  |
|            | ./test/speedTests:       | Python scripts comparing the speeds of different methods/classes\                            |
|            | ./test/test_ENV:         | Each test_XXX folder tests the code in ./MAPLEAF/XXX                                             |
|            | ./test/test_GNC:         | ''                                                                                           |
|            | ./test/test_IO:          | ''                                                                                           |
|            | ./test/test_Motion:      | ''                                                                                           |
|            | ./test/test_Rocket:      | ''                                                                                           |
|            | ./test/WindData:         | Radio sonde and wind rose data. Currently contains data relevant to Suffield\                |

## To run linting (static error checking)
1. `$ python3 -m flake8 --max-complexity 12`
2. Search for "import" in results (Powershell): `$ python3 -m flake8 --max-complexity 12 | findstr "import"`
3. Search for "import" in results (Bash): `$ python3 -m flake8 --max-complexity 12 | grep "import"`
4. Exact version GitHub auto-runs on commits to master (1 (check for show-stopper errors)): `python3 -m flake8 . --count --select=E9,F63,F7,F82 --show-source --statistics`
5. Exact version GitHub auto-runs on commits to master (2 (check for all errors)): `python3 -m flake8 . --count --exit-zero --max-complexity=10 --max-line-length=127 --statistics`

## To check unit test coverage
1. `$ python3 -m coverage run --source=./MAPLEAF -m unittest discover -v`
2. `$ python3 -m coverage html`
3. open ./htmlcov/index.html with a web browser to see line-by-line coverage results

## To generate documentation
Project is currently set up for Pdoc3: https://pdoc3.github.io/pdoc/doc/pdoc/#gsc.tab=0  
To run a live html documentation server (so you can see changes to the documentation as you make them):  
`$ pdoc --http : ./MAPLEAF`  - then navigate to `localhost:[PORT NUMBER HERE (default 8080)]` in a web browser  
To generate static html docs  
`$ pdoc --html --output-dir doc ./MAPLEAF`  

## To profile performance
1. To generate profile: `$ python3 -m cProfile -o testProfile.prof /path/to/Main.py /path/to/SimDefinition.txt`
2. To view results: `$ python3 -m snakeviz testProfile.prof`

## To generate class/package diagrams (Linux)
1. `$ sudo apt install graphviz`
2. (To generate .png files directly):`$ pyreverse -o png -p rocketSimulator ./rocketSimulator`
3. (To generate editable .dot files): `$ pyreverse -p rocketSimulator ./rocketSimulator`
4. (To generate .png from .dot): `$ dot -Tpng packages_rocketSimulator.dot -o packages_rocketSimulator.png`

## Developer: Use Julia w/ Python
**Note:** It seems like there's a lot of communication overhead with this method (for small tasks). Only consider outsourcing large computations.
1. Ensure Julia and Python are installed. They MUST BOTH be 64-bit versions or BOTH be 32-bit versions
2. `$pip install julia`
3. `>>> import julia`
4. `>>> julia.install()`

## Developer: Compile Python code w/ Cython
**Note:** Type-annotated Cython code is almost as fast as numba, but is no longer Python-compatible. Suitable for speeding up small tasks. Communication overhead between Python and Cython code seems to be near zero.  
**Note:** To compile all built-in cython code - run ./compileCythonCode.bat (Windows) or ./compileCythonCode.sh (Linux)

For windows, prior to running the previous command, you must:
Download the C++ option from Visual Studio build Tools installer: https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=BuildTools&rel=16

1. Locate a .pyx file for compilation
2. Two methods to compile the file:
  * Use the direct command line method: `$ cythonize -i fileToCompile.pyx`
  * Use a setup.py file: `$ python3 setup.py build_ext --inplace` - this is especially useful for compiling many files at once
  * More info: https://cython.readthedocs.io/en/latest/MAPLEAF/userguide/source_files_and_compilation.html


## Developer: Install virtualenvwrapper (Linux):
**Note:** Go to the website for official installation instructions (incl. Windows version): https://virtualenvwrapper.readthedocs.io/en/latest/install.html  
1. `$ sudo python3 -m pip install virtualenvwrapper`
  - Note: If this does not create the script /usr/local/bin/virtualenvwrapper.sh, try uninstalling and reinstalling virtualenvwrapper, making sure to use SUDO on the reinstall
2. Add lines to .bashrc:  
`export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3.8`  
`export WORKON_HOME=$HOME/.virtualenvs`  
`export PROJECT_HOME=$HOME/Devel`  
`source /usr/local/bin/virtualenvwrapper.sh`  
3. `$ source ~/.bashrc`
