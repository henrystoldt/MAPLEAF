# MAPLEAF - Modular Aerospace Prediction Lab for Engines and Aero Forces
Open-source 6-DOF Rocket Flight Simulation

![Unit Tests](https://github.com/henrystoldt/mapleaf/workflows/Tests/badge.svg)
![Linting](https://github.com/henrystoldt/mapleaf/workflows/Linting/badge.svg)
[![Documentation](https://github.com/henrystoldt/mapleaf/workflows/Docs/badge.svg)](https://henrystoldt.github.io/MAPLEAF/)
[![codecov](https://codecov.io/gh/henrystoldt/mapleaf/branch/master/graph/badge.svg)](https://codecov.io/gh/henrystoldt/mapleaf)
![python](https://img.shields.io/badge/python-3.6+-brightgreen)
[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://lbesson.mit-license.org/)

## Install from source code:
1. Clone this repository
2. Install Python 3.6+
    - (Linux only): Install corresponding Python3.X-dev package
    - (Optional, recommended) - create dedicated Python environment (Skip step 3):  
    a. Install virtualenvwrapper: https://virtualenvwrapper.readthedocs.io/en/latest/install.html (other versions available for Windows/Powershell)  
    b. Create environment: `$ mkvirtualenv -a ~/Documents/rocketSimulator -r ~/Documents/rocketSimulator/requirements_Dev.txt MAPLEAF`  
    c. Environment will be automatically activated. To re-activate in the future: `$ workon MAPLEAF`  
    d. To deactivate: `$ deactivate`  
    e. To delete: `$ rmvirtualenv MAPLEAF`  
3. Install MAPLEAF: `$ pip install -e .` (needs to be run from this repository's main folder)
    - This installs just like a normal pip install, making MAPLEAF libraries and the `mapleaf` command available system-wide, but the -e means 'editable', which means changes you make to the MAPLEAF's python code will take effect without reinstalling
4. Run the test suite (see below) to ensure everything is functioning properly

### Running Unit Tests
**Note:** The Tests Github action runs these tests after every commit to master
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
All regression tests are defined in test/regressionTesting/testDefinitions.mapleaf
`python3 test/regressionTesting/runCases.py`

## Install virtualenvwrapper (Linux):
**Note:** Go to the website for official installation instructions (incl. Windows version): https://virtualenvwrapper.readthedocs.io/en/latest/install.html  
1. `$ sudo python3 -m pip install virtualenvwrapper`
    - Note: If this does not create the script /usr/local/bin/virtualenvwrapper.sh, try uninstalling and reinstalling virtualenvwrapper, making sure to use SUDO on the reinstall
2. Add lines to .bashrc:  
`export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3.8`  
`export WORKON_HOME=$HOME/.virtualenvs`  
`export PROJECT_HOME=$HOME/Devel`  
`source /usr/local/bin/virtualenvwrapper.sh`  
3. `$ source ~/.bashrc`

## To run linting (static error checking)
**Note:** The Linting Github action runs these checks after every commit to master
1. `$ python3 -m flake8 --max-complexity 12`
2. Search for "import" in results (Powershell): `$ python3 -m flake8 --max-complexity 12 | findstr "import"`
3. Search for "import" in results (Bash): `$ python3 -m flake8 --max-complexity 12 | grep "import"`
4. Exact version GitHub auto-runs on commits to master (1 (check for show-stopper errors)): `python3 -m flake8 . --count --select=E9,F63,F7,F82 --show-source --statistics`
5. Exact version GitHub auto-runs on commits to master (2 (check for all errors)): `python3 -m flake8 . --count --exit-zero --max-complexity=10 --max-line-length=127 --statistics`

## To check unit test coverage
**Note:** The Tests Github action runs this check after every commit to master
1. `$ python3 -m coverage run --source=./MAPLEAF -m unittest discover -v`
2. `$ python3 -m coverage html`
3. open ./htmlcov/index.html with a web browser to see line-by-line coverage results


## To profile performance
1. To generate profile: `$ python3 -m cProfile -o testProfile.prof /path/to/Main.py /path/to/SimDefinition.txt`
2. To view results: `$ python3 -m snakeviz testProfile.prof`

## Recompile Cython code after editing
Some of the files in MAPLEAF are .pyx/.pxd instead of .py.  
These are Cython code files.  
Cython is a superset of Python, meaning all Python code is valid Cython.  
More info: https://cython.org/  

**Note:** Changes to the Cython code will not take effect without re-compiling the Cython code. To re-compile, run `$ python3 setup.py build_ext --inplace`


## To generate documentation
Project is currently set up to auto-generate documentation from module-, class-, function- and variable-level comments using Pdoc3: https://pdoc3.github.io/pdoc/doc/pdoc/#gsc.tab=0  
Live docs (https://henrystoldt.github.io/MAPLEAF/) are updated by a GitHub Action every time the master branch is updated.  

To run a local,live html documentation server (so you can see changes to the documentation as you make them):  
`$ pdoc --http : ./MAPLEAF`  - then navigate to `localhost:[PORT NUMBER HERE (default 8080)]` in a web browser  
To generate static html docs  
`$ pdoc --html --output-dir docs ./MAPLEAF`  

## To generate class/package diagrams (Linux)
1. `$ sudo apt install graphviz`
2. (To generate .png files directly):`$ pyreverse -o png -p rocketSimulator ./rocketSimulator`
3. (To generate editable .dot files): `$ pyreverse -p rocketSimulator ./rocketSimulator`
4. (To generate .png from .dot): `$ dot -Tpng packages_rocketSimulator.dot -o packages_rocketSimulator.png`