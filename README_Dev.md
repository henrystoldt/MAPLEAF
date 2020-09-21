<p align="center">
  <img src="Resources/Draft2Logo.png?raw=true" alt="Logo"
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

## Install from source code:
1. Clone this repository
2. Install Python 3.6+
    - (Linux only): Install corresponding Python3.X-dev package
    - (Windows): Install Visual Studio build tools (search for 'Build Tools for Visual Studio' [here](https://visualstudio.microsoft.com/downloads/))
    - (Optional) - [create dedicated Python environment](#create-python-virtual-environment-with-virtualenvwrapper)  
3. Install MAPLEAF: `$ pip install -e .` (needs to be run from the cloned repository)
    - Just like a normal pip install, but changes made to MAPLEAF's code will take effect without reinstalling
4. (Optional) To be able to do a lot of the things described below, install the packages in requirements_Dev.txt:  
    `$ pip install -r requirements_Dev.txt`  
    Installing Mayavi can be problematic, if using Python 3.8+, try installing the latest version directly from git `$ python3 -m pip install git+https://github.com/enthought/mayavi.git`, or check out [mayavi's documentation](https://github.com/enthought/mayavi) for further assisstance

## Running Unit Tests
**Note:** The Tests Github action runs these tests after every commit to master
Unit Testing Framework Info:
https://docs.python.org/3/library/unittest.html
  
To run all tests: `python3 -m unittest discover -v`   
To run a single test module: `python3 -m unittest -v test.test_Vector`  

## Running Regression Testing / V & V Suite
All regression and V&V tests are defined in [test/regressionTesting/testDefinitions.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/test/regressionTesting/testDefinitions.mapleaf)  
`python3 test/regressionTesting/runCases.py`

## Install virtualenvwrapper (Linux):
**Note:** Official instructions (incl. Windows version): https://virtualenvwrapper.readthedocs.io/en/latest/install.html  
1. `$ sudo python3 -m pip install virtualenvwrapper`
    - Note: If this does not create the script /usr/local/bin/virtualenvwrapper.sh, try uninstalling and reinstalling virtualenvwrapper, making sure to use SUDO on the reinstall
2. Add lines to .bashrc:  
`export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3.8`  
`export WORKON_HOME=$HOME/.virtualenvs`  
`export PROJECT_HOME=$HOME/Devel`  
`source /usr/local/bin/virtualenvwrapper.sh`  
3. `$ source ~/.bashrc`

## Create Python virtual environment with virtualenvwrapper:
1. [Install virtualenvwrapper](#install-virtualenvwrapper-(linux))
2. Create environment: `$ mkvirtualenv -a ~/Documents/rocketSimulator -r ~/Documents/rocketSimulator/requirements_Dev.txt MAPLEAF`  
3. Environment will be automatically activated. To re-activate in the future: `$ workon MAPLEAF`  
4. To deactivate: `$ deactivate`  
5. To delete: `$ rmvirtualenv MAPLEAF`  
More commands here: https://virtualenvwrapper.readthedocs.io/en/latest/command_ref.html

## Profiling Performance
1. To generate profile: `$ python3 -m cProfile -o testProfile.prof MAPLEAF/Main.py /path/to/SimDefinition.mapleaf`
2. To view results: `$ python3 -m snakeviz testProfile.prof`

## Recompiling Cython code
Some of the files in MAPLEAF are .pyx/.pxd instead of .py.  
These are [Cython](https://cython.org/) code files.  
Cython is a superset of Python, meaning all Python code is valid Cython.  

**Note:** Changes to the Cython code will not take effect without [re-compiling the Cython code](https://cython.readthedocs.io/en/latest/src/userguide/source_files_and_compilation.html). 
To re-compile: `$ python3 setup.py build_ext --inplace`

## Uploading to PyPI
(Must be done on Linux/Mac to preserve file cases)
Requires [twine](https://pypi.org/project/twine/)
1. Update the MAPLEAF version number in setup.py
2. `python setup.py sdist`
3. `twine upload dist/MAPLEAF-X.X.X.tar.gz`

## Linting (static error checking)
**Note:** The Linting Github action runs these checks after every commit to master
1. `$ python3 -m flake8 --max-complexity 12`
2. Search for "import" in results (Powershell): `$ python3 -m flake8 --max-complexity 12 | findstr "import"`
3. Search for "import" in results (Bash): `$ python3 -m flake8 --max-complexity 12 | grep "import"`
4. Exact version GitHub auto-runs on commits to master (1 (check for show-stopper errors)): `python3 -m flake8 . --count --select=E9,F63,F7,F82 --show-source --statistics`
5. Exact version GitHub auto-runs on commits to master (2 (check for all errors)): `python3 -m flake8 . --count --exit-zero --max-complexity=10 --max-line-length=127 --statistics`

## Checking test coverage
**Note:** The Tests Github action runs this check after every commit to master
1. `$ python3 -m coverage run --source=./MAPLEAF -m unittest discover -v`
2. `$ python3 -m coverage html`
3. open ./htmlcov/index.html with a web browser to see line-by-line coverage results

## Generating docs
Project is currently set up to auto-generate documentation from module-, class-, function- and variable-level comments using [Pdoc3](https://pdoc3.github.io/pdoc/doc/pdoc/#gsc.tab=0 )  
[Live documentation](https://henrystoldt.github.io/MAPLEAF/) is updated by a GitHub Action every time the master branch is updated.  

To run a local, live html documentation server (see changes to the documentation live):  
`$ pdoc --http : ./MAPLEAF --template-dir ./Resources/DocTemplate`  - then navigate to `localhost:[PORT NUMBER HERE (default 8080)]` in a web browser  
To generate static html docs  
`$ pdoc --html --output-dir docs ./MAPLEAF --template-dir ./Resources/DocTemplate`  

## Generating class/package Resources (Linux)
1. `$ sudo apt install graphviz`
2. (To generate .png files directly):`$ pyreverse -o png -p MAPLEAF ./MAPLEAF`
3. (To generate editable .dot files): `$ pyreverse -p MAPLEAF ./MAPLEAF`
4. (To generate .png from .dot): `$ dot -Tpng packages_MAPLEAF.dot -o packages_MAPLEAF.png`
