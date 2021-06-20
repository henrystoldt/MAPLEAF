<p align="center">
  <img src="Resources/Draft2Logo.png?raw=true" alt="Logo"
    title="MAPLEAF" height=150 style="padding-right: 10px;"/>
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
2. Install Python 3.6+ **(64-bit**)
    - (Linux only): Install corresponding Python3.X-dev package
    - (Windows): Install Visual Studio build tools (search for 'Build Tools for Visual Studio' a ways down the page in the 'All Downloads' section [here](https://visualstudio.microsoft.com/downloads/))
    - (Optional) - [create dedicated Python environment](#create-python-virtual-environment-with-virtualenvwrapper)  
3. Install MAPLEAF: `$ pip install -e .` (needs to be run from the cloned repository)
    - Just like a normal pip install, but changes made to MAPLEAF's code will take effect without reinstalling
    - If there's an error installing ray, either switch to a Linux/Mac environment, or remove it from the requirements.txt file
        - Without ray, everything except multi-threaded sims will work
4. (Optional) To be able to do a lot of the things described below, install the packages in requirements_Dev.txt:  
    `$ pip install -r requirements_Dev.txt`  
    Installing Mayavi can be problematic, if using Python 3.8+, try installing the latest version directly from git `$ python3 -m pip install git+https://github.com/enthought/mayavi.git`, or check out [mayavi's documentation](https://github.com/enthought/mayavi) for further assisstance

## Running All Unit and Regression Tests
`python3 test/runTests.py all`

For more options: `python3 test/runTests.py -h`  
See below for how to run unit and regression tests individually  

## Running Unit Tests
**Note:** The Tests Github action runs these tests after every push to master
Unit Testing Framework Info:
https://docs.python.org/3/library/unittest.html
  
To run all tests: `python3 -m unittest -v`   
To run a single test module: `python3 -m unittest -v test.test_Vector`  
To run tests by module use this script (-h is to see the help message): `python3 test/runTests.py -h`  
All unit tests are run automatically whenever the master branch is updated, see [UnitTests.yml](https://github.com/henrystoldt/MAPLEAF/blob/master/.github/workflows/UnitTests.yml) to see how this works.

## Running Regression Testing / V & V Suite
All regression and V&V tests are defined in batch file [MAPLEAF/Examples/BatchSims/regressionTests.mapleaf](https://github.com/henrystoldt/MAPLEAF/blob/master/MAPLEAF/Examples/V&V/testDefinitions.mapleaf)  

To run them:
`mapleaf-batch MAPLEAF/Examples/BatchSims/regressionTests.mapleaf`  
For more info: `mapleaf-batch -h`

Shows results in console, generates plots in `./MAPLEAF/Examples/V&V/`  
Results of these simulations are automatically displayed on the [verification and validation section of the documentation website](https://henrystoldt.github.io/MAPLEAF/V&V/index.html)  
To see how this works, have a look at [generateDocs.yml](https://github.com/henrystoldt/MAPLEAF/blob/master/.github/workflows/generateDocs.yml)

## Debugging a Simulation (Visual Studio Code)
1. Place a breakpoint (red dot on the left)
2. Open the simulation definition file you want to run
3. On the left sidebar, click on the run tab (Bug + play button)
4. At the top of the sidebar, in the dropdown menu, select 'RSim Curr Sim Def', which will run whatever simulation definition file you currently have selected
5. Hit the green play button to the left of that dropdown

Program execution will pause at your breakpoint and you'll be able to inspect the values of variables. A mini window with debugging controls will pop up as well.

More details: https://code.visualstudio.com/docs/python/python-tutorial#_configure-and-run-the-debugger

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
2. Create environment: `$ mkvirtualenv -a ~/Documents/MAPLEAF -r ~/Documents/MAPLEAF/requirements_Dev.txt MAPLEAF`  
3. Environment will be automatically activated. To re-activate in the future: `$ workon MAPLEAF`  
4. To deactivate: `$ deactivate`  
5. To delete: `$ rmvirtualenv MAPLEAF`  
More commands here: https://virtualenvwrapper.readthedocs.io/en/latest/command_ref.html

## Profiling Performance with cProfile + Snakeviz
1. To generate profile: `$ python3 -m cProfile -o testProfile.prof MAPLEAF/Main.py /path/to/SimDefinition.mapleaf`
2. To view results: `$ python3 -m snakeviz testProfile.prof`

## Profiling Performance with pyInstrument
`pyinstrument -r html ./MAPLEAF/Main.py ./MAPLEAF/Examples/Simulations/Wind.mapleaf`

## Benchmarking with airspeed velocity (asv)
Benchmarks are defined in ./benchmarks/benchmarks.py  
Benchmarking settings defined in ./asv.conf.json  

#### Running benchmarks:
Run benchmarks on latest commit in the master branch: `$ asv run`  
Run benchmarks on new commits: `$ asv run NEW`  
Run benchmarks on commits in a branch: `$ asv run master..myBranch`  
To avoid running benchmarks for every single commit in a range of commits, use the `--steps NSTEPS` argument to specify how many times you would like to run the benchmark suite  
To see error messages produced while trying to run benchmarks: `$ asv dev`  

#### Viewing results:
To view the web interface (performance vs time/commits):  
1. `$ asv publish`  
2. `$ asv preview`  
3. Navigate to the IP address it provides you in your browser (often 127.0.0.1:8080)

To view available sets of results in the command line: `$ asv show`  
To view a single set of results: `$ asv show CommitHashHere`  

More information: https://asv.readthedocs.io/en/stable/using.html

## Recompiling Cython code
Some of the files in MAPLEAF are .pyx/.pxd instead of .py.  
These are [Cython](https://cython.org/) code files.  
Cython is a superset of Python, meaning all Python code is valid Cython.  

**Note:** Changes to the Cython code will not take effect without [re-compiling the Cython code](https://cython.readthedocs.io/en/latest/src/userguide/source_files_and_compilation.html). 
To re-compile: `$ python3 setup.py build_ext --inplace` or simply reinstall: `$ pip install -e .`

## Uploading to PyPI
(Must be done on Linux/Mac to preserve file cases)
Requires [twine](https://pypi.org/project/twine/)
1. Update the MAPLEAF version number in setup.py
2. `python setup.py sdist`
3. `twine upload dist/MAPLEAF-X.X.X.tar.gz`

## Linting (static error checking)
**Note:** The Linting Github action runs these checks after every push to master
1. `$ python3 -m flake8 --max-complexity 12`
2. Search for "import" in results (Powershell): `$ python3 -m flake8 --max-complexity 12 | findstr "import"`
3. Search for "import" in results (Bash): `$ python3 -m flake8 --max-complexity 12 | grep "import"`
4. Exact version GitHub auto-runs on commits to master (1 (check for show-stopper errors)): `python3 -m flake8 . --count --select=E9,F63,F7,F82 --show-source --statistics`
5. Exact version GitHub auto-runs on commits to master (2 (check for all errors)): `python3 -m flake8 . --count --exit-zero --max-complexity=10 --max-line-length=127 --statistics`

## Checking test coverage
**Note:** The Tests Github action runs this check after every push to master
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
