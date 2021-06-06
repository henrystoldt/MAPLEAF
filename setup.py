import subprocess
import sys

import setuptools
from Cython.Build import cythonize
from pkg_resources import parse_requirements
from setuptools import Extension, setup

import MAPLEAF


#### Get/Set info to be passed into setup() ####
with open("README.md", "r") as fh:
    long_description = fh.read()

with open("requirements.txt") as reqFile:
    lines = reqFile.readlines()
    install_reqs = list([ str(x) for x in parse_requirements(lines)])

#### Create list of setuptools.Extension objects for Cython to compile ####
# Add Cython files here, together with ".c" if it compiles to Cython-Generated C code, or ".cpp" if it compiles to Cython-Generated C++ code
CythonFiles = [ [ "MAPLEAF/Motion/CythonVector.pyx", ".c" ],
                [ "MAPLEAF/Motion/CythonQuaternion.pyx", ".c" ],
                [ "MAPLEAF/Motion/CythonAngularVelocity.pyx", ".c" ],

                [ "MAPLEAF/Rocket/CythonFinFunctions.pyx", ".cpp" ],

                ["MAPLEAF/IO/CythonLog.pyx", ".c" ]
]

def buildExtensionObjectsForCythonCode(CythonFilesList):
    extensions = []
    for cyModule in CythonFiles:
        pyxPath = cyModule[0]

        pythonImportPath = pyxPath.replace("/", ".")
        pythonImportPath = pythonImportPath.replace(".pyx", "")
        extensions.append( Extension(name=pythonImportPath, sources=[pyxPath]) )
    
    return extensions

extensions = buildExtensionObjectsForCythonCode(CythonFiles)

#### If we are installing the package, try installing some optional packages that only sometimes work ####
installingMAPLEAF = "install" in sys.argv

if installingMAPLEAF:
    # ray and mayavi often cause issues on windows
    # PyQt5 only required for mayavi
    optionalProblematicPackages = [ "ray", "PyQt5", "mayavi" ]
    errorMessages = [
        "MAPLEAF will only run single-threaded.", 
        "", 
        "MAPLEAF will not produce 3D renders of the earth. Will fall back to Matplotlib instead."
    ]
        
    def tryInstallingPackage(packageName: str, errorMessage: str, install_reqs):
        ''' Function used to handle 'nice-to-have', but not required packages '''
        if packageName in install_reqs:
            install_reqs.remove(packageName)

            # Instead, try to install it here
            try:
                subprocess.check_call([sys.executable, "-m", "pip", "install", packageName])
                print("\nInstalled {}\n".format(packageName))
            
            except subprocess.CalledProcessError:
                # Output error, but continue installation if ray install fails
                    # Error message won't be visible unless running `python setup.py develop`
                print("\nWARNING: Unable to install {}. {}\n".format(packageName, errorMessage))

    for i in range(len(optionalProblematicPackages)):
        tryInstallingPackage(optionalProblematicPackages[i], errorMessages[i], install_reqs)

setup(
    name='MAPLEAF',
    version=MAPLEAF.__version__,
    author="Henry Stoldt",
    author_email="hhstoldt@ucalgary.ca",
    description="A compact, extensible rocket flight simulation framework for researchers and rocket designers",
    install_requires=install_reqs,
    license='MIT',
    long_description = long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/henrystoldt/MAPLEAF",
    packages=setuptools.find_packages(exclude=[ "test", "test.*", ]),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Developers",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Natural Language :: English",
        "Topic :: Scientific/Engineering :: Physics",
    ],
    include_package_data=True,
    test_suite='nose.collector',
    tests_require=['nose'],

    python_requires='>=3.6',

    ext_modules=cythonize(extensions, language_level="3" ),
    zip_safe=False,
    
    entry_points={
        'console_scripts': [ 
            'mapleaf = MAPLEAF.Main:main',
            'mapleaf-batch = MAPLEAF.SimulationRunners.Batch:main' ]
    }
)
