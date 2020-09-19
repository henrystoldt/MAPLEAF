import setuptools
from Cython.Build import cythonize
from pkg_resources import parse_requirements
from setuptools import Extension, setup

#### Get/Set info to be passed into setup() ####
with open("README.md", "r") as fh:
    long_description = fh.read()

with open("requirements.txt") as reqFile:
    lines = reqFile.readlines()
    install_reqs = list([ str(x) for x in parse_requirements(lines)])

MAPLEAFVersion = "0.8.8"

#### Create list of setuptools.Extension objects for Cython to compile ####
# Add Cython files here, together with ".c" if it compiles to Cython-Generated C code, or ".cpp" if it compiles to Cython-Generated C++ code
CythonFiles = [ [ "MAPLEAF/Motion/CythonVector.pyx", ".c" ],
                [ "MAPLEAF/Motion/CythonQuaternion.pyx", ".c" ],
                [ "MAPLEAF/Motion/CythonAngularVelocity.pyx", ".c" ],

                [ "MAPLEAF/Rocket/CythonFinFunctions.pyx", ".cpp" ],
]

def buildExtensionObjectsForCythonCode(CythonFilesList):
    extensions = []
    for cyModule in CythonFiles:
        pyxPath = cyModule[0]

        pythonImportPath = pyxPath.replace("/", ".")
        pythonImportPath = pythonImportPath.replace(".pyx", "")
        extensions.append( Extension(name=pythonImportPath, sources=[pyxPath]) )
    
    return extensions

setup(
    name='MAPLEAF',
    version=MAPLEAFVersion,
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

    ext_modules=cythonize( buildExtensionObjectsForCythonCode(CythonFiles), language_level="3" ),
    zip_safe=False,
    
    entry_points={
        'console_scripts': ['mapleaf = MAPLEAF.Main:main']
    }
)
