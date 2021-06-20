import sys
from timeit import timeit

import matplotlib.pyplot as plt
import numpy as np
# from numba import jit, njit
# from numba.typed import List
from statistics import mean
import ray
from multiprocessing import Pool

# Silences Numba warnings about a Python list being passed into a numba function
import warnings
warnings.filterwarnings('ignore')

#### OPTIONS ####
testCppFunctions = False #- set up for PyBind11 - only installed on one of my computers
testJuliaFunctions = True # - excessively slow - only installed on one of my computers
testCython = False
nTests=50

maxArraySize=1000
step=50
arrayLengths = list(range(1, maxArraySize, step))
#### END OPTIONS ####

#### Functions to test ####
def createPythonList(length):
    a = list(range(length))
    return [ float(x) for x in a ]

def createNumpyArray(length):
    pyList = createPythonList(length)
    return np.array(pyList, dtype=np.float64)

# def createNumbaTypedList(length):
#     pyList = createPythonList(length)
#     typedList = List()
#     [ typedList.append(x) for x in pyList ]
#     return typedList

def addFive_Python(array):
    return [ x+5.0 for x in array ]

@ray.remote
def addFive_Python_RaySub(array):
    return [ x+5.0 for x in array ]

def addFive_Python_Ray(array):
    chunk = round(len(array)/2)
    future1 = addFive_Python_RaySub.remote(array[:chunk])
    future2 = addFive_Python_RaySub.remote(array[chunk:])
    return ray.get(future1) + ray.get(future2)

def addFive_Python_Multiprocessing(array):
    chunk = round(len(array)/2)
    with Pool(2) as p:
        results = p.map(addFive_Python, [ array[:chunk], array[chunk:]])
    return results[0] + results[1]

# @njit()
def addFive_Numba(array):
    for i in range(len(array)):
        array[i] += 5.0
    return array

def addFive_Numpy(array):
    array += 5.0
    return array

# Assume the Python function will be imported from this file
pythonBenchmarkSetupString = """
from addScalarToArray import {}, {}
pyList = {}({})
"""

# Assume Comparison function list generator will be imported from this file
# Comparison function can be imported from any module
comparisonBenchmarkSetupString = """
from addScalarToArray import {}
from {} import {}
comparisonArray = {}({})
# Call function once to pre-compile it for JIT methods like numba
{}(comparisonArray)
"""

#### Functions that do the testing / result plotting ####
def getSpeedup(length, comparisonFunction, comparisonFunctionListGenerator, comparisonFnModule, pythonFunction, pythonListGenerator):
    setupPython = pythonBenchmarkSetupString.format(pythonFunction, pythonListGenerator, pythonListGenerator, length)
    setupComparison = comparisonBenchmarkSetupString.format(comparisonFunctionListGenerator, comparisonFnModule, comparisonFunction.__name__, comparisonFunctionListGenerator, length, comparisonFunction.__name__)

    pythonTime = timeit("{}(pyList)".format(pythonFunction), setup=setupPython, number=nTests)
    fnTime = timeit("{}(comparisonArray)".format(comparisonFunction.__name__), setup=setupComparison, number=nTests)

    return pythonTime/fnTime

def plotSpeedupForEachArrayLength(function, label="Unlabelled", comparisonFunctionListGenerator="createNumpyArray", comparisonFnModule="addScalarToArray", pythonFunction="addFive_Python", pythonListGenerator="createPythonList"):
    ratios = [ getSpeedup(l, function, comparisonFunctionListGenerator, comparisonFnModule, pythonFunction, pythonListGenerator) for l in arrayLengths ]
    print("Speedup {:<40}: {:>6.2f}, {:>6.2f}, {:>6.2f}".format("("+label+")", min(ratios), max(ratios), mean(ratios)))

    # Plot result, with different line styles depending on which data type is being operated on
    if "ndarray" in label:
        plt.plot(arrayLengths, ratios, linestyle="dashed", label=label)
    elif "numba.typed.List" in label:
        plt.plot(arrayLengths, ratios, linestyle="dotted", label=label)
    else:
        plt.plot(arrayLengths, ratios, label=label)

if testJuliaFunctions:
    import julia
    from julia import Main
    Main.include("addScalar.jl")
    # function to test is Main.addFive - seems very slow, must be converting types or not being compiled
    addFive_Julia = Main.addFive_Julia

#### Main ####
if __name__ == "__main__":
    print("Each operation performed {} times".format(nTests))
    print("Speedup {:<40}: {:>6}, {:>6}, {:>6}".format("", "Min", "Max", "Mean"))
    plotSpeedupForEachArrayLength(addFive_Python, label="Python loop: ndarray")
    plotSpeedupForEachArrayLength(addFive_Python, label="Python loop: list", comparisonFunctionListGenerator="createPythonList")
    plotSpeedupForEachArrayLength(addFive_Numpy, label="numpy +=: ndarray")

    # plotSpeedupForEachArrayLength(addFive_Numba, label="numba: list", comparisonFunctionListGenerator="createPythonList")
    # plotSpeedupForEachArrayLength(addFive_Numba, label="numba: numba.typed.List", comparisonFunctionListGenerator="createNumbaTypedList")
    # plotSpeedupForEachArrayLength(addFive_Numba, label="numba: ndarray")

    if testCython:
        # Must have compiled the 'addScalarCython.pyx' file on your machine using Cython to make this work
        # Run `cythonize addScalar.pyx`
        # https://cython.readthedocs.io/en/latest/MAPLEAF/tutorial/cython_tutorial.html
        import addScalarCython
        plotSpeedupForEachArrayLength(addScalarCython.addFive_Numpy, comparisonFnModule="addScalarCython",  label="Cython - strongly-typed: ndarray")
        plotSpeedupForEachArrayLength(addScalarCython.addFive_Plain, comparisonFnModule="addScalarCython",  label="Cython - Plain Python: list", comparisonFunctionListGenerator="createPythonList")
        plotSpeedupForEachArrayLength(addScalarCython.addFive_Plain, comparisonFnModule="addScalarCython",  label="Cython - Plain Python: ndarray")

    # Too slow
    # plotSpeedupForEachArrayLength(addFive_Python_Multiprocessing, label="Multiprocessing: 2 cores")

    # Also very slow, but less so because processes are only launched once
    # ray.init()
    # plotSpeedupForEachArrayLength(addFive_Python_Ray, label="Ray: 2 cores")

    if testCppFunctions:
        import example
        #Functions to test are:
        example.addToList_Cpp # - (converts to std::vector and back)
        # example.addFive(nV1) # - (no conversion, loops in C++)
        example.vectorizedAddFive #- (C++ function wrapped with py::vectorize to work on an array)

    if testJuliaFunctions:
        plotSpeedupForEachArrayLength(Main.addFive_Julia, label="Julia, ndarray")
        plotSpeedupForEachArrayLength(Main.addFive_Julia, label="Julia, list", comparisonFunctionListGenerator="createPythonList")

    plt.xlabel("Array size")
    plt.ylabel("Speedup")
    plt.title("Elementwise adding a scalar to an array of float64")
    
    plt.autoscale(tight=True, axis="y")
    plt.xlim([0, maxArraySize])
    # plt.yscale("log")

    plt.legend()
    plt.show()
