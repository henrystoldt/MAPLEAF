from timeit import timeit

#### Dot Product ####
nTests=10
print("Each operation performed {} times".format(nTests))

def getPythonNumpyPerformanceRatioForVectorLength(length):
    # Create setup string
    setup = '''
import numpy as np
from random import randint

len = {}

a = list(range(len))

def dotProd(list):
    list = sum([ x*x for x in list ])
    return list

nV1 = np.array(a)
    '''.format(length)
    
    pythonTime = timeit("b = dotProd(a)", setup=setup, number=nTests)
    numpyTime = timeit("nV2 = np.dot(nV1, nV1)", setup=setup, number=nTests)

    return pythonTime / numpyTime

lengthsToTry = list(range(1, 100, 1))
timeRatios = [ getPythonNumpyPerformanceRatioForVectorLength(l) for l in lengthsToTry]

import matplotlib.pyplot as plt

plt.plot(lengthsToTry, timeRatios, label="numpy")
plt.legend()
plt.xlabel("Array size")
plt.ylabel("Numpy speedup")
plt.title("Dot product")
plt.show()