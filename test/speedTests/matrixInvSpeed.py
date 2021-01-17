from timeit import timeit

nTests=100000

print("Each operation performed {} times".format(nTests))


print("\nNumpy linalg.inv")

setup = '''
import numpy as np

X1, X2 = 0.8, 1.4

AMatrix = \
np.array([  [ 1,    X1, X1**2,  X1**3 ],
            [ 1,    X2, X2**2,  X2**3 ],
            [ 0,    1,  2*X1, 3*X1**2 ],
            [ 0,    1,  2*X2, 3*X2**2 ] ])
'''

print(timeit("np.linalg.inv(AMatrix)", setup=setup, number=nTests))




print("\nSciPy linalg.inv")

setup = '''
import numpy as np
import scipy.linalg as sp

X1, X2 = 0.8, 1.4

AMatrix = \
np.array([  [ 1,    X1, X1**2,  X1**3 ],
            [ 1,    X2, X2**2,  X2**3 ],
            [ 0,    1,  2*X1, 3*X1**2 ],
            [ 0,    1,  2*X2, 3*X2**2 ] ])
'''

print(timeit("sp.inv(AMatrix)", setup=setup, number=nTests))

print("\nSciPy linalg.inv - list")

setup = '''
import numpy as np
import scipy.linalg as sp

X1, X2 = 0.8, 1.4

AMatrix = \
[   [ 1,    X1, X1**2,  X1**3 ],
    [ 1,    X2, X2**2,  X2**3 ],
    [ 0,    1,  2*X1, 3*X1**2 ],
    [ 0,    1,  2*X2, 3*X2**2 ] ]
'''

print(timeit("sp.inv(AMatrix)", setup=setup, number=nTests))