from timeit import timeit

nTests=10000
print("Each operation performed {} times".format(nTests))


print("")
print("Custom Quaternion")
print("")

importQuatVec = '''
from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import Vector
v1 = Vector(1, 1, 2)
'''
# Test Quaternion speed (init)
print("Initializing Quaternion (Axis-Angle):")
print(timeit("a = Quaternion(axisOfRotation=v1, angle=1.2)", setup=importQuatVec, number=nTests))

print("Initializing Quaternion (Components):")
print(timeit("a = Quaternion(components=[1, 1.2, 2.3, 4.5])", setup=importQuatVec, number=nTests))



setupRotQuat = '''
from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import Vector

qRot = Quaternion(axisOfRotation=Vector(1, 1, 2), angle=1.2)
vec = Vector(1, 2, 3)
'''
# Test Quaternion speed (rotate)
print("Quaternion Rotating Vector:")
print(timeit("a = qRot.rotate(vec)", setup=setupRotQuat, number=nTests))

print("")
print("Scipy")
print("")

setupScipyRot = '''
from scipy.spatial.transform import Rotation as R
from MAPLEAF.Motion import Vector
v1 = list(Vector(1, 1, 2).normalize() * 1.2)
'''
# Test Scipy speed (init)
print("Initializing Scipy Rotation (Rotation Vector):")
print(timeit("a = R.from_rotvec(v1)", setup=setupScipyRot, number=nTests))

setupScipyRot = '''
from scipy.spatial.transform import Rotation as R
from MAPLEAF.Motion import Vector
v1 = list(Vector(1, 1, 2).normalize() * 1.2)
sRot = R.from_rotvec(v1)
vec = [1, 2, 3]
'''
# Test Scipy speed (rotation)
print("Scipy Rotating Vector:")
print(timeit("a = sRot.apply(vec)", setup=setupScipyRot, number=nTests))


print("")
print("Custom Vector")
print("")

setup = '''
from MAPLEAF.Motion import Vector

import numpy as np
a = [1,2,3]
v1 = Vector(1,2,3)
v2 = Vector(2,3,4)
nV1 = np.array([1,2,3])
nV2 = np.array([2,3,4])
'''
print("Initializing Vector (Components):")
print(timeit("v1 = Vector(1, 1, 2)", setup=setup, number=nTests))

print("Initializing Vector (list):")
print(timeit("v1 = Vector(*a)", setup=setup, number=nTests))

print("Initializing Vector (String):")
print(timeit("v1 = Vector('(1 1 2)')", setup=setup, number=nTests))

print("Dot Product:")
print(timeit("v3 = v1 * v2", setup=setup, number=nTests))

print("Cross Product:")
print(timeit("v1.crossProduct(v2)", setup=setup, number=nTests))

print("")
print("Numpy Vector")
print("")

print("Initializing Vector (Components):")
print(timeit("v1 = np.array([1,2,3])", setup=setup, number=nTests))

print("Dot Product:")
print(timeit("v3 = np.dot(nV1, nV2)", setup=setup, number=nTests))

print("Cross Product:")
print(timeit("v3 = np.cross(nV1, nV2)", setup=setup, number=nTests))