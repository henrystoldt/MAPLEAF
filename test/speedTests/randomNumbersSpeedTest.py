from timeit import timeit

nTests=10000
print("Each operation performed {} times".format(nTests))


setup = '''
from numpy.random import normal
from random import gauss
'''
# Test Quaternion speed (init)
print("Numpy.random.normal")
print(timeit("a = normal()", setup=setup, number=nTests))

print("Random.gauss")
print(timeit("a = gauss(0, 1)", setup=setup, number=nTests))