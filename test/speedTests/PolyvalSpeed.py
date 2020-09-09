from timeit import timeit

nTests=10000

print("Each operation performed {} times".format(nTests))


print("")
print("Numpy Polyval")
print("")

setup = '''
from numpy import polyval, array
x = array([-0.10334193,  0.84130815, -2.59678816,  3.65668744, -2.09745604, 0.65980236])

print("Result = {}".format(polyval(x, 2)))
'''
print("Evaluate polynomial")
print(timeit("a = polyval(x, 2)", setup=setup, number=nTests))

print("")
print("Custom")
print("")

setup = '''
x = [-0.10334193,  0.84130815, -2.59678816,  3.65668744, -2.09745604, 0.65980236]

sum = 0
l = len(x)
for i in range(l):
    sum += x[i]*2**(l-1-i)
print("Result = {}".format(sum))
'''
code = '''
sum = 0
l = len(x)
for i in range(l):
    sum += x[i]*2**(l-1-i)
'''
print("Evaluate polynomial")
print(timeit(code, setup=setup, number=nTests))