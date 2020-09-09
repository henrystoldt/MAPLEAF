import numpy
from setuptools import setup

from Cython.Build import cythonize

setup(ext_modules=cythonize("addScalarCython.pyx"), include_dirs=[numpy.get_include()])
