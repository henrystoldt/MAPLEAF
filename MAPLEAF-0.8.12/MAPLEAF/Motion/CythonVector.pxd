#Created by: Henry Stoldt
#August 2020

cdef class Vector:
    cdef public double X
    cdef public double Y
    cdef public double Z
    
    # def __init__(self, X, Y=None, Z=None)

    # def __getitem__(self, position)

    # def __format__(self, fmt_spec='')

    # def __str__(self)

    # def __add__(self, vector2)

    # def __sub__(self, vector2)

    # def __neg__(self)

    # def __eq__(self, vector2)

    # def __mul__(op1, op2)

    # def __truediv__(self, scalar)

    # def __len__(self)

    cpdef length(self)

    cpdef normalize(self)

    cpdef crossProduct(self, Vector vector2)

    cpdef angle(self, Vector vector2)
