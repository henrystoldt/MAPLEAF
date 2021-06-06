from MAPLEAF.Motion.CythonVector cimport Vector

cdef class Quaternion:
    cdef public double Q0
    cdef public double Q1
    cdef public double Q2
    cdef public double Q3

    cpdef scaleRotation(self, double scalar)

    cpdef norm(self)

    cpdef dotProduct(self, Quaternion q2)

    cpdef normalize(self)

    cpdef conjugate(self)

    cpdef inverse(self)

    cpdef rotate(self, Vector vector)

    cpdef slerp(self, Quaternion quat2, double fraction)

    cpdef rotationAxis(self)

    cpdef rotationAngle(self)
    
    cpdef toEulerAngles(self)
    
    cpdef toRotationVector(self)