# Created by: Henry Stoldt
# June 2020 - moved from Quaternion.py

import numbers
from MAPLEAF.Motion.CythonQuaternion cimport Quaternion
from MAPLEAF.Motion.CythonVector cimport Vector

__all__ = [ "AngularVelocity" ]

cdef class AngularVelocity(Vector):
    '''
        Uses a vector to represent an angular velocity
    '''

    # Either provide axis and angularVelocity or a rotationVector, where the length represents the angularVelocity
    def __init__(self, axisOfRotation=None, angularVel=None, rotationVector=None):
        '''
            Initialize from axis-angle by passing in a Vector for axisOfRotation, and a numeric value for angularVel
            Can also pass in the components of a rotationVector, treating this function as __init__(X, Y, Z)
        '''
        try:
            # Initialize from components if all components are floats
            X = float(axisOfRotation)
            Y = float(angularVel)
            Z = float(rotationVector)

            self.X = X
            self.Y = Y
            self.Z = Z
        except:
            if axisOfRotation != None and angularVel != None:
                # Initialize from axis-angle
                rotationVector = axisOfRotation.normalize() * angularVel
                self.X = rotationVector.X
                self.Y = rotationVector.Y
                self.Z = rotationVector.Z

            elif rotationVector != None:
                # Initialize from rotationVector
                self.X = rotationVector.X
                self.Y = rotationVector.Y
                self.Z = rotationVector.Z

            else:
                ValueError("Not enough information provided to initialize the AngularVelocity class")

    def __mul__(op1, op2):
        #If also a vector, return the dot product
        if type(op1) == type(op2):
            return op1.X*op2.X + op1.Y*op2.Y + op1.Z*op2.Z
        
        #If a scalar, multiply elementwise
        elif isinstance(op2, numbers.Number):
            return AngularVelocity( op1.X*op2, op1.Y*op2, op1.Z*op2 )
        elif isinstance(op1, numbers.Number):
            return AngularVelocity( op1*op2.X, op1*op2.Y, op1*op2.Z )
        
        # Multiply by Quaternion
        elif isinstance(op1, Quaternion):
            return op1 * op2.toQuaternion()
        elif isinstance(op2, Quaternion):
            return op1.toQuaternion() * op2
        
        else:
            raise NotImplementedError("Vectors can only be multiplied by other vectors (dot product), or scalars (elementwise multiplication).")

    def __add__(self, vector2):
        if isinstance(vector2, Vector):
            return AngularVelocity(self.X+vector2.X, self.Y+vector2.Y, self.Z+vector2.Z)
        elif isinstance(vector2, numbers.Number):
            return AngularVelocity(self.X+vector2, self.Y+vector2, self.Z+vector2)
        else:
            raise NotImplementedError("Only other vectors or scalars can be added to Vectors.")
    
    def __neg__(self):
        return AngularVelocity(-self.X, -self.Y, -self.Z)
    
    cpdef toQuaternion(self):
        ''' After an angular velocity has been multiplied by a timestep (integrated), used to convert it to a Quaternion representing a rotation over a timestep '''
        return Quaternion(axisOfRotation=self.rotationAxis(), angle=self.angVel())

    cpdef angVel(self):
        return self.length()

    cpdef rotationAxis(self):
        return self.normalize()
