#Created by: Henry Stoldt
#January 2019

#Use objects of this class to contain data for all vector information:
#   Velocities, positions, directions etc...

import numbers
from libc.math cimport acos, sqrt
from libc.stdlib cimport atof

__all__ = [ "Vector" ]

cdef class Vector:
    
    def __init__(self, X, Y=None, Z=None):
        ''' 
            Can be initialized by passing in three numeric components or a single string (ex: "(0, 1, 2)" ) containing numeric values separated by ', ' or ' ', or ';', or '; '  and enclosed by () or {} or [] or no brackets.
            To initialize from an iterable, unpack the iterable into the constructor with an asterisk:
                a = np.array([1,2,3])
                b = Vector(*a)
        '''
        try:
            # Initialize from components
            self.X = X
            self.Y = Y
            self.Z = Z
        except TypeError:
            # Try parsing X as a string
            #Remove leading/trailing spaces/brackets
            origX = X
            X = X.strip()
            if X[0] in [ '[', '(', '{' ]:
                X = X[1:-1].strip()
            
            #Split into components
            splitX = X.split(", ")
            if len(splitX) == 1:
                splitX = splitX[0].split("; ")
            if len(splitX) == 1:
                splitX = splitX[0].split()
            if len(splitX) == 1:
                splitX = splitX[0].split(",")
            if len(splitX) == 1:
                splitX = splitX[0].split(";")

            #Store result
            if len(splitX) == 3:
                self.X = float(splitX[0])
                self.Y = float(splitX[1])
                self.Z = float(splitX[2])
            else:
                raise ValueError("'" + origX + "' Was not successfully parsed into Vector format")

    def __getitem__(self, int position):
        ''' Add ability to access X,Y,Z via index '''
        if position == 0:
            return self.X
        elif position == 1:
            return self.Y
        elif position == 2:
            return self.Z
        else:
            raise IndexError("Index {} > 2".format(position))

    def __format__(self, str fmt_spec=''):
        ''' Allow printing Vectors with .format() '''
        components = (format(c, fmt_spec) for c in [self.X, self.Y, self.Z])
        return "{} {} {}".format(*components)

    def __str__(self):
        ''' Controls what is printed by print() '''
        return "({} {} {})".format(self.X, self.Y, self.Z)

    def __add__(self, vector2):
        ''' Addition operator + '''
        if isinstance(vector2, Vector):
            return Vector(self.X+vector2.X, self.Y+vector2.Y, self.Z+vector2.Z)
        elif isinstance(vector2, numbers.Number):
            return Vector(self.X+vector2, self.Y+vector2, self.Z+vector2)
        else:
            raise NotImplementedError("Only other vectors or scalars can be added to Vectors.")

    def __sub__(self, vector2):
        ''' Subtraction operator '''
        return self.__add__(-vector2)

    def __neg__(self):
        return Vector(-self.X, -self.Y, -self.Z)

    def __eq__(self, vector2):
        if isinstance(vector2, Vector):
            return self.X == vector2.X and self.Y == vector2.Y and self.Z == vector2.Z
        else:
            return False

    def __mul__(op1, op2):
        ''' Multiplication operator * '''
        if type(op1) == type(op2):
            #If also a vector, return the dot product
            return op1.X*op2.X + op1.Y*op2.Y + op1.Z*op2.Z
        elif isinstance(op2, numbers.Number):
            #If a scalar, multiply elementwise
            return Vector( op1.X*op2, op1.Y*op2, op1.Z*op2 )
        elif isinstance(op1, numbers.Number):
            return Vector( op1*op2.X, op1*op2.Y, op1*op2.Z )
        else:
            raise NotImplementedError("Vectors can only be multiplied by other vectors (dot product), or scalars (elementwise multiplication).")

    def __truediv__(self, scalar):
        ''' Division operator / '''
        return self * (1/scalar)

    def __len__(self):
        return 3

    cpdef length(self):
        return sqrt(self.X*self.X + self.Y*self.Y + self.Z*self.Z)

    cpdef normalize(self):
        ''' Returns a unit vector pointing in the same direction as the current Vector '''
        l = self.length()
        if l > 0:
            return Vector( self.X/l, self.Y/l, self.Z/l )
        else:
            return Vector(0,0,0)

    cpdef crossProduct(self, Vector vector2):
        return Vector(  self.Y*vector2.Z - self.Z*vector2.Y,
                        self.Z*vector2.X - self.X*vector2.Z,
                        self.X*vector2.Y - self.Y*vector2.X )

    
    cpdef angle(self, Vector vector2):
        ''' Returns the angle between two vectors, always <= 180 degrees '''
        interior = (self * vector2) / (self.length() * vector2.length())
        if abs(interior - 1) < 0.00000001:
            return acos(1)
        else:
            return acos(interior)
