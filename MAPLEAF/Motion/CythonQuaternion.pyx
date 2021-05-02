#Created by: Henry Stoldt
#January 2019

import numbers
from libc.math cimport atan2, asin, cos, pi, sin, sqrt

from MAPLEAF.Motion.CythonVector import Vector

__all__ = [ 'Quaternion' ]

# TODO: Restrict magnitude more strictly to 1? -> Perhaps make child rotation class in which the quaternion magnitude is always 1.

cdef class Quaternion:
    '''
        Use objects of this class to store orientation information - direction and rotation
            Elements are: [ scalar, xi, yj, zk ] where scalar, x, y, and z are scalars
            i,j, and k squared = -1
            ij=k ji=-k
            jk=i kj=-i
            ki=j ik=-j

        To define the rotation quaternion between two frames:
        If Q(2) = Q(1) * Q(1-2)
        Q(1)^-1 * Q(2) = Q(1)^-1 * Q(1) * Q(1-2)
        Q(1)^-1 * Q(2) = Q(1-2) ------------- where Q(1-2) defines the rotation from frame 1 to 2 (as defined in the first step)
        Q(2-1) = Q(1-2).conjugate

        Quaternion normalization is not enforced by this class, but quaternions do not represent a rotation unless they are normalized.
        Quaternions are automatically normalized if this class is constructed using the angle/axis terms and if .rotate() is used.

        Future: Could increase rotation performance by ~50% by converting to a rotation matrix
            Only worth it if we rotate many vectors per quaternion

        A positive angle of rotation is clockwise if we are looking down the defined rotation axis

        Class definition in CythonQuaternion.pxd
    '''
    def __init__(self, axisOfRotation=None, angle=None, components=None, Q3=None):
        '''
            Can initialize in one of three ways:
            1. orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=(pi)) - angle should be in radians
            2. orientation = Quaternion(components=[0,1,0,1]) NOTE: Deprecated - will be removed
            3. orientation = Quaternion(0, 1, 0, 1) - Fastest approach
        '''
        if Q3 != None:
            self.Q0 = axisOfRotation
            self.Q1 = angle
            self.Q2 = components
            self.Q3 = Q3
        elif axisOfRotation != None and angle != None:
            #Automatically converts axis of rotation vector to a unit vector
            axisOfRotation = axisOfRotation.normalize()
            self.Q0 = cos(angle/2)
            self.Q1 = sin(angle/2) * axisOfRotation.X
            self.Q2 = sin(angle/2) * axisOfRotation.Y
            self.Q3 = sin(angle/2) * axisOfRotation.Z
        elif components != None:
            # TODO: Remove this option
            self.Q0 = components[0]
            self.Q1 = components[1]
            self.Q2 = components[2]
            self.Q3 = components[3]
        else:
            raise ValueError("Not enough Initialization info provided")

    def __getitem__(self, int position):
        ''' Add ability to access components by index '''
        if position == 0:
            return self.Q0
        elif position == 1:
            return self.Q1
        elif position == 2:
            return self.Q2
        elif position == 3:
            return self.Q3
        else:
            raise IndexError("Index {} > 3".format(position))
    
    def __len__(self):
        return 4

    def __str__(self):
        """Print Statement"""
        return "<" + str(self.Q0) + ", " + str(self.Q1) + ", " + str(self.Q2) + ", " + str(self.Q3) + ">"

    def __format__(self, fmt_spec=''):
        components = (format(c, fmt_spec) for c in [self.Q0, self.Q1, self.Q2, self.Q3])
        return "{} {} {} {}".format(*components)

    def __add__(self, q2):
        return Quaternion(self.Q0+q2.Q0, self.Q1+q2.Q1, self.Q2+q2.Q2, self.Q3+q2.Q3)

    def __sub__(self, q2):
        return self + (-q2)

    def __neg__(self):
        return Quaternion(-self.Q0, -self.Q1, -self.Q2, -self.Q3)

    def __eq__(self, q2):
        return (self.Q0 == q2.Q0 and self.Q1 == q2.Q1 and self.Q2 == q2.Q2 and self.Q3 == q2.Q3)

    def __mul__(q1, q2):
        """Multiplication operator *
            Multiplying two quaternions adds up their rotations
            Multiplications are not commutative, are applied in order from left to right
        """
        if type(q2) == type(q1): # Quaternion - Quaternion case
            return Quaternion(  q1.Q0*q2.Q0 - q1.Q1*q2.Q1 - q1.Q2*q2.Q2 - q1.Q3*q2.Q3,
                                q1.Q0*q2.Q1 + q1.Q1*q2.Q0 + q1.Q2*q2.Q3 - q1.Q3*q2.Q2,
                                q1.Q0*q2.Q2 - q1.Q1*q2.Q3 + q1.Q2*q2.Q0 + q1.Q3*q2.Q1,
                                q1.Q0*q2.Q3 + q1.Q1*q2.Q2 - q1.Q2*q2.Q1 + q1.Q3*q2.Q0 )
        elif isinstance(q2, numbers.Number):
            return Quaternion( q1.Q0*q2, q1.Q1*q2, q1.Q2*q2, q1.Q3*q2 )
        elif isinstance(q1, numbers.Number):
            return Quaternion( q2.Q0*q1, q2.Q1*q1, q2.Q2*q1, q2.Q3*q1 )
        try: # Angular velocity case
            return q1 * q2.toQuaternion()
        except AttributeError:
            raise NotImplementedError("Quaternions can only be multiplied by 1) Quaternions, 2) Scalars, or 3) AngularVelocities (which have been 'integrated' through multiplication by a scalar timestep)")

    def __truediv__(self,scalar):
        ''' Division operator / '''
        if isinstance(scalar, Quaternion):
            return scalar.inverse() * self
        else:
            return self * (1.0/scalar)

    cpdef scaleRotation(self, double scalar):        
        rotAngle = self.rotationAngle()
        if rotAngle == 0:
            return self
        
        rotAxis = self.rotationAxis()
        return Quaternion(rotAxis, scalar * rotAngle)

    cpdef norm(self):
        ''' Get the norm of the quaternion '''
        return sqrt(self.Q0*self.Q0 + self.Q1*self.Q1 + self.Q2*self.Q2 + self.Q3*self.Q3)

    cpdef dotProduct(self, Quaternion q2):
        ''' Used to compute the angle parameter in spherical linear interpolation '''
        return self.Q0*q2.Q0 + self.Q1*q2.Q1 + self.Q2*q2.Q2 + self.Q3*q2.Q3 

    cpdef normalize(self):
        ''' Create unit quaternion '''
        n = self.norm()
        if n == 1.0:
            return self
        else:
            self.Q0 = self.Q0 / n
            self.Q1 = self.Q1 / n
            self.Q2 = self.Q2 / n
            self.Q3 = self.Q3 / n
            return self

    cpdef conjugate(self):
        ''' Negate all complex terms - produces an inverse rotation if the quaternion is a unit quaternion '''
        return Quaternion(self.Q0, -self.Q1, -self.Q2, -self.Q3)

    cpdef inverse(self):
        ''' Conjugate divided by self.norm '''
        return self.conjugate().__truediv__(self.norm()**2)

    cpdef rotate(self, Vector vector):
        ''' Use the quaternion to rotate a vector '''
        rotQ = self.normalize()
        QuaternionVec = Quaternion(0, vector.X, vector.Y, vector.Z)
        qRotated = rotQ * QuaternionVec * rotQ.conjugate()
        return Vector(qRotated.Q1, qRotated.Q2, qRotated.Q3)

    cpdef slerp(self, Quaternion quat2, double fraction):
        ''' Spherical Linear Interpolation - fraction = (0-1) where 0 is self, 1 is quat2 '''        
        #Normalize both vectors
        q1 = self.normalize()
        q2 = quat2.normalize()

        #Calculate the quaternion which would rotate q1 to become q2 (q1 * q1^-1 * q2 = q2)
        qDelta = q1.conjugate() * q2
        
        #If the rotation has ended up going the long way around, go the short way
        rotAngle = qDelta.rotationAngle()
        if rotAngle > pi:
            rotationAngle = (2*pi - rotAngle) * fraction
        else:
            rotationAngle = rotAngle * fraction

        if rotationAngle > 0:
            rotationAxis = qDelta.rotationAxis()
        else:
            rotationAxis = Vector(1,0,0)
        return Quaternion(axisOfRotation=rotationAxis, angle=rotationAngle)

        #More efficient, less intuitive implementation below:
        #angle = 2*acos(q1 * q2)
        #return q1*(sin((1-fraction)*angle)/sin(angle)) + q2*(sin(fraction*angle)/sin(angle))

    cpdef rotationAxis(self):
        ''' Returns the axis about which this quaternion would rotate a vector '''
        if abs(self.rotationAngle()) > 0:
            n = sqrt(self.norm() - self.Q0**2)
            return Vector(self.Q1/n, self.Q2/n, self.Q3/n)
        else:
            return Vector(1,0,0)

    cpdef rotationAngle(self):
        ''' Returns the angle (radians) by which this quaternion rotates vectors about the rotationAxis '''
        quat = self.normalize()
        n = sqrt(1 - quat.Q0**2)
        return 2 * atan2(n,quat.Q0)

    cpdef toEulerAngles(self):
        ''' Returns Tait-Bryan 3-2-1, z-y-x convention Euler Angles Vector '''
        cdef double xAngle = atan2(2*(self.Q0*self.Q1 + self.Q2*self.Q3), 1-2*(self.Q1**2 + self.Q2**2))
        cdef double yAngle = asin(2*(self.Q0*self.Q2 - self.Q3*self.Q1))
        cdef double zAngle = atan2(2*(self.Q0*self.Q3 + self.Q1*self.Q2), 1-2*(self.Q2**2 + self.Q3**2))
        
        return Vector(xAngle, yAngle, zAngle)
    
    cpdef toRotationVector(self):
        quat = self.normalize()
        n = sqrt(1 - quat.Q0**2)
        angle = 2 * atan2(n,quat.Q0)
        if angle == 0:
            return Vector(0,0,0)
        else:
            return Vector(self.Q1*angle/n, self.Q2*angle/n, self.Q3*angle/n)

    def plotRotation(self, showPlot=True):
        import matplotlib.pyplot as plt
        import mpl_toolkits.mplot3d.axes3d as p3
        L1 = 3
        RefOrientation = Vector(0,0,L1)
        RefOrientation2 = Vector(1,0,0)
        TransformedOrientation = self.rotate(RefOrientation)
        TransformedOrientation2 = self.rotate(RefOrientation2)

        lines = [ RefOrientation, RefOrientation2, TransformedOrientation, TransformedOrientation2 ]

        refLineData = [ [], [], [] ]
        for line in lines[:2]:
            refLineData[0].append(0)
            refLineData[1].append(0)
            refLineData[2].append(0)

            refLineData[0].append(line[0])
            refLineData[1].append(line[1])
            refLineData[2].append(line[2])

        transformedLineData = [ [], [], [] ]
        for line in lines[2:]:
            transformedLineData[0].append(0)
            transformedLineData[1].append(0)
            transformedLineData[2].append(0)

            transformedLineData[0].append(line[0])
            transformedLineData[1].append(line[1])
            transformedLineData[2].append(line[2])

        fig = plt.figure()
        ax = p3.Axes3D(fig)
        line1 = ax.plot(refLineData[0], refLineData[1], refLineData[2], label="Reference Orientation")
        line2 = ax.plot(transformedLineData[0], transformedLineData[1], transformedLineData[2], label="Transformed Orientation")

        # Setting the axes properties
        ax.set_xlim3d([-L1, L1])
        ax.set_xlabel('X')

        ax.set_ylim3d([-L1, L1])
        ax.set_ylabel('Y')

        ax.set_zlim3d([-L1, L1])
        ax.set_zlabel('Z')

        ax.set_title('Quaternion Rotation')

        if showPlot:
            plt.show()

if __name__ == '__main__':
    q = Quaternion(axisOfRotation=Vector(0,1,0), angle=pi/4)
    q2 = Quaternion(axisOfRotation=Vector(1,0,0), angle=pi/2)
    q3 = q2 * q
    q3.plotRotation()
