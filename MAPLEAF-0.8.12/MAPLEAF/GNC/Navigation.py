''' Navigators determine where to go '''

import abc

from MAPLEAF.Motion import Quaternion, Vector

__all__ = [ "Navigator", "Stabilizer" ]

class Navigator(abc.ABC):
    @abc.abstractmethod
    def getTargetOrientation(self, rocketState, targetState, time):
        ''' Should return a target orientation quaternion '''
        pass

class Stabilizer(Navigator):
    def __init__(self, desiredFlightDirection):
        # Calculate target orientation
        direction = desiredFlightDirection.normalize()
        angleFromVertical = Vector(0,0,1).angle(direction)
        rotationAxis = Vector(0,0,1).crossProduct(direction)
        self.targetOrientation = Quaternion(rotationAxis, angleFromVertical)

    def getTargetOrientation(self, rocketState, targetState, time):
        return self.targetOrientation
