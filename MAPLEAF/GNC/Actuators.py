'''
Modelling of actuators, which can control things like the position of rocket fin angles...
'''

import abc
from math import e

import numpy as np
from scipy.interpolate import LinearNDInterpolator

from MAPLEAF.Motion import AeroParameters

__all__ = [ "TableInterpolatingActuatorController", "FirstOrderActuator", "FirstOrderSystem", "ActuatorController", "Actuator" ]

class FirstOrderSystem():
    def __init__(self, responseTime):
        self.responseTime = responseTime

    def getPosition(self, currentTime, lastTime, lastPosition, lastTarget):
        dt = currentTime - lastTime
        if dt < -1e-17: # Exception check here because this would pass through the exponent expression below without causing an error
            raise ValueError("Current time ({}) must be after lastTime ({})".format(currentTime, lastTime))
            
        if dt < 0:
            dt = 0

        lastError = lastTarget - lastPosition
        errorFractionRemoved = (1 - e**(-dt/self.responseTime))
        newPosition = lastPosition + lastError*errorFractionRemoved

        return newPosition


class ActuatorController(abc.ABC):
    ''' 
        Interface for actuator controllers.   
        Actuator controllers are in charge of determining what actuator deflections are required to produce moments desired by the rocket's `MAPLEAF.GNC.ControlSystems.ControlSystem`
    '''
    @abc.abstractmethod
    def setTargetActuatorDeflections(self, desiredMoments, state, environment, time):
        ''' Should return nothing, control the appropriate rocket system to generate the desired moments '''
        pass

class TableInterpolatingActuatorController(ActuatorController):
    '''
        Controls actuators 
    '''
    def __init__(self, deflectionTableFilePath, nKeyColumns, keyFunctionList, actuatorList):
        self.actuatorList = actuatorList
        self.keyFunctionList = keyFunctionList

        # Load actuator-deflection data table
        deflData = np.loadtxt(deflectionTableFilePath, skiprows=1)
        keys = deflData[:,0:nKeyColumns]
        deflData = deflData[:,nKeyColumns:]

        # Check that number of actuators matches number of deflection entries in the deflection tables
        nActuators = len(actuatorList)
        nDeflectionTableEntries = len(deflData[0,:])
        if nActuators != nDeflectionTableEntries:
            raise ValueError("Number of actuators: {}, must match number of actuator deflection columns in deflection table: {}".format(nActuators, nDeflectionTableEntries))

        # Create interpolation function for fin deflections
        self._getPositionTargets = LinearNDInterpolator(keys, deflData)

    def setTargetActuatorDeflections(self, desiredMoments, state, environment, time):
        '''
            Inputs:
                desiredMoments: (3-length iterable) contains desired moments about the x,y,and z axes
                time:           time at which the moments are requested (time of current control loop execution)
        '''
        # Construct key vector, starting with non-moment components:
        keyVector = AeroParameters.getAeroPropertiesList(self.keyFunctionList, state, environment)
        for desiredMoment in desiredMoments:
            keyVector.append(desiredMoment)

        newActuatorPositionTargets = self._getPositionTargets(*keyVector)

        for i in range(len(self.actuatorList)):
            self.actuatorList[i].setTargetDeflection(newActuatorPositionTargets[i], time)

        return list(newActuatorPositionTargets)




class Actuator(abc.ABC):
    ''' 
        Interface for actuators.
        Actuators model the movement of a physical actuator (ex. servo, hydraulics).
        Target deflections are usually provided by the rocket's `MAPLEAF.GNC.ControlSystems.ControlSystem`, usually obtained from a `ActuatorController`
    '''
    @abc.abstractmethod
    def setTargetDeflection(self, newTarget, time):
        ''' Should return nothing, update setPoint for the actuator '''
        pass

    @abc.abstractmethod
    def getDeflection(self, time):
        ''' Should return the current deflection '''
        pass

    @abc.abstractmethod
    def getDeflectionDerivative(self, state, environment, time):
        ''' Should return the current deflection derivative '''
        pass

class FirstOrderActuator(Actuator, FirstOrderSystem):
    '''
        First-order system - simple enought that its motion between control loops can be determined analytically.
        Motion does not need to be integrated

        Basically just a version of the FirstOrderSystem class that internally stores state information
    '''
    def __init__(self, responseTime, initialDeflection=0, initialTime=0, maxDeflection=None, minDeflection=None):
        self.responseTime = responseTime
        self.lastDeflection = initialDeflection
        self.lastTime = 0
        self.targetDeflection = 0
        self.maxDeflection = maxDeflection
        self.minDeflection = minDeflection

    def setTargetDeflection(self, newTarget, time):
        # Record current deflection and time
        self.lastDeflection = self.getDeflection(time)
        self.lastTime = time

        # Apply limiters to new target
        if self.maxDeflection != None:
            newTarget = min(newTarget, self.maxDeflection)
        if self.minDeflection != None:
            newTarget = max(newTarget, self.minDeflection)

        # Set new target
        self.targetDeflection = newTarget

    def getDeflection(self, time):
        return self.getPosition(time, self.lastTime, self.lastDeflection, self.targetDeflection)

    def getDeflectionDerivative(self, _, _2, _3):
        ''' Not required for first-order actuator - deflections can be integrated analytically '''
        pass
