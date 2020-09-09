import abc

import MAPLEAF.Rocket.AeroFunctions as AeroFunctions
from MAPLEAF.GNC.Actuators import (FirstOrderActuator,
                               TableInterpolatingActuatorController)


class ActuatedSystem(abc.ABC):
    '''
        Mixin-type class
            Inherit from this class to add its functionality to a class
                __init__ method initializes actuators defined in the component's 'Actuator' subdictionary
                These actuators are then stored in self.actuatorList
            
            Child classes must call ActuatedSystem.__init__() to actually instantiate the actuators
                Expects that child classes also inherit from SubDictReader, and have already initialized the SubDictReader functionality
                Expects that child classes have a self.rocket attribute
    '''
    def __init__(self, nActuators):
        '''  This is a conrete, implemented method, intended to be called by child classes '''

        # Initialize Actuator Models
        actuatorList = []
        for i in range(nActuators):
            actuatorResponseModel = self.componentDictReader.getString("Actuators.responseModel")
            if actuatorResponseModel == "FirstOrder":
                responseTime = self.componentDictReader.getFloat("Actuators.responseTime")
                maxDeflection = self.componentDictReader.tryGetFloat("Actuators.maxDeflection", defaultValue=None)
                minDeflection = self.componentDictReader.tryGetFloat("Actuators.minDeflection", defaultValue=None)
                actuator = FirstOrderActuator(responseTime, maxDeflection=maxDeflection, minDeflection=minDeflection)
                actuatorList.append(actuator)
            else:
                raise ValueError("Actuator response model {} not implemented. Try 'FirstOrder'".format(actuatorResponseModel))

        self.actuatorList = actuatorList

        # Initialize Actuator controller
        controllerType = self.componentDictReader.getString("Actuators.controller")
        if controllerType == "TableInterpolating":
            deflectionTablePath = self.componentDictReader.getString("Actuators.deflectionTablePath")

            # Get list of 'key' columns
            deflectionKeyColumns = self.componentDictReader.getString("Actuators.deflectionKeyColumns").split()
            noDesiredKeys = []
            for key in deflectionKeyColumns:
                if "Desired" not in key:
                    noDesiredKeys.append(key)
                else:
                    break

            # Check that no non-'desired' key columns after desired ones
            for i in range(len(noDesiredKeys), len(deflectionKeyColumns)):
                if "Desired" not in deflectionKeyColumns[i]:
                    raise ValueError("'Desired' columns such as DesiredMx must come last in deflectionKeyColumns")

            deflectionKeyFunctionVector = [ AeroFunctions.stringToAeroFunctionMap[key] for key in noDesiredKeys ]

            self.actuatorController = TableInterpolatingActuatorController(deflectionTablePath, len(deflectionKeyColumns), deflectionKeyFunctionVector, actuatorList)
        else:
            raise ValueError("Actuator controller: {} not implemented. Try TableInterpolating.")

    @abc.abstractmethod
    def initializeActuators(self, controlSystem):
        ''' 
            Function should call ActuatedSystem.__init__(self, nActuators) - function needs to determine and pass in nActuators
        '''        
        pass
