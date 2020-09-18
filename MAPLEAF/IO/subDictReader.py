'''
    Wrapper class to read from a specific sub-dictionary in a SimDefinition.
'''

from distutils.util import strtobool
from typing import List, Union

from MAPLEAF.Motion import Vector

__all__ = [ "SubDictReader" ]

class SubDictReader():

    def __init__(self, stringPathToThisItemsSubDictionary, simDefinition):
        '''
            Example stringPathToThisItemsSubDictionary = 'Rocket.Stage1.Nosecone' if we're initializing a nosecone object
        '''
        self.simDefDictPathToReadFrom = stringPathToThisItemsSubDictionary
        self.simDefinition = simDefinition

    def getString(self, key):
        ''' 
            Pass in either relative key or absolute key:
                Ex 1 (Relative): If object subdictionary (self.simDefDictPathToReadFrom) is 'Rocket.Stage1.Nosecone', relative keys could be 'position' or 'aspectRatio'
                    These would retrieve Rocket.Stage1.Nosecone.position or Rocket.Stage1.Nosecone.aspectRatio from the sim definition
                Ex 2 (Absolute): Can also pass in full absolute key, like 'Rocket.Stage1.Nosecone.position', and it will retrieve that value, as long as there isn't a 'path collision' with a relative path
        '''
        try:
            return self.simDefinition.getValue(self.simDefDictPathToReadFrom + "." + key)
        except KeyError:
            try:
                return self.simDefinition.getValue(key)
            except KeyError:
                attemptedKey1 = self.simDefDictPathToReadFrom + "." + key
                attemptedKey2 = key
                raise KeyError("{} and {} not found in {} or in default value dictionary".format(attemptedKey1, attemptedKey2, self.simDefinition.fileName))

    #### Get parsed values ####
    def getInt(self, key: str) -> int:
        return int(self.getString(key))

    def getFloat(self, key: str) -> float:
        return float(self.getString(key))

    def getVector(self, key: str) -> Vector:
        return Vector(self.getString(key))

    def getBool(self, key: str) -> bool:
        return strtobool(self.getString(key))

    #### Try get values (return specified default value if not found) ####
    def tryGetString(self, key: str, defaultValue: Union[None, str]=None):
        try:
            return self.getString(key)
        except KeyError:
            return defaultValue

    def tryGetInt(self, key: str, defaultValue: Union[None, int]=None):
        try:
            return int(self.getString(key))
        except KeyError:
            return defaultValue

    def tryGetFloat(self, key: str, defaultValue: Union[None, float]=None):
        try:
            return float(self.getString(key))
        except KeyError:
            return defaultValue

    def tryGetVector(self, key: str, defaultValue: Union[None, Vector]=None):
        try:
            return Vector(self.getString(key))
        except KeyError:
            return defaultValue

    def tryGetBool(self, key: str, defaultValue: Union[None, bool]=None):
        try:
            return strtobool(self.getString(key))
        except KeyError:
            return defaultValue

    #### Introspection ####
    def getImmediateSubDicts(self, key=None) -> List[str]:
        if key == None:
            key = self.simDefDictPathToReadFrom
        return self.simDefinition.getImmediateSubDicts(key)

    def getSubKeys(self, key=None) -> List[str]:
        if key == None:
            key = self.simDefDictPathToReadFrom
        return self.simDefinition.getSubKeys(key)

    def getImmediateSubKeys(self, key=None) -> List[str]:
        if key == None:
            key = self.simDefDictPathToReadFrom
        return self.simDefinition.getImmediateSubKeys(key)

    def getDictName(self) -> str:
        lastDotIndex = self.simDefDictPathToReadFrom.rfind('.')
        return self.simDefDictPathToReadFrom[lastDotIndex+1:]
