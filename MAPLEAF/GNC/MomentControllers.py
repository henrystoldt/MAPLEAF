'''
Moment controllers determine what overall moments should be applied to the rocket to remain on the course provided by the `MAPLEAF.GNC.ControlSystems.ControlSystem`
'''

import abc

import numpy as np
import pandas as pd

from itertools import combinations_with_replacement as cwithr

from MAPLEAF.Motion import AeroParameters, AngularVelocity, Vector
from MAPLEAF.GNC import *

__all__ = ["ConstantGainPIDRocketMomentController", "TableScheduledGainPIDRocketMomentController", "EquationScheduledGainPIDRocketMomentController", "MomentController", "IdealMomentController" ]

class MomentController(abc.ABC):
    @abc.abstractmethod
    def getDesiredMoments(self, rocketState, environment, targetOrientation, time, dt):
        ''' Should return a list [ desired x-axis, y-axis, and z-axis ] moments '''

class TableScheduledGainPIDRocketMomentController(MomentController, TableScheduledGainPIDController):
    def __init__(self, gainTableFilePath, keyColumnNames):
        '''
            Assumes the longitudinal (Pitch/Yaw) PID coefficients are in columns nKeyColumns:nKeyColumns+2
            Assumes the roll PID coefficients are in columns nKeyColumns+3:nKeyColumns+5
            Sample gain file: MAPLEAF/Examples/TabulatedData/testPIDCoeffs.txt
        '''
        self.keyFunctionList = [ AeroParameters.stringToAeroFunctionMap[x] for x in keyColumnNames ]
        nKeyColumns = len(keyColumnNames)
        TableScheduledGainPIDController.__init__(self, gainTableFilePath, nKeyColumns, PCol=nKeyColumns, DCol=nKeyColumns+5)
        
    def updateCoefficientsFromGainTable(self, keyList):
        ''' Overriding parent class method to enable separate longitudinal and roll coefficients in a single controller '''
        Pxy, Ixy, Dxy, Pz, Iz, Dz = self._getPIDCoeffs(*keyList)

        # Coefficient for each of P, I, and D are: Longitudinal, Longitudinal, Roll
        # Operates the internal PID controller in vector-mode with numpy arrays (using their elementwise operations)
        P = np.array([Pxy, Pxy, Pz])
        I = np.array([Ixy, Ixy, Iz])
        D = np.array([Dxy, Dxy, Dz])
        self.updateCoefficients(P, I, D)

    def _getOrientationError(self, rocketState, targetOrientation):
        return np.array((targetOrientation / rocketState.orientation).toRotationVector())

    def getDesiredMoments(self, rocketState, environment, targetOrientation, time, dt):
        '''
            Inputs: 
                gainKeyList:        iterable of length nKeyColumns, containing the data required to interpolate the PIDxy and PIDz coefficients in the gain table
                rocketState:        must have .orientation attribute (Quaternion)
                targetOrientation:  (Quaternion)
                _:                  currently unused (time argument)
                dt:                 (numeric) time since last execution of the control system
        '''
        orientationError = self._getOrientationError(rocketState, targetOrientation)
        gainKeyList = AeroParameters.getAeroPropertiesList(self.keyFunctionList, rocketState, environment)
        self.updateCoefficientsFromGainTable(gainKeyList)
        return self.getNewSetPoint(orientationError, dt)

class EquationScheduledGainPIDRocketMomentController(MomentController):

    def __init__(self, lateralCoefficientsPath, longitudinalCoefficientsPath, parameterList, equationOrder, controlSystemDictReader):

      def _getEquationCoefficientsFromTextFile(textFilePath):
        equationCoefficients = pd.read_csv(textFilePath)
        fileHeader = equationCoefficients.columns.to_list()

        if fileHeader != ['P','I','D']:
          raise ValueError("The data in text file {} is not in the proper format".format(textFilePath))

        PCoefficients = equationCoefficients["P"].to_list()
        ICoefficients = equationCoefficients["I"].to_list()
        DCoefficients = equationCoefficients["D"].to_list()

        allCoefficients = []
        allCoefficients.append(PCoefficients)
        allCoefficients.append(ICoefficients)
        allCoefficients.append(DCoefficients)

        return allCoefficients
      #parameterList must contains strings that match those in Motion/AeroParameters
      self.parameterFetchFunctionList = [ AeroParameters.stringToAeroFunctionMap[x] for x in parameterList ]

      pitchCoefficientList = _getEquationCoefficientsFromTextFile(lateralCoefficientsPath)
      yawCoefficientList = pitchCoefficientList
      rollCoefficientList = _getEquationCoefficientsFromTextFile(longitudinalCoefficientsPath)

      for i in range(len(pitchCoefficientList)):
        controlSystemDictReader.simDefinition.setValue('PitchC' + str(i),pitchCoefficientList[i])
        controlSystemDictReader.simDefinition.setValue('YawC' + str(i),yawCoefficientList[i])
        controlSystemDictReader.simDefinition.setValue('RollC' + str(i),rollCoefficientList[i])


      self.pitchController = EquationScheduledGainPIDController(pitchCoefficientList, parameterList, equationOrder)
      self.yawController = EquationScheduledGainPIDController(yawCoefficientList, parameterList, equationOrder)
      self.rollController = EquationScheduledGainPIDController(rollCoefficientList, parameterList, equationOrder)
    
    def _getOrientationError(self, rocketState, targetOrientation):
        return np.array((targetOrientation / rocketState.orientation).toRotationVector())

    def getDesiredMoments(self, rocketState, environment, targetOrientation, time, dt):

      orientationError = self._getOrientationError(rocketState, targetOrientation)
      variableFunctionList= AeroParameters.getAeroPropertiesList(self.parameterFetchFunctionList, rocketState, environment)
      self._updateCoefficientsFromEquation(variableFunctionList)

      return self._getNewSetPoint(orientationError, dt)

    def _updateCoefficientsFromEquation(self, variableFunctionList):

      self.yawController.updateCoefficientsFromEquation(variableFunctionList)
      self.pitchController.updateCoefficientsFromEquation(variableFunctionList)
      self.rollController.updateCoefficientsFromEquation(variableFunctionList)

    def _getNewSetPoint(self,currentError,dt):


      output = [0,0,0]
      output[0] = self.pitchController.getNewSetPoint(currentError[0],dt)
      output[1] = self.yawController.getNewSetPoint(currentError[1],dt)
      output[2] = self.rollController.getNewSetPoint(currentError[2],dt)

      return output


class ConstantGainPIDRocketMomentController(MomentController, ConstantGainPIDController):
    def __init__(self, Pxy, Ixy, Dxy, Pz, Iz, Dz):
        '''
            Constant PID coefficient controller
        '''
        ConstantGainPIDController.__init__(self)
        
        P = np.array([Pxy, Pxy, Pz])
        I = np.array([Ixy, Ixy, Iz])
        D = np.array([Dxy, Dxy, Dz])
        self.updateCoefficients(P,I,D)

    def _getOrientationError(self, rocketState, targetOrientation):
        return np.array((targetOrientation / rocketState.orientation).toRotationVector())

    def getDesiredMoments(self, rocketState, environment, targetOrientation, time, dt):
        '''
            Inputs: 
                gainKeyList:        iterable of length nKeyColumns, containing the data required to interpolate the PIDxy and PIDz coefficients in the gain table
                rocketState:        must have .orientation attribute (Quaternion)
                targetOrientation:  (Quaternion)
                _:                  currently unused (time argument)
                dt:                 (numeric) time since last execution of the control system
        '''
        orientationError = self._getOrientationError(rocketState, targetOrientation)
        return self.getNewSetPoint(orientationError, dt)

class IdealMomentController():
    def __init__(self, rocket):
        self.rocket = rocket

    def getDesiredMoments(self, rocketState, environment, targetOrientation, time, dt):
        """ Exactly match the desired target orientation and set the rockets angular velocity to zero """
        self.rocket.rigidBody.state.orientation = targetOrientation
        self.rocket.rigidBody.state.angularVelocity = AngularVelocity(rotationVector=Vector(0,0,0))