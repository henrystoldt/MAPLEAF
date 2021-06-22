''' PID controllers control parts of the control system and adaptive simulation timestepping '''

import numpy as np
from MAPLEAF.Motion import NoNaNLinearNDInterpolator

__all__ = [ "PIDController", "ConstantGainPIDController", "TableScheduledGainPIDController", "EquationScheduledGainPIDController"]

class PIDController():

    def __init__(self, P, I, D, initialError=0, maxIntegral=None):
        ''' To make this PID controller work for vectors of values, pass in np arrays 
                - they are added elementwise by default
        '''
        self.P = P
        self.I = I
        self.D = D
        self.lastError = initialError
        self.errorIntegral = initialError * 0 # Initialized like this in case value passed in is an array
        self.updateMaxIntegral(maxIntegral)

    def updateMaxIntegral(self, maxIntegral):
        if maxIntegral is not None:
            self.maxIntegralMagnitude = abs(maxIntegral)
        else:
            self.maxIntegralMagnitude = maxIntegral

    def getNewSetPoint(self, currentError, dt):
        # Calculate derivative
        derivative = (currentError - self.lastError) / dt
        
        # Calculate integral term
        self.errorIntegral = self.errorIntegral + (currentError + self.lastError)*dt / 2

        # Cap the magnitude of the integral term if necessary
        if self.maxIntegralMagnitude is not None:
            try:
                # Check if vector valued
                testIter = iter(self.errorIntegral)
                for i in range(len(self.errorIntegral)):
                    if abs(self.errorIntegral[i]) > self.maxIntegralMagnitude[i]:
                        self.errorIntegral[i] = self.errorIntegral[i] * self.maxIntegralMagnitude[i] / abs(self.errorIntegral[i])

            except TypeError:
                # Scalar valued
                if abs(self.errorIntegral) > self.maxIntegralMagnitude:
                    self.errorIntegral = self.errorIntegral * self.maxIntegralMagnitude / abs(self.errorIntegral)

        # Store error for next calculation
        self.lastError = currentError

        return self.P*currentError + self.I*self.errorIntegral + self.D*derivative

    def updateCoefficients(self, P, I, D, maxIntegral=None):
        self.P = P
        self.I = I
        self.D = D
        self.updateMaxIntegral(maxIntegral)

    def resetIntegral(self):
        self.errorIntegral = self.lastError * 0 # Done to handle arbitrary size np arrays

class ConstantGainPIDController(PIDController):

    def __init__(self, P=0, I=0, D=0, initialError=0, maxIntegral=None):
        '''
            Inputs:
                P:                  (int) Proportional Gain
                I:                  (int) Integral Gain
                D:                  (int) Derivative Gain
                DCol:               (int) zero-indexed column number of D Coefficient

                Note:
                    It is assumed that PCol, ICol, and DCol exist one after another in the table
                
                Inputs passed through to parent class (PICController):
                    initialError, maxIntegral
        '''
        PIDController.__init__(self, P,I,D, initialError=initialError, maxIntegral=maxIntegral)

class TableScheduledGainPIDController(PIDController):
    def __init__(self, gainTableFilePath, nKeyColumns=2, PCol=3, DCol=5, initialError=0, maxIntegral=None):
        '''
            Inputs:
                gainTableFilePath:  (string) Path to gain table text file ex: './MAPLEAF/Examples/TabulatedData/constPIDCoeffs.txt'
                nKeyColumns:        (int) Number of 'key' columns (independent variables). Key columns are assumed to be the nKeyColumns leftmost ones
                PCol:               (int) zero-indexed column number of P Coefficient
                DCol:               (int) zero-indexed column number of D Coefficient

                Note:
                    It is assumed that PCol, ICol, and DCol exist one after another in the table
                
                Inputs passed through to parent class (PICController):
                    initialError, maxIntegral
        '''
        PIDController.__init__(self, 0,0,0, initialError=initialError, maxIntegral=maxIntegral)

        # Columns are: Mach, Altitude(ASL), P1, I1, D1
        pidData = np.loadtxt(gainTableFilePath, skiprows=1)
        keys = pidData[:,0:nKeyColumns]
        pidData = pidData[:,PCol:DCol+1]

        #Create interpolation function for PID coefficients
        self._getPIDCoeffs = NoNaNLinearNDInterpolator(keys, pidData)

    def updateCoefficientsFromGainTable(self, keyList):
        P, I, D = self._getPIDCoeffs(keyList)
        self.updateCoefficients(P, I, D)

class EquationScheduledGainPIDController(PIDController):
    def __init__(self, coefficientMatrix, parameterList, equationOrder, initialError=0, maxIntegral=None):
        '''
            Inputs:
                coefficientList     (int) List of coefficients to be used in the gain scheduling equation
                parameterList:      (string) List of names of the parameters used in the gain scheduling, must be in the standardParameters dictionary
                equationOrder:       (int) Max order of the gain schedule equation
                
                Inputs passed through to parent class (PIDController):
                    initialError, maxIntegral
        '''
        PIDController.__init__(self, 0,0,0, initialError=initialError, maxIntegral=maxIntegral)

        #Move inputs into internal variables
        self.equationOrder = equationOrder
        self.PcoefficientList = coefficientMatrix[0]
        self.IcoefficientList = coefficientMatrix[1]
        self.DcoefficientList = coefficientMatrix[2]
        self.numberedVariableList = []
        numVariables = 0

        #Create a list that represents the parameters as numbers
        self.numberedParameterList = []
        for i in range(len(parameterList)):
          self.numberedParameterList.append(i)

        #variablesList is a list containing every variable combination using the equation order and the parameter list
        #numVariables is used to check that correct number of coefficients was provided
        for i in range(self.equationOrder+1):
          parameterCombinationsList = list(cwithr(self.numberedParameterList, i))
          numVariables = numVariables + len(parameterCombinationsList)
          self.numberedVariableList.append(parameterCombinationsList)

        #Store the number of coefficients
        self.numPCoefficients = len(self.PcoefficientList)
        self.numICoefficients = len(self.IcoefficientList)
        self.numDCoefficients = len(self.DcoefficientList)
        
        if self.numPCoefficients != numVariables:
            raise ValueError("Number of given P coefficients: {}, not suitable for equation of order {} with {} scheduled parameters".format(len(self.PcoefficientList),\
            self.equationOrder,len(parameterList)))

        if self.numICoefficients != numVariables:
            raise ValueError("Number of given I coefficients: {}, not suitable for equation of order {} with {} scheduled parameters".format(len(self.IcoefficientList),\
            self.equationOrder,len(parameterList)))

        if self.numDCoefficients != numVariables:
            raise ValueError("Number of given D coefficients: {}, not suitable for equation of order {} with {} scheduled parameters".format(len(self.DcoefficientList),\
            self.equationOrder,len(parameterList)))

        self.variableValuesList = np.zeros(numVariables)

    def _updateVariableValuesFromParameters(self,parameterValueList):
        
        variableIndex = 0
        for i in range(len(self.numberedVariableList)):
          for j in range(len(self.numberedVariableList[i])):
            variableValue = 1
            if i != 0: #Skipping the empty "constant"entry for now"
                for k in range(len(self.numberedVariableList[i][j])):
                  temp = parameterValueList[self.numberedVariableList[i][j][k]]
                  variableValue = variableValue*temp
            self.variableValuesList[variableIndex] = variableValue
            variableIndex = variableIndex + 1
        
    def updateCoefficientsFromEquation(self,parameterValueList):

      self._updateVariableValuesFromParameters(parameterValueList)
      P = 0
      I = 0
      D = 0
      for i in range(len(self.variableValuesList)):
        P = P + self.variableValuesList[i]*self.PcoefficientList[i]
        I = I + self.variableValuesList[i]*self.IcoefficientList[i]
        D = D + self.variableValuesList[i]*self.DcoefficientList[i]

      self.updateCoefficients(P,I,D)