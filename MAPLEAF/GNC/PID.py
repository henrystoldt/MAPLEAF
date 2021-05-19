''' PID controllers control parts of the control system and adaptive simulation timestepping '''

import numpy as np
from scipy.interpolate import LinearNDInterpolator
from itertools import combinations_with_replacement as cwithr

__all__ = [ "PIDController", "ConstantGainPIDController", "ScheduledGainPIDController" ]

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
        self._getPIDCoeffs = LinearNDInterpolator(keys, pidData)

    def updateCoefficientsFromGainTable(self, keyList):
        P, I, D = self._getPIDCoeffs(keyList)
        self.updateCoefficients(P, I, D)

class EquationScheduledGainPIDController(PIDController):
    def __init__(self, coefficientList, scheduledParameterList, equationOrder, initialError=0, maxIntegral=None):
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

        #Check that there are enough coefficients in the coefficients list
        self.numScheduledParameters = len(scheduledParameterList)
        for i in range(self.numScheduledParameters):
            self.scheduledParametersPositionList(i) = (i)

        self.equationOrder = equationOrder
        self.coefficientList = coefficientList
        self.variableValues = []

        desiredNumberOfCoefficients = 0
        for i in range(self.equationOrder):
            possibleCombinations = cwithr(self.scheduledParametersPositionList,1)
            numberOfCombinations = len(possibleCombinations)
            desiredNumberOfCoefficients = desiredNumberOfCoefficients + numberOfCombinations
        
        if len(coefficientList) != desiredNumberOfCoefficients:
            raise ValueError("Number of given coefficients: {} not suitable for equation of order {} with {} scheduled parameters".format(len(self.coefficientList),\
            self.equationOrder,self.numScheduledParameters))

        self.variableValues = np.zeros(len(coefficientList))

    def _updateVariablesFromParameters(self,parameterValues):
        
        variableList = []
        for order in range(self.equationOrder):
            variableCombinations = cwithr(self.scheduledParametersPositionList,order)
            variableList.append(variableCombinations)

        for variable in range(len(self.coefficientList)):
            total = 1
            for parameter in variableList(variable):
                total = total*parameterValues(parameter)
            self.variableValue(variable) = total

    def _getPIDCoeffs(parameterValues):

        for 




        #Create interpolation function for PID coefficients
        self._getPIDCoeffs = LinearNDInterpolator(keys, pidData)

    def updateCoefficientsFromGainEquation(self, parameterValues):
        P, I, D = self._getPIDCoeffs(keyList)
        self.updateCoefficients(P, I, D)

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