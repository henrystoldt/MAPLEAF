'''
Interpolation rigid body states and scalar values.
State interpolation used in flight animations.
Scalar interpolation used for interpolation of transonic aerodynamic forces.
'''
from bisect import bisect
from functools import lru_cache

import numpy as np
import scipy.linalg as linalg
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator

__all__ = [ "linInterp", "linInterpWeights", "calculateCubicInterpCoefficients", "cubicInterp", "NoNaNLinearNDInterpolator" ]

def linInterp(X, Y, desiredX):
    '''
        Arguments:
            X: Sorted list or numpy array of numeric x-values
            Y: Sorted list or numpy array of numeric y-values
            desiredX: Numeric x-value, indicating point to interpolate to

        Returns:
            desiredY: The linearly-interpolated y value at x=desiredX

        Notes:
            Uses binary search (bisect) to locate interpolation interval
            Faster than built-in methods for our application (see test/LinInterpSpeed.py)
    '''
    interpPt = bisect(X, desiredX)
    
    if interpPt >= len(X):
        return Y[len(X)-1]
    elif interpPt < 1:
        return Y[0]
    else:
        lgX = X[interpPt]
        smX = X[interpPt-1]
        lgY = Y[interpPt]
        smY = Y[interpPt-1]

    return (lgY - smY)*(desiredX - smX)/(lgX - smX) + smY

def linInterpWeights(X, desiredX):
    ''' 
        Expects the list X is sorted
        Returns smallYIndex, smallYWeight, largeYIndex, largeYWeight:
            Ex: X = [ 0, 1, 2, 3 ], desiredX = 0.75
                smallYIndex = 0
                smallYWeight = 0.25
                largeYIndex = 1
                largeYWeight = 0.75

            Then, to calculate the interpolate value:
                interpVal = Y[smallYIndex]*smallYWeight + Y[largeYIndex]*largeYWeight
    '''
    interpPt = bisect(X, desiredX)

    #Edge cases
    if interpPt >= len(X):
        return 0, 0, -1, 1
    elif interpPt < 1:
        return 0, 1, 0, 0

    # Normal cases
    smallYIndex = interpPt -1
    largeYIndex = interpPt
    largeYWeight = (desiredX - X[smallYIndex]) / (X[largeYIndex] - X[smallYIndex])
    smallYWeight = 1 - largeYWeight

    return smallYIndex, smallYWeight, largeYIndex, largeYWeight

@lru_cache(20)
def getInvAMatrix(X1, X2):
    '''
        Computes the inverse of the A matrix used in `calculateCubicInterpCoefficients` below.
        During a simulation, these will only change from component to component, not from time step to time step, hence the cache
    '''
    AMatrix = \
        np.array([  [ 1,    X1, X1**2,  X1**3 ],
                    [ 1,    X2, X2**2,  X2**3 ],
                    [ 0,    1,  2*X1, 3*X1**2 ],
                    [ 0,    1,  2*X2, 3*X2**2 ] ])

    return linalg.inv(AMatrix)

def calculateCubicInterpCoefficients(X1, X2, Y1, Y2, dydx1, dydx2):
    ''' Returns coefficients for a cubic polynomial that matches values and derivatives at x1 and x2 '''
    # AMatrix and B, together define the following equations which constrain the cubic interpolation
    # f(x=x1)       == Y1
    # f(x=x2)       == Y2
    # df/dx (x=x1)  == dydx1
    # df/dx (x=x2)  == dydx2
    Ainv = getInvAMatrix(X1, X2)
    
    B = np.array([  [Y1], 
                    [Y2], 
                    [dydx1], 
                    [dydx2]])

    return Ainv.dot(B)

def cubicInterp(X, X1, X2, Y1, Y2, Y1_plusDx, Y2_plusDx, dx):
    dy_dx_x1 = (Y1_plusDx - Y1) / dx
    dy_dx_x2 = (Y2_plusDx - Y2) / dx

    interpCoeffs = calculateCubicInterpCoefficients(X1, X2, Y1, Y2, dy_dx_x1, dy_dx_x2)
    return float(interpCoeffs[0] + interpCoeffs[1]*X + interpCoeffs[2]*X**2 + interpCoeffs[3]*X**3)

class NoNaNLinearNDInterpolator():
    def __init__(self, keys, values, tablePath=None) -> None:
        self.linearInterpolator = LinearNDInterpolator(keys, values)
        self.nearestInterpolator = NearestNDInterpolator(keys, values)
        self.tablePath = tablePath

    def __call__(self, *keyVector):
        linearResult = self.linearInterpolator(*keyVector)
        
        if np.isnan(linearResult).any():
            # Occurs if the requested values are outside of the bounds of the table being interpolated over
                # In that case just return the nearest result
            print("WARNING: Interpolation requested outside of bounds in table: {}. Current key vector = {}. Extrapolation not supported, returning nearest result instead".format(self.tablePath, keyVector))
            return self.nearestInterpolator(*keyVector)
        
        else:
            return linearResult  