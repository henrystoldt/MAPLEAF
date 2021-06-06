''' 
    Defines ODE integrators for constant and adaptive time stepping. Used by the `MAPLEAF.Motion.RigidBody` classes to integrate rocket motion 

    Both the integrator and the adaptive integrator classes are callable, meaning they can be called like a function once instantiated
        This is facilitated by their __call__ methods

    See `MAPLEAF.Motion.RigidBodies.RigidBody_3DoF.timeStep` for an example use of these Integrators 
        (see `MAPLEAF.Motion.RigidBodies.RigidBody_3DoF.__init__` for creation of the self.integrate function using the integratorFactory below)

    Many of the integration methods defined below rely on Butcher tableaus, in MAPLEAF they are represented by lists of lists as follows:
        Expected Format (Example is RK4 - 3/8 method, see conventional Butcher tableau here: https://en.wikipedia.org/wiki/List_of_Runge%E2%80%93Kutta_methods#3/8-rule_fourth-order_method):
            [                                   # Top 0 row ommitted
                [ 1/3, 1/3 ],                   # Row 1: all coefficients sequentially, first column is current time (c_i), remainder are a_{i1} to a_{in} (derivative coefficients)
                [ 2/3, -1/3, 1.0 ],             # Row 2: ''
                [ 1.0, 1.0, -1.0, 1.0 ],        # Row 3: ''
                [ 1/8, 3/8, 3/8, 1/8 ]          # Row 4: (result calculation row) - empty space on left ignored, just enter all coefficients
            ]

        For adaptive methods, the bottom two rows are 'result calculation rows'
            The second last row is expected to represent the higher-accuracy method
            The last row is expected to represent the lower-accuracy method 
            The difference between the results obtained from the two methods becomes the error estimate

        Learn about Butcher Tableaus here: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
            RK4 - 3/8 Butcher Tableau is present there in the Examples section
'''
import math
from abc import ABC, abstractmethod
from typing import Callable

__all__ = [ "integratorFactory" ]

def checkButcherTableau(tableau):
    ''' Checks that the Butcher tableau passed in represents a consistent R-K method '''
    lastRowLength = 0
    for i in range(len(tableau)):
        if len(tableau[i]) == lastRowLength:
            # We're checking a row of "b" coefficients (constructing a final answer)
            # Here the sum of all numbers should be 1
            sumB = sum(tableau[i])
            assert math.isclose(sumB, 1.0), "Sum of 'b' coefficients ({}) of butcher tableau row {} don't = 1".format(sumB, i)
        else:
            # We're checking a row of "c" and "a" coefficients (constructing one of the k values)
            # Here the first number, the c coefficient, should be equal to the sum of all the other numbers (the a coefficients)
            c = tableau[i][0]
            sumA = sum(tableau[i][1:])
            assert math.isclose(c, sumA), "Sum of 'a' coefficients ({}) of butcher tableau row {} don't = 'c' coefficient from the same row {}".format(sumA, i, c)

        lastRowLength = len(tableau[i])

    for i in range(len(tableau)):
        for j in range(len(tableau[i])):
            if tableau[i][j] == 0:
                tableau[i][j] = 1e-16

def integratorFactory(integrationMethod="Euler", simDefinition=None, discardedTimeStepCallback=None):
    ''' 
        Returns a callable integrator object

        Inputs:
            * integrationMethod: (str) Name of integration method: Examples = "Euler", "RK4", "RK23Adaptive", and "RK45Adaptive"
            * simDefinition: (`MAPLEAF.IO.SimDefinition`) for adaptive integration, provide a simdefinition file with time step adaptation parameters
            * discardedTimeStepCallback: (1-argument function reference) for adaptive integration, this function (if provided) is called when a time step is computed,
                but then discarded and re-computed with a smaller timestep to remain below the max estimated error threshold. Used by MAPLEAF to remove
                force calculation logs from those discarded time steps
    '''
    if "Adapt" in integrationMethod:
        if simDefinition == None:
            raise ValueError("SimDefinition object required to initialize adaptive integrator")

        from MAPLEAF.IO import SubDictReader
        adaptDictReader = SubDictReader("SimControl.TimeStepAdaptation", simDefinition)

        # Adaptive Integration
        controller = adaptDictReader.getString("controller")
        if controller == "PID":
            PIDCoeffs = [ float(x) for x in adaptDictReader.getString("PID.coefficients").split() ]
            safetyFactor = None
        elif controller == "elementary":
            safetyFactor = adaptDictReader.getFloat("Elementary.safetyFactor")
            PIDCoeffs = None

        targetError = adaptDictReader.getFloat("targetError")
        minFactor = adaptDictReader.getFloat("minFactor")
        maxFactor = adaptDictReader.getFloat("maxFactor")
        maxTimeStep = adaptDictReader.getFloat("maxTimeStep")
        minTimeStep = adaptDictReader.getFloat("minTimeStep")
        
        return AdaptiveIntegrator(
            method=integrationMethod, 
            controller=controller, 
            targetError=targetError, 
            maxMinSafetyFactors=[maxFactor, minFactor, safetyFactor], 
            PIDCoeffs=PIDCoeffs, 
            maxTimeStep=maxTimeStep, 
            minTimeStep=minTimeStep, 
            discardedTimeStepCallback=discardedTimeStepCallback
        )
    
    else:
        # Constant time step integrator
        return ClassicalIntegrator(method=integrationMethod)


class Integrator(ABC):
    @abstractmethod
    def __call__(self, initVal, initTime:float, derivativeFunc:Callable, dt:float):
        '''
            See `ClassicalIntegrator._integrateEuler` for simplest possible implementation
            
            Inputs:
                initVal: must be addable with other objects of its type
                derivativeFunc: is expected to be a function which accepts the following arguments: initTime, initVal
                    Should return an object representing the derivative of objects of type(initVal)
                        - must be multipliable by objects of type(dt) -> should return an object of type(initVal) from the multiplication operation

                    See `MAPLEAF.Motion.RigidBodies.RigidBody_3DoF.rigidBodyStateDerivative` for an example of derivativeFunc

            Returns (in order):
                An object of type IntegrationResult
                
        '''
        pass

class IntegrationResult():
    __slots__ = [ 'newValue', 'timeStepAdaptationFactor', 'errorMagEstimate', 'dt', 'derivativeEstimate' ]

    def __init__(self, newValue, dt, derivativeEstimate, timeStepAdaptationFactor=1.0, errorMagEstimate=0.0):
        '''
            newValue:                   Value of quantity represented by initVal at time initTime+dt
            dt:                         The size of the time step actually taken (error-limited adaptive integrators can override to shrink the time step)
            derivativeEstimate:         Estimate of the value of the function derivative obtained by the integrator over the last time step
            timeStepAdaptationFactor:   For adaptive methods, suggest time step adaption (otherwise 1)
            errorMagEstimate:           For adaptive methods, provide error estimate (otherwise 0)            
                errorMagEstimate:           For adaptive methods, provide error estimate (otherwise 0)
            errorMagEstimate:           For adaptive methods, provide error estimate (otherwise 0)            
        '''
        self.newValue = newValue
        self.dt = dt
        self.derivativeEstimate = derivativeEstimate
        self.timeStepAdaptationFactor = timeStepAdaptationFactor
        self.errorMagEstimate = errorMagEstimate

class ClassicalIntegrator(Integrator):
    ''' Callable class for constant-dt ODE integration '''

    def __init__(self, method="Euler"):
        # Save integration method and associated Butcher tableau
        self.method = method
        self.tableau = None

        if method == "Euler":
            self.integrate = self._integrateEuler
        elif method == "RK2Midpoint":
            self.integrate = self._integrateByButcherTableau
            self.tableau = [
                [0.5, 0.5],
                [0,   1  ]
            ]
        elif method == "RK2Heun":
            self.integrate = self._integrateByButcherTableau
            self.tableau = [
                [1, 1],
                [0.5, 0.5]
            ]
        elif method == "RK4":
            self.integrate = self._integrateByButcherTableau
            self.tableau = [
                [ 0.5, 0.5 ],
                [ 0.5, 0, 0.5 ],
                [ 1, 0, 0, 1 ],
                [ 1/6, 1/3, 1/3, 1/6 ]
            ]
        elif method == "RK4_3/8":
            self.integrate = self._integrateByButcherTableau
            self.tableau = [
                [ 1/3, 1/3 ],
                [ 2/3, -1/3, 1.0 ],
                [ 1.0, 1.0, -1.0, 1.0 ],
                [ 1/8, 3/8, 3/8, 1/8 ]
            ]
        else:       
            raise ValueError("Integration method: {} not implemented. See SimDefinitionTemplate.txt for options.".format(method))

        # If a butcher tableau is used to define the R-K method, check that it is valid
        if self.tableau != None:
            checkButcherTableau(self.tableau)

    def __call__(self, initVal, initTime, derivativeFunc, dt):
        return self.integrate(initVal, initTime, derivativeFunc, dt)

    #### Constant time step methods ####
    def _integrateEuler(self, initVal, initTime, derivativeFunc, dt):
        yPrime = derivativeFunc(initTime, initVal)
        return IntegrationResult(initVal + yPrime*dt, dt, yPrime)

    def _integrateByButcherTableau(self, initVal, initTime, derivativeFunc, dt):
        '''
            Integrates a function based on a Butcher tableau defined in self.tableau
            Format expected is:
            [
                [ c2, a21 ],
                [ c3, a31, a32 ],
                ...
                [ b1, b2, ... ]

            Then for row i of the tableau, ki = derivativefunc(t + ci*dt, y + dt(ai1*k1 + ai2*k2 + ...))
            Then final result is y + dt*(b1*k1 + b2*k2 + ...)
            Further explanation: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#Use
        '''
        tab = self.tableau

        # Initialize array of derivatives (k's) with first k-value from beginning of interval
        k = [ derivativeFunc(initTime, initVal) ]
        # Calculate all other k's - one for each row of the tableau except the last one
        for i in range(len(tab) - 1):
            evalDt = dt*tab[i][0]
            evalTime = initTime + evalDt

            evalK = k[0] / (1/(tab[i][1]*dt)) # Do not start summation with 0, because could be a non-scalar type
            for a in range(2, len(tab[i])):
                evalK = evalK + k[a-1] / (1/(tab[i][a] * dt))
            evalY = initVal + evalK*1

            k.append(derivativeFunc(evalTime, evalY))

        # Calculate final result using last row of coefficients
        # In this summation and the one in the k-calculation loop above, terms are divided by the inverse of the terms they would normally be multiplied by
            # This does not affect the results for scalars
            # But it does prevent rigid body derivative objects from prematurely turning themselves into rigid body state objects (which occurs upon multiplication by a scalar)
        derivative = k[0] / (1/ tab[-1][0]) # Do not start summation with 0, because could be a non-scalar type
        for i in range(1, len(tab[-1])):
            derivative = derivative + k[i]/(1/tab[-1][i])


        return IntegrationResult((initVal + derivative*dt), dt, derivative)

class AdaptiveIntegrator(Integrator):
    ''' Callable class for error-limited adaptive-dt ODE integration '''

    def __init__(self, method="RK45Adaptive", controller="constant", targetError=0.001, maxMinSafetyFactors=[1.5, 0.3, 0.9], PIDCoeffs=[1.0, 0.1, 0.0], maxTimeStep=5, minTimeStep=0.0001, discardedTimeStepCallback=None):
        '''
            RepeatedTimeStepCallback: (function) will be called with (self) when a computed time step is discarded and recompouted (occurs if the estimated error is > targetError * 100)
                                            this callback is currently used to discard log entries associate with discarded time steps
        '''
        # Save integration method and associated
        self.method = method
        self.tableau = None
    
        self.derivativeCache = None # For some adaptive methods, the last derivative of the previous integration step is the same as the first of the following step. This field caches it
        if method == "RK12Adaptive": # Just a test method, already implemented non-butcher tableau version
            self.tableau = [
                [ 0.5, 0.5 ],
                [ 0.0, 1.0 ],
                [ 1.0, 0.0 ]
            ]
            self.firstSameAsLast = False
        elif method == "RK23Adaptive": # Bogacki-Shampine method
            self.tableau = [
                [ 0.5, 0.5 ],
                [ 3/4, 0.0, 3/4 ],
                [ 1.0, 2/9, 1/3, 4/9 ],
                [ 2/9, 1/3, 4/9, 0.0 ],
                [ 7/24, 1/4, 1/3, 1/8 ]
            ]
            self.firstSameAsLast = True
        elif method == "RK45Adaptive": # Dormand-Prince RK5(4)7FM method
            self.tableau = [
                [ 1/5, 1/5 ],
                [ 3/10, 3/40, 9/40 ],
                [ 4/5, 44/45, -56/15, 32/9 ],
                [ 8/9, 19372/6561, -25360/2187, 64448/6561, -212/729 ],
                [ 1.0, 9017/3168, -355/33, 46732/5247, 49/176, -5103/18656 ],
                [ 1.0, 35/384, 0.0, 500/1113, 125/192, -2187/6784, 11/84 ],
                [ 35/384, 0.0, 500/1113, 125/192, -2187/6784, 11/84, 0.0 ], # 5th order
                [ 5179/57600, 0.0, 7571/16695, 393/640, -92097/339200, 187/2100, 1/40 ] # 4th order
            ]
            self.firstSameAsLast = True
        elif method == "RK78Adaptive": # Dormand-Prince RK8(7)13M method (rational approximation)
            self.tableau = [
                [ 1/18, 1/18 ], 
                [ 1/12, 1/48, 1/16 ], 
                [ 1/8, 1/32, 0, 3/32 ], 
                [ 5/16, 5/16, 0, -75/64, 75/64 ], 
                [ 3/8, 3/80, 0, 0, 3/16, 3/20 ], 
                [ 59/400, 29443841/614563906, 0, 0, 77736538/692538347, -28693883/1125000000, 23124283/1800000000 ],
                [ 93/200, 16016141/946692911, 0, 0, 61564180/158732637, 22789713/633445777, 545815736/2771057229, -180193667/1043307555 ],
                [ 5490023248/9719169821, 39632708/573591083, 0, 0, -433636366/683701615, -421739975/2616292301, 100302831/723423059, 790204164/839813087, 800635310/3783071287 ],
                [ 13/20, 246121993/1340847787, 0, 0, -37695042795/15268766246, -309121744/1061227803, -12992083/490766935, 6005943493/2108947869, 393006217/1396673457, 123872331/1001029789 ],
                [ 1201146811/1299019798, -1028468189/846180014, 0, 0, 8478235783/508512852, 1311729495/1432422823, -10304129995/1701304382, -48777925059/3047939560, 15336726248/1032824649, -45442868181/3398467696, 3065993473/597172653 ],
                [ 1, 185892177/718116043, 0, 0, -3185094517/667107341, -477755414/1098053517, -703635378/230739211, 5731566787/1027545527, 5232866602/850066563, -4093664535/808688257, 3962137247/1805957418, 65686358/487910083 ],
                [ 1, 403863854/491063109, 0, 0, -5068492393/434740067, -411421997/543043805, 652783627/914296604, 11173962825/925320556, -13158990841/6184727034, 3936647629/1978049680, -160528059/685178525, 248638103/1413531060, 0 ],
                [ 14005451/335480064, 0, 0, 0, 0, -59238493/1068277825, 181606767/758867731,   561292985/797845732,   -1041891430/1371343529,  760417239/1151165299, 118820643/751138087, -528747749/2220607170,  1/4], # 8th order
                [ 13451932/455176623, 0, 0, 0, 0, -808719846/976000145, 1757004468/5645159321, 656045339/265891186,   -3867574721/1518517206,   465885868/322736535,  53011238/667516719,                  2/45,    0] # 7th order
            ]
            self.firstSameAsLast = False
        else:
            raise ValueError("Integration method: {} not implemented. See SimDefinitionTemplate.txt for options.".format(method))

        checkButcherTableau(self.tableau)

        # Save limiter info and target error, required for any adaptation scheme
        self.maxFactor, self.minFactor, self.safetyFactor = maxMinSafetyFactors
        self.targetError = targetError
        self.maxTimeStep = maxTimeStep
        self.minTimeStep = minTimeStep
        self.discardedTimeStepCallback = discardedTimeStepCallback

        ### Set up the chosen adaptation method ###
        if controller == "elementary":
            # Adjust time step size using an elementary controller
            self.getTimeStepAdjustmentFactor = self._getTimeStepAdjustmentFactor_Elementary

        elif controller == "PID":
            # Adjust time step size using a PID controller
            self.safetyFactor = 1
            
            # Delayed import of GNC.PID to avoid circular import problems
            from MAPLEAF.GNC import PIDController
            self.PIDController = PIDController(*PIDCoeffs, maxIntegral=1)

            self.getTimeStepAdjustmentFactor = self._getTimeStepAdjustmentFactor_PID

        elif controller == "Constant":
            # Do not adjust the time step
            self.getTimeStepAdjustmentFactor = self._getTimeStepAdjustmentFactor_Constant

        else:
            raise ValueError("Adaptive Integrator requires non-constant step size controller such as 'PID' or 'elementary'")

    def __call__(self, initVal, initTime, derivativeFunc, dt):
        # Set up incorrect values to force first iteration, like a do-while loop
        errorMagEstimate = 10000000
        maxErrorMultiple = 20 # Will recompute timestep if estimated error > maxErrorMultiple * targetError
        dt *= 3

        # Loop will lower the actual time step taken if error is more than 20*target
        while errorMagEstimate > maxErrorMultiple*self.targetError:
            if errorMagEstimate != 10000000:
                # This means this is not the first loop. Therefore a previously-computed time step has been discarded, and will be recomputed below with a smaller time step
                if self.discardedTimeStepCallback != None:
                    self.discardedTimeStepCallback(self)

            dt = max(self.minTimeStep, dt/3)
            result, derivative, errorMagEstimate, lastDerivativeEvaluation = self._integrate(initVal, initTime, derivativeFunc, dt, self.derivativeCache)
            if math.isclose(dt,self.minTimeStep):
                break
        
        if self.firstSameAsLast:
            self.derivativeCache = lastDerivativeEvaluation

        # Compute adaptation factor
        desiredAdaptFactor = self.getTimeStepAdjustmentFactor(errorMagEstimate, dt)
        limitedAdaptFactor = self._limitAdaptationFactor(desiredAdaptFactor, dt)

        return IntegrationResult(result, dt, derivative, limitedAdaptFactor, errorMagEstimate)
   
    #### Time Step adjustment ####
    def _limitAdaptationFactor(self, desiredAdaptFactor, dt):
        ''' 
            Apply adaptation limiters 
            Adaptation can be limited in two ways: by self.maxFactor/self.minFactor and by self.minTimeStep/self.maxTimeStep.
            The below checks which limitation is currently most restrictive and applies that one
        '''
        # Calculate min/max adaptation factors based on min/max time step size restrictions
        minFactor2 = self.minTimeStep / dt
        maxFactor2 = self.maxTimeStep / dt

        # Calculate resulting total min/max factors
        minFactor = max(minFactor2, self.minFactor)
        maxFactor = min(maxFactor2, self.maxFactor)

        # Apply limits
        return self.safetyFactor * min(max(desiredAdaptFactor, minFactor), maxFactor)

    def _getTimeStepAdjustmentFactor_Constant(self, errorMag, dt):
        ''' Don't adjust the time step (always 1.0) '''
        return 1

    def _getTimeStepAdjustmentFactor_Elementary(self, errorMag, dt):
        ''' Calculates the time step adjustment factor when using an elementary controller '''
        return (self.targetError / (2*errorMag))**0.5

    def _getTimeStepAdjustmentFactor_PID(self, errorMag, dt):
        ''' Calculates the time step adjustment factor when using a PID controller '''
        errorInErrorMagnitude = (errorMag - self.targetError) / self.targetError
        # Minimum value of the equation above is -1, correspondingly, we limit positive values to 1
        errorInErrorMagnitude = min(1, errorInErrorMagnitude)
        return 1 + self.PIDController.getNewSetPoint(errorInErrorMagnitude, dt)

    #### Adaptive Integration Method ####
    def _integrate(self, initVal, initTime, derivativeFunc, dt, firstSameAsLast=None):
        '''
            Integrates a function based on a Butcher tableau defined in self.tableau
            Format expected is:
            [
                [ c2, a21 ],
                [ c3, a31, a32 ],
                ...
                [ b1, b2, ... ]
                [ b1*, b2*, ... ]

            Then for row i of the tableau, ki = derivativefunc(t + ci*dt, y + dt(ai1*k1 + ai2*k2 + ...))
            Then final result is y + dt*(b1*k1 + b2*k2 + ...)
            For these adaptive methods, two rows of b's (b's and b*'s) produce two estimates of the solution
            Subtracting these gives an error estimate which can be used to adjust the time step size
            Also see comment at the top of this file
            Further explanation: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#Use
        '''
        tab = self.tableau

        # Initialize array of derivatives (k's) with first k-value from beginning of interval
        if firstSameAsLast != None:
            # Use first same as last property if possible to save a function evaluation
            k = [ firstSameAsLast ]
        else:
            k = [ derivativeFunc(initTime, initVal) ]

        # Calculate all other k's - one for each row of the tableau except the last two
        for i in range(len(tab) - 2):
            evalDt = dt*tab[i][0]
            evalTime = initTime + evalDt
            
            evalK = k[0] * tab[i][1] # Do not start summation with 0, because could be a non-scalar type
            for a in range(2, len(tab[i])):
                evalK += k[a-1] * tab[i][a]
            evalY = initVal + evalK*dt

            k.append(derivativeFunc(evalTime, evalY))

        lastDerivativeEvaluation = k[-1]

        # Calculate final high/low accuracy results
        # In this summation and the one in the k-calculation loop above, terms are divided by the inverse of the terms they would normally be multiplied by
            # This does not affect the results for scalars
            # But it does prevent rigid body derivative objects from prematurely turning themselves into rigid body state objects (which occurs upon multiplication by a scalar)        
        fineDerivative = k[0] / (1/tab[-2][0]) # Do not start summation with 0, because derivative could be a non-scalar type
        coarseDerivative =  k[0] / (1/tab[-1][0]) # Do not start summation with 0, because derivative could be a non-scalar type
        for i in range(1, len(tab[-1])):
            fineDerivative += k[i] / (1/tab[-2][i])
            coarseDerivative += k[i] / (1/tab[-1][i])

        # Compute error estimate
        finePred = initVal + fineDerivative*dt
        coarsePred = initVal + coarseDerivative*dt
        errorEstimate = (-coarsePred + finePred)
        errorMagEstimate = abs(errorEstimate)

        return finePred, fineDerivative, errorMagEstimate, lastDerivativeEvaluation
