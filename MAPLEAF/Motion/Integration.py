''' Defines ODE integrators for constant and adaptive time stepping. Used by the `MAPLEAF.Motion.RigidBody` classes to integrate rocket motion '''

import math

__all__ = [ "integratorFactory" ]

def checkButcherTableau(tableau):
    ''' 
        Checks that the Butcher tableau passed in represents a consistent R-K method 
        Expected Format (Example is RK4 - 3/8 method):
            [                                   # Top 0 row ommitted
                [ 1/3, 1/3 ],                   # Row 1: all coefficients sequentially
                [ 2/3, -1/3, 1.0 ],             # Row 2: ''
                [ 1.0, 1.0, -1.0, 1.0 ],        # Row 3: ''
                [ 1/8, 3/8, 3/8, 1/8 ]          # Row 4: (result calculation row) - empty space on left ignored, just enter all coefficients
            ]

        Learn about Butcher Tableaus here: https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
            RK4 - 3/8 Butcher Tableau is present there in the Examples section
    '''

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
        return Integrator(method=integrationMethod)

class Integrator:
    ''' Callable class for constant-dt ODE integration '''

    def __init__(self, method="Euler"):
        # Save integration method and associated
        self.method = method
        self.tableau = None
        if method == "Euler":
            self.integrate = self.IntegrateEuler
        elif method == "RK2Midpoint":
            self.integrate = self.IntegrateRK2MidPoint
        elif method == "RK2Heun":
            self.integrate = self.IntegrateRK2Heun
        elif method == "RK4":
            self.integrate = self.IntegrateRK4
        elif method == "RK4_3/8":
            self.integrate = self.IntegrateByButcherTableau
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

    def integrate(self, initVal, initTime, derivativeFunc, dt):
        ''' Template method, replaced by one of the Constant time step methods below in self.__init__ '''
        pass

    #### Constant time step methods ####
    def IntegrateEuler(self, initVal, initTime, derivativeFunc, dt):
        yPrime = derivativeFunc(initTime, initVal)
        return initVal + yPrime * dt, 1.0, dt

    def IntegrateRK2MidPoint(self, initVal, initTime, derivativeFunc, dt):
        initSlope = derivativeFunc(initTime, initVal)

        halfwayVal = initVal + initSlope*(dt/2)
        halfwayTime = initTime + dt/2
        halfwaySlope = derivativeFunc(halfwayTime, halfwayVal)

        return initVal + halfwaySlope*dt, 1.0, dt

    def IntegrateRK2Heun(self, initVal, initTime, derivativeFunc, dt):
        '''
            Integrates a function using second order Runge Kutta Method (Heun Variant).
            Inputs:
                initVal: Implement (+, *) operator
                initTime: Implement (+) operator
                derivativeFunc: Function(time, val)
                    Should return an object of the same type as initVal, representing the derivative
                    Accepts time and value objects of the same types as initTime, initVal
                dt: Must implement multiplication
            Outputs:
                Returns estimated integral value after timestep dt
        '''
        initSlope = derivativeFunc(initTime, initVal)

        endEstimate = initVal + initSlope*dt
        endSlope = derivativeFunc(initTime + dt, endEstimate)

        slopeEstimate = (endSlope + initSlope)/2
        return initVal + slopeEstimate*dt, 1.0, dt

    def IntegrateRK4(self, initVal, initTime, derivativeFunc, dt):
        k1 = derivativeFunc(initTime, initVal)

        halfwayVal = initVal + k1*(dt/2)
        halfwayTime = initTime + (dt/2)
        k2 = derivativeFunc(halfwayTime, halfwayVal)

        halfwayVal2 = initVal + k2*(dt/2)
        k3 = derivativeFunc(halfwayTime, halfwayVal2)

        endVal = initVal + k3*dt
        endTime = initTime + dt
        k4 = derivativeFunc(endTime, endVal)

        slopeEstimate = ((k1+k4)/2 + k2 + k3) / 3
        return initVal + slopeEstimate*dt, 1.0, dt

    def IntegrateByButcherTableau(self, initVal, initTime, derivativeFunc, dt):
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
            evalTime = initTime + dt*tab[i][0]
            dy = k[0] * tab[i][1]
            for a in range(2, len(tab[i])):
                dy = dy + k[a-1] * tab[i][a]
            evalY = initVal + dy*dt
            k.append(derivativeFunc(evalTime, evalY))

        # Calculate final result using last row of coefficients
        totalDerivative = tab[-1][0] * k[0]
        for i in range(1, len(tab[-1])):
            totalDerivative = totalDerivative + tab[-1][i]*k[i]

        return (initVal + totalDerivative*dt), 1.0, dt        

class AdaptiveIntegrator():
    ''' Callable class for adaptive-dt ODE integration '''

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
            self.integrate = self.IntegrateByButcherTableau_Adaptive
            self.tableau = [
                [ 0.5, 0.5 ],
                [ 0.0, 1.0 ],
                [ 1.0, 0.0 ]
            ]
            self.firstSameAsLast = False
        elif method == "RK23Adaptive": # Bogacki-Shampine method
            self.integrate = self.IntegrateByButcherTableau_Adaptive
            self.tableau = [
                [ 0.5, 0.5 ],
                [ 3/4, 0.0, 3/4 ],
                [ 1.0, 2/9, 1/3, 4/9 ],
                [ 2/9, 1/3, 4/9, 0.0 ],
                [ 7/24, 1/4, 1/3, 1/8 ]
            ]
            self.firstSameAsLast = True
        elif method == "RK45Adaptive": # Dormand-Prince RK5(4)7FM method
            self.integrate = self.IntegrateByButcherTableau_Adaptive
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
            self.integrate = self.IntegrateByButcherTableau_Adaptive
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

        # If a butcher tableau is used to define the R-K method, check that it is valid
        if self.tableau != None:
            checkButcherTableau(self.tableau)

        # Save limiter info and target error, required for any adaptation scheme
        self.maxFactor, self.minFactor, self.safetyFactor = maxMinSafetyFactors
        self.targetError = targetError
        self.maxTimeStep = maxTimeStep
        self.minTimeStep = minTimeStep
        self.discardedTimeStepCallback = discardedTimeStepCallback

        ### Set up the chosen adaptation method ###
        if controller == "elementary":
            self.getTimeStepAdjustmentFactor = self.getTimeStepAdjustmentFactor_Elementary

        elif controller == "PID":
            self.safetyFactor = 1
            P, I, D = PIDCoeffs
            
            # Delayed import of GNC.PID to avoid circular import problems
            from MAPLEAF.GNC import PIDController
            self.PIDController = PIDController(P, I, D, maxIntegral=1)

            self.getTimeStepAdjustmentFactor = self.getTimeStepAdjustmentFactor_PID

        elif controller == "Constant":
            self.getTimeStepAdjustmentFactor = self.getTimeStepAdjustmentFactor_Constant

        else:
            raise ValueError("Adaptive Integrator requires non-constant step size controller such as 'PID' or 'elementary'")

    def __call__(self, initVal, initTime, derivativeFunc, dt):
        return self.errorLimitedAdaptiveIntegration(initVal, initTime, derivativeFunc, dt)

    def integrate(self, initVal, initTime, derivativeFunc, dt):
        ''' Template method, replaced by one of the Constant time step methods below in self.__init__ '''
        pass

    def errorLimitedAdaptiveIntegration(self, initVal, initTime, derivativeFunc, dt):
        # Set up incorrect values to force first iteration, like a do-while loop
        errorMagEstimate = 10000000
        maxErrorMultiple = 100 # Will recompute timestep if estimated error > maxErrorMultiple * targetError
        dt *= 3

        # Loop will lower the actual time step taken if error is more than 3*target
        while errorMagEstimate > maxErrorMultiple*self.targetError:
            if errorMagEstimate != 10000000:
                # This means this is not the first loop. Therefore a previously-computed time step has been discarded, and will be recomputed below with a smaller time step
                # if self.discardedTimeStepCallback != None:
                self.discardedTimeStepCallback(self)

            dt = max(self.minTimeStep, dt/3)
            result, errorMagEstimate, lastDerivativeEvaluation = self.integrate(initVal, initTime, derivativeFunc, dt, self.derivativeCache)
            if math.isclose(dt,self.minTimeStep):
                break
        
        if self.firstSameAsLast:
            self.derivativeCache = lastDerivativeEvaluation

        adaptFactor = self.getTimeStepAdjustmentFactor(errorMagEstimate, dt)

        # Correct finer result with error estimate (Richardson Extrapolation), timestepAdaptationFactor, actual time step taken
        return result, adaptFactor, dt

    #### Time Step adjustment ####
    def limitAdaptationFactor(func):
        ''' 
            Function decorator - limits the returned adaptation factor to enforce min/max time step size and min/max adaptation factor restrictions 
            Expects the wrapped function to return two values: desiredAdaptationFactor, currentTimeStep
        '''

        def limitedDtControllerFunction(*args, **kwargs):
            # Calculate desired adaptation factor using the wrapped function
            self, desiredAdaptFactor, currentTimeStep = func(*args, **kwargs)

            ### Apply adaptation limiters ###
            # Adaptation can be limited in two ways: by self.maxFactor/self.minFactor and by self.minTimeStep/self.maxTimeStep.
                # The below checks which limitation is currently most restrictive and applies that one
            
            # Calculate min/max adaptation factors based on min/max time step size restrictions
            minFactor2 = self.minTimeStep / currentTimeStep
            maxFactor2 = self.maxTimeStep / currentTimeStep

            # Calculate resulting total min/max factors
            minFactor = max(minFactor2, self.minFactor)
            maxFactor = min(maxFactor2, self.maxFactor)

            # Apply limits
            return self.safetyFactor * min(max(desiredAdaptFactor, minFactor), maxFactor)
        
        return limitedDtControllerFunction

    # Constant dt does not require limiting
    def getTimeStepAdjustmentFactor_Constant(self, errorMag, dt):
        ''' Calculates the time step adjustment factor when using a constant time stepping (always 1.0) '''
        return 1

    @limitAdaptationFactor
    def getTimeStepAdjustmentFactor_Elementary(self, errorMag, dt):
        ''' Calculates the time step adjustment factor when using an elementary controller '''
        return self, (self.targetError / (2*errorMag))**0.5, dt

    @limitAdaptationFactor
    def getTimeStepAdjustmentFactor_PID(self, errorMag, dt):
        ''' Calculates the time step adjustment factor when using a PID controller '''
        errorInErrorMagnitude = (errorMag - self.targetError) / self.targetError
        return self, 1 + self.PIDController.getNewSetPoint(errorInErrorMagnitude, dt), dt

    #### Adaptive Integration Method ####
    def IntegrateByButcherTableau_Adaptive(self, initVal, initTime, derivativeFunc, dt, firstSameAsLast=None):
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
            It is assumed that the 
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
            evalTime = initTime + dt*tab[i][0]
            dy = k[0] * tab[i][1]
            for a in range(2, len(tab[i])):
                dy = dy + k[a-1] * tab[i][a]
            evalY = initVal + dy*dt
            k.append(derivativeFunc(evalTime, evalY))

        lastDerivativeEvaluation = k[-1]

        # Calculate final high/low accuracy results
        fineDerivative = tab[-2][0] * k[0]
        coarseDerivative = tab[-1][0] * k[0]
        for i in range(1, len(tab[-1])):
            fineDerivative = fineDerivative + tab[-2][i]*k[i]
            coarseDerivative = coarseDerivative + tab[-1][i]*k[i]

        # Compute error estimate
        finePred = initVal + fineDerivative*dt
        coarsePred = initVal + coarseDerivative*dt
        errorEstimate = (-coarsePred + finePred)
        errorMagEstimate = abs(errorEstimate)

        return finePred, errorMagEstimate, lastDerivativeEvaluation
