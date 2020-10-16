"""
Classes that represent 3- and 6-DoF rigid bodies.
Rigid body classes contain the logic for calculating rigid body state derivatives, given current applied forces and inertia of the rigid body
"""

from MAPLEAF.Motion import (AngularVelocity, RigidBodyStateDerivative,
                            RigidBodyStateDerivative_3DoF, integratorFactory)

__all__ = [ "RigidBody_3DoF", "RigidBody", "StatefulRigidBody", "StateList" ]

#### 3 DoF ####
class RigidBody_3DoF:
    """Interface:
        ### Properties ###
        .state = RigidBodyState (pos, vel, orientation, angVel)
        .time = currentSimTime
        
        ### Methods ###
        .timeStep(deltaT) -> advances simulation by deltaT
    """
    
    ''' forceParam and inertiaParam should be functions, they will be passed:
            1) Simulation Time
            2) RigidBodyState object containing the rigidBody's Position, Velocity, Orientation and AngularVelocity
        Functions are expected to return:
            1) forceParam: A Force object containing the total force and moment applied to the rigidBody at it's CG location in the local frame of reference
            2) inertiaParam: An Inertia object containing the mass of the object (other fields ignored)
                If the rigid body is representing a CompositeObject, pass in a reference to the getMass function here
    '''
    def __init__(self, rigidBodyState, forceParam, inertiaParam, startTime=0, integrationMethod="Euler", discardedTimeStepCallback=None, simDefinition=None):
        self.time = startTime
        self.state = rigidBodyState
        
        self.forceFunc = forceParam
        self.inertiaFunc = inertiaParam
          
        self.integrate = integratorFactory(integrationMethod=integrationMethod, simDefinition=simDefinition, discardedTimeStepCallback=discardedTimeStepCallback)

    def rigidBodyStateDerivative(self, time, state):
        force = self.forceFunc(time, state).force
        
        DposDt = state.velocity
        DvelDt = force / self.inertiaFunc(time, state)

        return RigidBodyStateDerivative_3DoF(DposDt, DvelDt)

    def timeStep(self, deltaT):
        self.state, timeStepAdaptionFactor, deltaT = self.integrate(self.state, self.time, self.rigidBodyStateDerivative, deltaT)
        self.time += deltaT
        return timeStepAdaptionFactor, deltaT

#### 6 DoF ####
class RigidBody(RigidBody_3DoF):
    """
        6DoF version of RigidBody_3DoF. Calculates angular velocities and accelerations

        Interface:
            Properties:
                .state = RigidBodyState (pos, vel, orientation, angVel)
                .time = currentSimTime
            
            External functions wrapped by this class
                .forceFunc(time, state)
                .inertiaFunc(time, state)

            Methods
                .timeStep(deltaT)
    """
    #TODO: Moment of Inertia Tensor? Required for aircraft

    def __init__(self, rigidBodyState, forceParam, inertiaParam, integrationMethod="Euler", discardedTimeStepCallback=None, simDefinition=None):
        ''' Just calls RigidBodyState_3DoF constructor '''
        super().__init__(rigidBodyState, forceParam, inertiaParam, integrationMethod=integrationMethod, simDefinition=simDefinition, discardedTimeStepCallback=discardedTimeStepCallback)

    def rigidBodyStateDerivative(self, time, state):
        # Forces are expected to be calculated in a body frame, where each coordinate axis is aligned with a pricipal axis
        appliedForce_localFrame = self.forceFunc(time, state)
        inertia = self.inertiaFunc(time, state)

        # Get CG velocity relative to body geometry
        dt = 1e-8
        inertia2 = self.inertiaFunc(time+dt, state)
        CGVel_local = (inertia2.CG - inertia.CG) / dt
        CGVel_global = state.orientation.rotate(CGVel_local)

        ### Translation - calculated in global frame ###
        #Convert from local to global frame
        fVec_global = state.orientation.rotate(appliedForce_localFrame.force)        
        linAccel_global = fVec_global / inertia.mass

        ### Rotation - calculated in local frame (Euler equations) ###
        # convert angular velocity to global frame - to be added to orientation
        angVel_global = AngularVelocity(*state.orientation.rotate(state.angularVelocity)) # Will be transformed into a quaternion once multiplied by a timestep
        # Calc angular acceleration (in local frame)
        moi = inertia.MOI
        dAngVelDtX = (appliedForce_localFrame.moment.X + (moi.Y - moi.Z) * state.angularVelocity.Y * state.angularVelocity.Z) / moi.X
        dAngVelDtY = (appliedForce_localFrame.moment.Y + (moi.Z - moi.X) * state.angularVelocity.Z * state.angularVelocity.X) / moi.Y
        dAngVelDtZ = (appliedForce_localFrame.moment.Z + (moi.X - moi.Y) * state.angularVelocity.X * state.angularVelocity.Y) / moi.Z
        dAngVelDt = AngularVelocity(dAngVelDtX, dAngVelDtY, dAngVelDtZ)

        return RigidBodyStateDerivative(state.velocity+CGVel_global, linAccel_global, angVel_global, dAngVelDt)

class StatefulRigidBody(RigidBody):
    '''
        Intended to represent rigid bodies that have additional stateful properties outside of their rigd body state (ex. tank levels, actuator positions).
        State is defined a by a `StateList` instead of by a `MAPLEAF.Motion.RigidBodyState`.
    '''
    def __init__(self, rigidBodyState, forceParam, inertiaParam, integrationMethod="Euler", discardedTimeStepCallback=None, simDefinition=None):
        super().__init__(rigidBodyState, forceParam, inertiaParam, integrationMethod, discardedTimeStepCallback, simDefinition)

        self.state = StateList([ rigidBodyState ])
        self.derivativeFuncs = [ self.rigidBodyStateDerivative ]

    def addStateVariable(self, name, currentValue, derivativeFunction):
        ''' 
            Pass in the current value of a parameter which needs to be integrated, and a derivative function
            Derivative function should accept the current time and StateList as inputs and return the derivative of the new parameter
        '''
        self.state.addStateVariable(name, currentValue)
        self.derivativeFuncs.append(derivativeFunction)

    def getStateDerivative(self, time, state):
        return StateList([ derivativeFunc(time, state) for derivativeFunc in self.derivativeFuncs ], _nameToIndexMap=state.nameToIndexMap)

    def timeStep(self, deltaT):
        self.state, timeStepAdaptationFactor, deltaT = self.integrate(self.state, self.time, self.getStateDerivative, deltaT)
        self.time += deltaT
        return timeStepAdaptationFactor, deltaT

class StateList(list):
    ''' 
        Purpose is to provide a generalized version of RigidBodyState, which can be integrated like a scalar, but allows MAPLEAF to integrate arbitrary additional parameters.

        Contains a list of parameters that define the (rocket) state, all of which are to be integrated
        Overrides operators to make +-* operations elementwise
        Capable of representing a StateList and the derivative of a StateList

        Called StateList instead of StateVector because it can contain objects of any type, like a Python list, and unlike a Vector.

        Example: RocketState([ initRigidBodyState, tankLevel1, actuatorPosition1 ])
    '''
    #### Initialization and setting/getting attributes ####
    def __init__(self, stateVariables, variableNames=None, _nameToIndexMap=None):
        '''
            stateVariables: (`list`) of state variable values (RigidBodyState assumed to be in position 0)
            variableNames:  (`list[str]`) of state variable names. If passed in, can access values as attributes: stateList.varName
           _ nameToIndexMap:(`dict[str:int]`) maps variable names to indices in the itemList. If not provided, but a nameList is, will be generated from the nameList

            Pass in a nameList to be able to access variables by name - order must match that of the itemList.
                For unnamed variables, put None in the namelist

            Only pass in one of nameList or _nameToIndexMap. When both are passed in, _nameToIndexMap takes precedence.
                _nameToIndexMap intended for internal use only, no correctness checks performed on it
        '''
        super().__init__(stateVariables) # Pass itemList to parent (list) constructor

        if _nameToIndexMap != None:
            # Does not check whether each variable is in the nameToIndexMap - could omit some if desired
            self.nameToIndexMap = _nameToIndexMap

        elif variableNames != None:
            # Generate nameToIndexMap from variableNames list
            if len(variableNames) == len(stateVariables):
                forbiddenVarNames = [ "position", "velocity", "orientation", "angularVelocity" ]
                for forbiddenName in forbiddenVarNames:
                    if forbiddenName in variableNames:
                        raise ValueError("ERROR: The following variable names are reserved for rigid body states: {}".format(forbiddenVarNames))

                self.nameToIndexMap = { variableNames[i]:i for i in range(len(variableNames)) }
                if len(variableNames) != len(self.nameToIndexMap.keys()):
                    raise ValueError("ERROR: Duplicate state variable name in: {}".format(variableNames))
            else:
                raise ValueError("ERROR: Number of state variables must match number of variable names provided")
        
        else:
            self.nameToIndexMap = dict()

    def __getattr__(self, name):
        try:
            # Check if the attribute name is in the nameToIndexMap - return item from that index
            return self[self.nameToIndexMap[name]]
        except KeyError:
            # Try getting the attribute from the rigidBodyState (assumed first element)
            return getattr(self[0], name)

    def __setattr__(self, name, value):
        if name in self.__dict__ or name == "nameToIndexMap":
            self.__dict__[name] = value
        elif name in self.nameToIndexMap:
            # Check if the attribute name is in the nameToIndexMap - set item at that index
            indexToSet = self.nameToIndexMap[name]
            self[indexToSet] = value
        elif name in self[0].__dict__:
            # Try getting the attribute from the rigidBodyState (assumed first element)
            setattr(self[0], name, value)

    def addStateVariable(self, name, currentValue):
        self.append(currentValue)
        self.nameToIndexMap[name] = len(self)-1

    #### Arithmetic ####
    def __add__(self, state2):
        return StateList([ x + y for x, y in zip(self, state2) ], _nameToIndexMap=self.nameToIndexMap)

    def __sub__(self, state2):
        return StateList([ x - y for x, y in zip(self, state2) ], _nameToIndexMap=self.nameToIndexMap)

    def __mul__(self, scalar):
        return StateList([ x*scalar for x in self ], _nameToIndexMap=self.nameToIndexMap)

    def __truediv__(self, scalar):
        return StateList([ x/scalar for x in self ], _nameToIndexMap=self.nameToIndexMap)

    def __abs__(self):
        return sum([ abs(x) for x in self ])

    def __eq__(self, state2):
        try:
            return all([ x == y for x,y in zip(self, state2) ])
        except TypeError:
            return False

    def __neg__(self):
        return StateList([ -x for x in self ], _nameToIndexMap=self.nameToIndexMap)

    #### String functions ####
    def getLogHeader(self):
        header = ""
        for i in range(len(self)):
            try:
                # If the variable defines a getLogHeader function, use it
                header += self[i].getLogHeader()
            
            except AttributeError:
                # Item doesn't have a getLogHeader function (ex. it's a float)
                # Try to find it in nameToIndexMap
                varName = None
                for key, val in self.nameToIndexMap.items():
                    if val == i:
                        varName = " " + key
                
                # Otherwise just call it stateVariableN
                if varName == None:
                    varName = " StateVariable{}".format(i)

                header += varName
        
        return header

    def __str__(self):
        varStrings = [ x.__str__() for x in self ]
        return " ".join(varStrings)