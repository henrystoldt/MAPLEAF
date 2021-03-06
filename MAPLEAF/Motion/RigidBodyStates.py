''' 
Define standard and time-derivative rigid body states.  
These are defined in such a way that, for the purposes of Runge-Kutta motion integration, they can be treated like scalars.
'''

from MAPLEAF.Motion import Vector
from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import AngularVelocity

__all__ = [ "RigidBodyState_3DoF", "RigidBodyStateDerivative_3DoF", "RigidBodyState", "RigidBodyStateDerivative", "interpolateRigidBodyStates", "StateList" ]

class RigidBodyState_3DoF():
    """ Class created to be able to treat rigidBody states like scalars when integrating the movement of a rigid body
            Pos/Vel are expected to be Vectors - Defined with reference to the global frame

        Adding rigidBodyStates adds the vectors

        Multiplying an rigidBodyState by a scalar scales the vectors
            0.5 *  = half the vector length
    """
    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity

    def __add__(self, rigidBodyState2):
        ''' Used in: initVal {+} timeStep * slope '''
        newPos = self.position + rigidBodyState2.position
        newVel = self.velocity + rigidBodyState2.velocity
        
        return RigidBodyState_3DoF(newPos, newVel)

    def __mul__(self, scalar):
        ''' Used to negate the rigid body state for subtractions '''
        scalar = float(scalar)
        newPos = self.position * scalar
        newVel = self.velocity * scalar

        return RigidBodyState_3DoF(newPos, newVel)

    def __abs__(self):
        ''' Used to quantify the difference between two RigidBodyStates as a scalar value during error estimation in adaptive time stepping methods '''
        #TODO: Ensure this scales properly with the version in the 6DoF IntegratbleRigidBodyState __abs__ function

        return self.position.length() + self.velocity.length()

    def __str__(self):
        ''' Called by print '''
        return " {:>10.3f} {:>10.4f}".format(self.position, self.velocity)

    def __neg__(self):
        return self*-1

class RigidBodyStateDerivative_3DoF():
    def __init__(self, velocity, acceleration):
        self.velocity = velocity
        self.acceleration = acceleration

    def __add__(self, rigidBodyState2):
        ''' Used in: initVal {+} timeStep * slope '''
        newPos = self.velocity + rigidBodyState2.velocity
        newVel = self.acceleration + rigidBodyState2.acceleration
        
        return RigidBodyStateDerivative_3DoF(newPos, newVel)

    def __mul__(self, timeStep):
        ''' Used in: initVal + timeStep {*} slope '''
        timeStep = float(timeStep)
        dPos = self.velocity * timeStep
        dVel = self.acceleration * timeStep

        # After multiplying by a timestep, we get a regular (integrated) rigid body state back
        return RigidBodyState_3DoF(dPos, dVel)

    def __truediv__(self, invScalar):
        ''' 
            Used in (k1 + k4) {/} 2 
            Does not 'integrate' the result to make a rigidBodyState, returns a new rigidBodyStateDerivative
        '''
        invScalar = 1/float(invScalar)

        vel = self.velocity * invScalar
        accel = self.acceleration * invScalar

        return RigidBodyStateDerivative_3DoF(vel, accel)

    ### Wrapper/Thin functions ###
    def __rmul__(self, scalar):
        return self * scalar

class RigidBodyState():
    """ Class created to be able to treat rigidBody states like scalars when integrating the movement of a rigid body
            Pos/Vel are expected to be Vectors - Defined with reference to the global frame
            Orientation is expected to be a Quaternion - Defines the rotation from the global inertial reference frame to the rocket's local frame 
                (Orientation of the rocket in the global frame)
            Angular Velocity is expected to be an Angular Velocity - Defined with reference to the local frame

        Adding rigidBodyStates adds the vectors and multiplies the quaternions (which adds the rotations they represent)

        Multiplying an rigidBodyState by a scalar scales the vectors and rotation defined by the quaternions
            0.5 *  = half the vector length, half the rotation size, directions the same

        No other operations are defined
    """
    def __init__(self, position=None, velocity=None, orientation=None, angularVelocity=None):
        self.position = Vector(0,0,0) if (position is None) else position
        self.velocity = Vector(0,0,0) if (velocity is None) else velocity
        self.orientation = Quaternion(1,0,0,0) if (orientation is None) else orientation
        self.angularVelocity = AngularVelocity(0,0,0) if (angularVelocity is None) else angularVelocity
    
    def __add__(self, rigidBodyState2):
        ''' Used in: initVal {+} (timeStep * slope) '''
        newPos = self.position + rigidBodyState2.position
        newVel = self.velocity + rigidBodyState2.velocity
        newAngVel = self.angularVelocity + rigidBodyState2.angularVelocity
        newOrientation = rigidBodyState2.orientation * self.orientation.normalize()
        
        return RigidBodyState(newPos, newVel, newOrientation.normalize(), newAngVel)

    def __mul__(self, timeStep):
        '''
            Used in: initVal + timeStep {*} slope
            Expected to always be multiplied by a scalar
        '''
        timeStep = float(timeStep)

        newPos = self.position * timeStep
        newVel = self.velocity * timeStep
        newAngVel = self.angularVelocity * timeStep
        newOrientation = self.orientation.scaleRotation(timeStep)

        return RigidBodyState(newPos, newVel, newOrientation, newAngVel)

    def __abs__(self):
        ''' Used to quantify the difference between two RigidBodyStates as a scalar value during error estimation in adaptive time stepping methods '''

        positionMag = self.position.length() + self.velocity.length()
        orientationMag = abs(self.orientation.rotationAngle()) + self.angularVelocity.angVel()

        return orientationMag*100 + positionMag

    def __eq__(self, iRBS2):
        try:
            properties = [ self.position, self.velocity, self.orientation, self.angularVelocity ]
            otherProperties = [ iRBS2.position, iRBS2.velocity, iRBS2.orientation, iRBS2.angularVelocity ]
            return all([ x == y for (x,y) in zip(properties, otherProperties) ])
        except AttributeError:
            return False

    def __str__(self):
        ''' Called by print function '''
        return " {:>10.3f} {:>10.4f} {:>11.7f} {:>9.4f}".format(self.position, self.velocity, self.orientation, self.angularVelocity)

    ### Wrapper/Thin functions ###
    def __neg__(self):
        return RigidBodyState(self.position*-1, self.velocity*-1, self.orientation.conjugate(), self.angularVelocity*-1)

class RigidBodyStateDerivative():
    def __init__(self, velocity, acceleration, angularVelocity, angularAccel):
        self.velocity = velocity
        self.acceleration = acceleration
        self.angularVelocity = angularVelocity
        self.angularAccel = angularAccel

    def __add__(self, state2):
        ''' Used in: initVal {+} (timeStep * slope) '''
        newVel = self.velocity + state2.velocity
        newAccel = self.acceleration + state2.acceleration
        newAngVel = self.angularVelocity + state2.angularVelocity
        newAngAccel = self.angularAccel + state2.angularAccel
        
        return RigidBodyStateDerivative(newVel, newAccel, newAngVel, newAngAccel)

    def __mul__(self, timeStep):
        '''
            Used in: timeStep {*} slope
            Expected to always be multiplied by a scalar, timestep
            Returns integrated change in rigid body state over given time step
        '''
        timeStep = float(timeStep)

        dPos = self.velocity * timeStep
        dVel = self.acceleration * timeStep
        dOrientation = (self.angularVelocity * timeStep).toQuaternion()
        dOrientation.normalize()
        dAngVel = self.angularAccel * timeStep

        # After integration over a time step, get a regular rigid body state back
        return RigidBodyState(dPos, dVel, dOrientation, dAngVel)

    def __truediv__(self, invScalar):
        ''' 
            Used in (k1 + k4) {/} 2 
            Does not 'integrate' the result to make a rigidBodyState, returns a new rigidBodyStateDerivative
        '''
        invScalar = 1/float(invScalar)

        vel = self.velocity * invScalar
        accel = self.acceleration * invScalar
        angVel = self.angularVelocity * invScalar
        angAccel = self.angularAccel * invScalar

        return RigidBodyStateDerivative(vel, accel, angVel, angAccel)

    def __abs__(self):
        ''' Used to quantify the difference between two RigidBodyStates as a scalar value during error estimation in adaptive time stepping methods '''

        # positionMag = self.position.length() + self.velocity.length()

        orientationMag = self.angularVelocity.angVel() + self.angularAccel.angVel()

        return orientationMag*100

    def __eq__(self, state2):
        try:
            properties = [ self.velocity, self.acceleration, self.angularVelocity, self.angularAccel ]
            otherProperties = [ state2.velocity, state2.acceleration, state2.angularVelocity, state2.angularAccel ]
            return all([ x == y for (x,y) in zip(properties, otherProperties) ])
        except AttributeError:
            return False

    ### Wrapper/Thin functions ###
    def __rmul__(self, scalar):
        return self * scalar

class StateList(list):
    ''' 
        Purpose is to provide a generalized version of RigidBodyState, which can be integrated like a scalar, but allows MAPLEAF to integrate arbitrary additional parameters.

        Contains a list of parameters that define the (rocket) state, all of which are to be integrated
        Overrides operators to make +-* operations elementwise
        Capable of representing a StateList and the derivative of a StateList

        Called StateList instead of StateVector because it can contain objects of any type, like a Python list, and unlike a Vector.
                
        ..note: If we want to compile this object w/ Cython in the future, We may want to restrict ourselves to scalars

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

    def __rmul__(self, scalar):
        return self * scalar # Call regular __mul__ function

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
    # TODO: Convert to new logging framework
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

def interpolateRigidBodyStates(state1, state2, state1Weight):
    '''
        Linearly interpolates between state 1 and state2.
        state1Weight should be a decimal value between 0 and 1.
    '''
    state2Weight = 1 - state1Weight
    
    # Properties of all rigid body states
    pos = state1.position*state1Weight + state2.position*state2Weight
    vel = state1.velocity*state1Weight + state2.velocity*state2Weight

    try:
        # 6DoF Properties
        orientationDelta = state1.orientation.slerp(state2.orientation, state1Weight) # Use spherical linear interpolation for quaternions
        orientation = state1.orientation * orientationDelta
        angVel = state1.angularVelocity*state1Weight + state2.angularVelocity*state2Weight
        return RigidBodyState(pos, vel, orientation, angVel)

    except AttributeError:
        # 3DoF doesn't include orientation / angVel
        return RigidBodyState_3DoF(pos, vel)