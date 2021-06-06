"""
Classes that represent 3- and 6-DoF rigid bodies.
Rigid body classes contain the logic for calculating rigid body state derivatives, given current applied forces and inertia of the rigid body
"""

from MAPLEAF.Motion import (AngularVelocity, RigidBodyStateDerivative,
                            RigidBodyStateDerivative_3DoF, StateList, integratorFactory, Vector)

__all__ = [ "RigidBody_3DoF", "RigidBody", "StatefulRigidBody" ]

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

        self.lastStateDerivative = RigidBodyStateDerivative_3DoF(rigidBodyState.velocity, Vector(0,0,0))

    def getRigidBodyStateDerivative(self, time, state):
        force = self.forceFunc(time, state).force
        
        DposDt = state.velocity
        DvelDt = force / self.inertiaFunc(time, state)

        return RigidBodyStateDerivative_3DoF(DposDt, DvelDt)

    def timeStep(self, deltaT):
        # TODO: Current derivative estimates are first-order, replace with a more accurate iterative method
        self.state.estimatedDerivative = self.lastStateDerivative

        integrationResult = self.integrate(self.state, self.time, self.getRigidBodyStateDerivative, deltaT)

        # This is where the simulation time and state are kept track of
        self.time += integrationResult.dt
        self.state = integrationResult.newValue

        # Save for next time step
        self.lastStateDerivative = integrationResult.derivativeEstimate

        return integrationResult

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
        self.lastStateDerivative = RigidBodyStateDerivative(rigidBodyState.velocity, Vector(0,0,0), rigidBodyState.angularVelocity, AngularVelocity(0,0,0))

    def getRigidBodyStateDerivative(self, time, state):
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
        self.derivativeFuncs = [ self.getRigidBodyStateDerivative ]

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
        # TODO: Current derivative estimates are first-order, replace with a more accurate iterative method
        self.state.estimatedDerivative = self.lastStateDerivative

        integrationResult = self.integrate(self.state, self.time, self.getStateDerivative, deltaT)

        # This is where the simulation time and state are kept track of
        self.time += integrationResult.dt
        self.state = integrationResult.newValue

        # Save for next time step
        self.lastStateDerivative = integrationResult.derivativeEstimate

        return integrationResult