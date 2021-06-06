''' Modeling of the effects of a Launch Rail on rocket launches '''

from MAPLEAF.Motion import (AngularVelocity, ForceMomentSystem, Quaternion,
                            Vector)

__all__ = [ "LaunchRail" ]

class LaunchRail():
    def __init__(self, initialPosition, initialDirection, length, earthRotationRate=0):
        ''' Provide initial position and direction in global inertial frame '''
        self.initialPosition = initialPosition # m
        self.initialDirection = initialDirection.normalize() # Ensure the direction is a unit vector
        self.length = length # m
        self.earthAngVel = earthRotationRate # rad/s, about z-axis

    def getPosition(self, time):
        ''' Launch Rail moves with the earth's surface '''
        rotationSoFar = Quaternion(
            axisOfRotation=Vector(0,0,1), 
            angle=(self.earthAngVel * time)
        )
        return rotationSoFar.rotate(self.initialPosition)

    def getDirection(self, time):
        ''' Launch Rail moves with the earth's surface '''
        rotationSoFar = Quaternion(
            axisOfRotation=Vector(0,0,1), 
            angle=(self.earthAngVel * time)
        )
        return rotationSoFar.rotate(self.initialDirection)

    def getVelocity(self, time):
        pos = self.getPosition(time)
        angVel = AngularVelocity(0, 0, self.earthAngVel)
        return angVel.crossProduct(pos)

    def applyLaunchTowerForce(self, state, time, unadjustedForce):
        '''
            If on launch tower, projects forces experienced onto the launch tower directions and sets Moments = 0
            Returns two values: onTower(Bool), adjustedForce(Motion.ForceMomentSystem)
        '''
        distanceTravelled = (state.position - self.getPosition(time)).length()
        if distanceTravelled < self.length:
            # Vehicle still on launch rail

            # Project total force onto the launch rail direction (dot product)
            adjustedForceMagnitude = unadjustedForce.force * self.getDirection(time)
            adjustedForceVector = self.initialDirection * adjustedForceMagnitude

            # No resultant moments while on the launch rail
            adjustedForce = ForceMomentSystem(adjustedForceVector)

            return adjustedForce
        else:
            # Vehicle has left the rail
            return unadjustedForce

    def applyLaunchTowerMotionConstraint(self, state, time):
        ''' If on launch tower, stops rocket from sliding off the bottom before engine is lit '''
        currPosition = self.getPosition(time)
        currDirection = self.getDirection(time)

        distanceTravelled = (state.position - currPosition) * currDirection
        if distanceTravelled < self.length:
            # Vehicle still on launch rail

            if distanceTravelled <= 0:
                # If vehicle has slid of the bottom of the rail
                # Reset its position to the bottom of the rail
                state.position = currPosition

                # Velocity adjustment
                velocityAlongRail = state.velocity * currDirection
                if velocityAlongRail < 0:
                    # If velocity along rail is also negative, set it to zero relative to the rail
                    state.velocity = self.getVelocity(time)

            return True, state
        else:
            # Vehicle has left the rail
            return False, state
