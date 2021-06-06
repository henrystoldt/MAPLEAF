'''
Temporarily holds simulation results.
Used for creating flight animations.
'''

from math import sqrt

from MAPLEAF.Motion import Vector

__all__ = [ "RocketFlight" ]

class RocketFlight():
    ''' Holds simulation results '''

    def __init__(self):
        '''  These arrays are filled in during a flight simulation. If the rocket is not controlled, self.actuatorDefls will be set to None '''
        self.times = []
        self.rigidBodyStates = []
        self.actuatorDefls = []
        self.actuatorTargetDefls = []

        # To be filled during flight
        self.engineOffTime = None
        self.mainChuteDeployTime = None
        self.targetLocation = None

    def getFlightTime(self):
        return self.times[-1]

    def getApogee(self):
        maxAltitude = -10000000
        for state in self.rigidBodyStates:
            if state.position.Z > maxAltitude:
                maxAltitude = state.position.Z
        return maxAltitude

    def getMaxSpeed(self):
        maxVel = 0
        for state in self.rigidBodyStates:
            if state.velocity.length() > maxVel:
                maxVel = state.velocity.length()
        return maxVel

    def getMaxHorizontalVel(self):
        maxHorizontalVel = 0
        for state in self.rigidBodyStates:
            horizVel = sqrt(state.velocity.X**2 + state.velocity.Y**2)
            if horizVel > maxHorizontalVel:
                maxHorizontalVel = horizVel
        return maxHorizontalVel

    def getLandingLocation(self):
        '''
            Extrapolates to z = 0
            If the simulation stopped before z = 0 is reached, the last position is returned
        '''

        #TODO: Handle non-flat surface to land on
        secondLastPosition = self.rigidBodyStates[-2].position
        lastPosition = self.rigidBodyStates[-1].position

        dz = lastPosition.Z - secondLastPosition.Z

        if lastPosition.Z > dz*2:
            dxdz = (lastPosition.X - secondLastPosition.X) / dz
            dydz = (lastPosition.Y - secondLastPosition.Y) / dz

            dzLand = -lastPosition.Z 

            xLand = lastPosition.X + dxdz*dzLand
            yLand = lastPosition.Y + dydz*dzLand
            zLand = 0.0
            
            return Vector(xLand, yLand, zLand)
        else:
            return lastPosition
