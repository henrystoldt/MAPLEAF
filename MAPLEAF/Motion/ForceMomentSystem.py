import math
import MAPLEAF.Rocket.AeroFunctions as AeroFunctions
from MAPLEAF.Motion.CythonVector import Vector
from MAPLEAF.Motion.CythonQuaternion import Quaternion


class ForceMomentSystem():
    '''
        Defines an applied force-moment pair at a location.
        Can be moved to other locations - recalculates the moment accordingly.
    '''
    __slots__ = [ "force", "location", "moment" ]

    def __init__(self, force, location=None, moment=None):
        self.force = force

        # Avoids using mutable default values
        if location == None:
            location = Vector(0,0,0)
        self.location = location

        if moment == None:
            moment = Vector(0,0,0)
        self.moment = moment

    def __add__(self, force2):
        '''
            New force/moment system is calculated about self.location
        '''
        force2AtPresentLocation = force2.getAt(self.location)
        newForce = self.force + force2AtPresentLocation.force
        newMoment = self.moment + force2AtPresentLocation.moment
        return ForceMomentSystem(newForce, self.location, newMoment)

    def __neg__(self):
        return ForceMomentSystem(-self.force, self.location, -self.moment)

    def __sub__(self, force2):
        return self + (-force2)

    def getAt(self, newLocation):
        '''
            Returns a new force object, where the application location has been changed and the applied moment has been updated accordingly

            Moving the force/moment pair application to a new application location does not change the resulting applied force
            Only the applied moment changes to compensate for the force application location change
        '''
        # Find moment that the old force produces about the new location
        newToOld = self.location - newLocation
        momentAppliedAboutNewLocation = newToOld.crossProduct(self.force)
        # Add it to the previous moment
        return ForceMomentSystem(self.force, newLocation, self.moment + momentAppliedAboutNewLocation)

    def __eq__(self, force2):
        return self.force == force2.force and self.location == force2.location and self.moment == force2.moment

    @classmethod
    def fromCoefficients(self, rocketState, environment, Cd, CN, CPLocation, refArea, moment=None):
        ''' 
            Convenience function for Barrowman Aero methods
            Initialize ForceMomentSystem from aerodynamic coefficients Cd and CN
            Cd should NOT already be adjusted for angle of attack 
            Moment should be a dimensional moment vector
        '''
        angleOfAttack = AeroFunctions.getTotalAOA(rocketState, environment)
        q = AeroFunctions.getDynamicPressure(rocketState, environment)

        #### Axial Force ####
        CA = AeroFunctions.getDragToAxialForceFactor(angleOfAttack) * Cd
        axialForce = Vector(0, 0, -CA) #Axial force is in the negative Z-direction 

        #### Normal Force ####
        normalForce = AeroFunctions.getNormalAeroForceDirection(rocketState, environment) * CN

        totalForce = (axialForce + normalForce) * refArea * q
        return ForceMomentSystem(totalForce, CPLocation, moment)

    @classmethod
    def fromAllCoefficients(self, rocketState, environment, Cd, Cl, CMx, CMy, CMz, CPLocation, refArea, refLength):
        ''' Initialize ForceMomentSystem from all aerodynamic coefficients '''
        q = AeroFunctions.getDynamicPressure(rocketState, environment)
        if q == 0.0:
            # No force without dynamic pressure
            return ForceMomentSystem(Vector(0,0,0))
        nonDimConstant = q * refArea

        #### Drag ####
        localFrameAirVel = AeroFunctions.getLocalFrameAirVel(rocketState, environment)
        dragDirection = localFrameAirVel.normalize()
        dragForce = dragDirection * Cd

        #### Lift ####
        # Find the lift force direction
            # Lift force will be in the same plane as the normalForceDirection & rocketFrameAirVel vectors
            # But, the lift force will be perpendicular to rocketFraeAirVel, whereas the normalForceDirection might form an acute angle with it
            # Rotate the normal force direction vector until it's perpendicular with rocketFrameAirVel
        normalForceDir = AeroFunctions.getNormalAeroForceDirection(rocketState, environment)
        rotAxis = localFrameAirVel.crossProduct(normalForceDir)
        angle = math.pi/2 - localFrameAirVel.angle(normalForceDir)
        rotatorQuat = Quaternion(rotAxis, angle)
        liftDirection = rotatorQuat.rotate(normalForceDir)
        liftForce = liftDirection * Cl

        # Combine and redimensionalize
        totalForce = (dragForce + liftForce) * nonDimConstant

        #### Moments ####
        moments = Vector(CMx, CMy, CMz) * nonDimConstant * refLength

        return ForceMomentSystem(totalForce, CPLocation, moments)