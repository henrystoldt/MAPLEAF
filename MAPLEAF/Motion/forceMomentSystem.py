from MAPLEAF.Motion import Vector

__all__ = [ 'ForceMomentSystem' ]

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

    def __str__(self):
        ''' Return string representation of object (Used by print()) '''
        return 'Force=({}) At=({}) + Moment=({})'.format(self.force, self.location, self.moment)

    def __eq__(self, force2):
        return self.force == force2.force and self.location == force2.location and self.moment == force2.moment
