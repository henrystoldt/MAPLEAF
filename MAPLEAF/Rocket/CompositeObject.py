'''
Both `MAPLEAF.Rocket.Stage` and `MAPLEAF.Rocket.Rocket` objects inherit from `CompositeObject`.
It implements functionality to add forces and inertias from an arbitrary number of subcomponents.
'''

from MAPLEAF.Motion import ForceMomentSystem, Inertia, Vector
from MAPLEAF.Utilities import cacheLastResult
from MAPLEAF.Rocket import FixedMass

__all__ = [ 'CompositeObject' ]

class CompositeObject():
    """
        Represents a collection of physical objects with masses.
        Expects:
            Components to have the following methods:
                .getAeroForce(state, time, environmentalConditions, rocketCG)
                .getInertia(time)
                .getMass(time) (optional as long as getInertia is implemented)
            Objects that inherit from FixedMass to be fixed mass objects. Their inertias are cached and not recomputed
            Objects that do not inherit from FixedMass to be variable mass objects. Their inertias are recalculated all the time.
    """
    def __init__(self, components=[], CGOverride=None, MOIOverride=None, massOverride=None):
        self.components = components

        self.CGOverride = CGOverride
        self.MOIOverride = MOIOverride
        self.massOverride = massOverride

        # Initialize lists of fixed and variable-mass components
        self.fixedMassComponents = []
        self.variableMassComponents = []
        self._initializeComponentLists()

        # Pre-calculate inertia of all fixed-mass components
        self.fixedMassInertiaComponent = None
        self.fixedMassMassComponent = None
        self._initializeFixedMassInertia()

        #TODO: Add ability to override aero-properties

    def _initializeComponentLists(self):
        # Sort component lists
        self.fixedMassComponents = []
        self.variableMassComponents = []

        for component in self.components:
            if isinstance(component, FixedMass):
                self.fixedMassComponents.append(component)
            else:
                self.variableMassComponents.append(component)

    def _initializeFixedMassInertia(self):
        if len(self.fixedMassComponents) > 0:
            inertiasList = []
            for fixedMassComponent in self.fixedMassComponents:
                inertiasList.append(fixedMassComponent.getInertia(0, None))

            self.fixedMassInertiaComponent = inertiasList[0].combineInertias(inertiasList)
            self.fixedMassMassComponent = self.fixedMassInertiaComponent.mass
        else:
            self.fixedMassInertiaComponent = Inertia(Vector(0,0,0), Vector(0,0,0), 0)
            self.fixedMassMassComponent = 0

    def recomputeFixedMassInertia(self):
        ''' Intended to be called if fixedMass components are added/removed or modified from the composite object '''

        self._initializeComponentLists()
        self._initializeFixedMassInertia()

        # Try to recompute inertias of sub-object, in case any of them are also CompositeObjects
            # This happens in the case of a Rocket, where the components are Stages (also CompositeObjects)
        for component in self.components:
            try:
                component.recomputeFixedMassInertia()
            except AttributeError:
                pass # Not a CompositeObject

    #TODO: Compute forces at interfaces between components
    
    @cacheLastResult
    def getAeroForce(self, state, time, environmentalConditions, rocketCG):
        '''
            Computes the aerodynamic force experienced by the stage.
            Does not include gravitational force - this is expected to be added at the rocket level.
        '''
        # Add up forces from all subcomponents
        totalAppliedComponentForce = ForceMomentSystem(Vector(0,0,0), rocketCG)
        for component in self.components:
            totalAppliedComponentForce += component.getAeroForce(state, time, environmentalConditions, rocketCG)

        return totalAppliedComponentForce

    @cacheLastResult
    def getInertia(self, time, state):
        ''' Returns an Inertia object, used for 6DoF computations '''            
        # Add up component inertias
        inertiasList = [ self.fixedMassInertiaComponent ]

        for component in self.variableMassComponents:
            inertiasList.append(component.getInertia(time, state))

        combinedInertia = inertiasList[0].combineInertias(inertiasList)

        # Overrides
        if self.CGOverride != None:
            combinedInertia.CG = self.CGOverride
            combinedInertia.MOICentroidLocation = self.CGOverride
        if self.MOIOverride != None:
            combinedInertia.MOI = self.MOIOverride
        if self.massOverride != None:
            combinedInertia.mass = self.massOverride

        return combinedInertia

    @cacheLastResult
    def getMass(self, time, state):
        ''' 
            Returns an inertia object where the CG and MOI members are zero.
            Only the mass is filled out correctly.
            Used for 3DoF computations 
        '''
        if self.massOverride == None:
            totalMass = self.fixedMassMassComponent
            for component in self.variableMassComponents:
                try:
                    totalMass += component.getMass(time, state).mass
                except AttributeError:
                    totalMass += component.getInertia(time, state).mass
        else:
            return self.massOverride

        return totalMass

    def getCG(self, time, state):
        return self.getInertia(time, state).CG
