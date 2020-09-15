from typing import Union

from MAPLEAF.Rocket.CompositeObject import CompositeObject
from MAPLEAF.Motion import Vector
from MAPLEAF.Rocket import (BodyComponent, PlanarInterface,
                            rocketComponentFactory, stringNameToClassMap)

__all__ = [ "Stage" ]

class Stage(CompositeObject, BodyComponent):
    ''' Represents a single rocket stage '''

    def __init__(self, stageDictReader, rocket):
        #TODO: This should take a boolean parameter 'dropped' which specified whether the stage being initialized is on the pad (full engine) or being dropped (empty engine)
        self.rocket = rocket
        self.stageDictReader = stageDictReader

        self.name = stageDictReader.getDictName()
        self.stageNumber = stageDictReader.getInt("stageNumber")
        self.position = stageDictReader.getVector("position")

        self.separationConditionType = stageDictReader.getString("separationTriggerType")
        self.separationDelay = stageDictReader.getFloat("separationDelay")
        self.separationConditionValue = stageDictReader.getFloat("separationTriggerValue")

        #### Placeholder fields, to be filled by subcomponents ####
        self.motor = None # Filled in by Motor.__init__
        self.engineShutOffTime = None # Filled in by Motor.__init__ - used for base drag calculations

        self.components = []
        self.initializeSubComponents()

        # Check if any stage-level inertia properties are being overriden
        CGOverride = stageDictReader.tryGetVector("constCG", defaultValue=None)
        if CGOverride != None:
            CGOverride += self.position
        massOverride = stageDictReader.tryGetFloat("constMass", defaultValue=None)
        MOIOverride = stageDictReader.tryGetVector("constMOI", defaultValue=None)

        # Initialize functionality inherited from CompositeObject - pass in any overrides
        CompositeObject.__init__(self, components=self.components, CGOverride=CGOverride, MOIOverride=MOIOverride, massOverride=massOverride)
        
    def initializeSubComponents(self):
        # Assume each sub dictionary represents a component
        subDicts = self.stageDictReader.getImmediateSubDicts()

        def returnZeroIfBodyComponent(componentDictPath):
            className = self.stageDictReader.getString(componentDictPath + ".class")
            componentClass = stringNameToClassMap[className]
            
            if issubclass(componentClass, BodyComponent):
                return 0, componentDictPath
            else: 
                return 1, componentDictPath

        # Make sure BodyComponents are initializede before others
            # This allows non-body components (which are attached to body components) like the fins
                # to get info about the body radius where they're mounted
        subDicts.sort(key=returnZeroIfBodyComponent)

        # Initialize each component, add to componets
        for componentDict in subDicts:
            newComponent = rocketComponentFactory(componentDict, self.rocket, self)
            self.components.append(newComponent)     

        # Create Planar Interfaces b/w components
        self.components = PlanarInterface.sortByZLocation(self.components)
        self.componentInterfaces = PlanarInterface.createPlanarComponentInterfaces(self.components)      

    def getComponentsOfType(self, type):
        matchingComponents = []
        for comp in self.components:
            if isinstance(comp, type):
                matchingComponents.append(comp)

        return matchingComponents

    #### Geometry / Introspection ####
    def _getFirstComponentOfType(self, componentList, desiredType):
        for comp in componentList:
            if isinstance(comp, desiredType):
                return comp
        return None

    def getTopInterfaceLocation(self) -> Union[Vector, None]:
        # Expects components to be sorted by z-location, top to bottom
        comp = self._getFirstComponentOfType(self.components, BodyComponent)
        try:
            topZ = comp.getTopInterfaceLocation().Z
            return Vector(self.position.X, self.position.Y, topZ)   
        except AttributeError:
            return None # Occurs when comp.getTopInterfaceLocation() == None

    def getBottomInterfaceLocation(self) -> Union[Vector, None]:
        # Expects components to be sorted by z-location, top to bottom
        comp = self._getFirstComponentOfType(reversed(self.components), BodyComponent)
        try:
            bottomZ = comp.getBottomInterfaceLocation().Z
            return Vector(self.position.X, self.position.Y, bottomZ)   
        except AttributeError:
            return None # Occurs when comp.getBottomInterfaceLocation() == None

    def getLength(self):
        try:
            topBodyComponent = self._getFirstComponentOfType(self.components, BodyComponent)
            topLocationZ = topBodyComponent.position.Z

            bottomBodyComponent = self._getFirstComponentOfType(reversed(self.components), BodyComponent)
            bottomLocationZ = bottomBodyComponent.position.Z - bottomBodyComponent.length
            return (topLocationZ - bottomLocationZ)
        except AttributeError:
            return None # No body components in current Stage

    def getMaxDiameter(self):
        maxDiameter = 0
        for comp in self.components:
            try:
                maxDiameter = max(comp.getMaxDiameter(), maxDiameter)
            except AttributeError:
                pass # Component is not a BodyComponent
        
        return maxDiameter

    def getRadius(self, distanceFromTop):
        topOfStageZ = self.position.Z
        desiredZ = topOfStageZ - distanceFromTop

        for comp in self.components:
            try:
                # Check if desired location is part of this component, if so, return the component's radius
                compTopZ = comp.position.Z
                compBottomZ = compTopZ - comp.length
                if desiredZ <= compTopZ and desiredZ >= compBottomZ:
                    distanceFromTopOfComponent = compTopZ - desiredZ
                    return comp.getRadius(distanceFromTopOfComponent)

            except AttributeError:
                pass # Not a body component
        
        raise ValueError("Length {} from the top of stage '{}' does not fall between the top and bottom of any of its components".format(distanceFromTop, self.name))

    def plotShape(self):
        '''
            Calls .plotShape() on all subcomponents
            Returns CGsubZ, CGsubY (two lists of scalars for a 2D plot)
        '''
        CGsubZ = []
        CGsubY = []
        for component in self.components:               
            try:
                component.plotShape()
                initialComponentInertia = component.getInertia(0, self.rocket.rigidBody.state)
                CGsubZ.append(initialComponentInertia.CG.Z)
                CGsubY.append(initialComponentInertia.CG.Y)
            
            except AttributeError:
                pass # Plotting function not implemented
        
        return CGsubZ, CGsubY
