    # -*- coding: utf-8 -*-
"""
Contains interface definitions (`RocketComponent`/`BodyComponent`) and Base classes for all RocketComponents (`FixedMass`),  
as well as some simple rocket component classes (`FixedForce`,`AeroForce`,`AeroDamping` etc.)

"""

from abc import ABC, abstractmethod
from typing import List, Union

import numpy as np
from scipy.interpolate import LinearNDInterpolator

from MAPLEAF.Motion import (AeroParameters, ForceMomentSystem, Inertia,
                            RigidBodyState, Vector, linInterp)
from . import AeroFunctions
from MAPLEAF.Utilities import logForceResult

__all__ = [ "RocketComponent", "BodyComponent", "PlanarInterface", "FixedMass", "FixedForce", "AeroForce", "AeroDamping", "TabulatedAeroForce", "TabulatedInertia", "FractionalJetDamping" ]

class RocketComponent(ABC):
    ''' Interface definition for rocket components '''
    @abstractmethod
    def __init__(self, componentDictReader, rocket, stage):
        return

    @abstractmethod
    def getInertia(self, time, state) -> Inertia:
        return

    @abstractmethod
    def getAeroForce(self, rocketState, time, environmentalConditions, rocketCG) -> ForceMomentSystem:
        return

class BodyComponent(ABC):
    ''' 
        Class that defines interface for axisymmetric body components.
        Contains logic for detecting adjacent body components & defining interfaces with them 
        Examples: `MAPLEAF.Rocket.NoseCone`, `MAPLEAF.Rocket.Stage`
    '''
    # Override these attributes in child classes to change whether they can connect to components above/below them
    canConnectToComponentAbove = True
    canConnectToComponentBelow = True

    def getTopInterfaceLocation(self) -> Union[None, Vector]:
        ''' For planar cylindrical interfaces, returns the location of the center of the cylindrical interface '''
        if self.canConnectToComponentAbove:
            return self.position
        else:
            return None

    def getBottomInterfaceLocation(self) -> Union[None, Vector]:
        ''' For planar cylindrical interfaces, returns the location of the center of the cylindrical interface '''
        if self.canConnectToComponentBelow:
            baseZCoord = self.position.Z-self.length
            return Vector(self.position.X, self.position.Y, baseZCoord)
        else:
            return None

    def getLogHeader(self):
        return " {}FX(N) {}FY(N) {}FZ(N) {}MX(Nm) {}MY(Nm) {}MZ(Nm)".format(*[self.name]*6)

    def _getCenterOfPressure(self, *args) -> Vector:
        return self.CPLocation

    @abstractmethod
    def getMaxDiameter(self):
        ''' These functions used for determining rocket's current max diameter '''
        return

    @abstractmethod
    def getRadius(self, distanceFromTop: float) -> float:
        ''' Should return body component radius as a function of distance from the top of the component '''
        return

class PlanarInterface():
    def __init__(self, location: Vector, component1: RocketComponent, component2: RocketComponent, planeNormal=Vector(0,0,-1)):
        ''' 
            Defines a planar interface between two components  
            In the local frame, the normalVector is expected to point across the interface from component1 to component2   
            In a rocket, this means that (with the default normalVector pointing in the -'ve Z direction (towards the tail)), component1 is above component2   
        '''
        self.location = location
        self.component1 = component1
        self.component2 = component2
        self.normalVector = planeNormal

    @classmethod
    def createPlanarComponentInterfaces(cls, components):
        '''
            Expects components in the list to be sorted by z location (top to bottom)
            Tries to construct PlanarInterface objects connecting all of the BodyComponent object from top to bottom
            Returns a list of PlanarInterface objects, ordered from top to bottom
        '''
        return None
        # Ignore components that aren't of type 'BodyComponent'
        bodyComponents = []
        for comp in components:
            if isinstance(comp, BodyComponent):
                bodyComponents.append(comp)
        
        # Construct interfaces between components
        componentInterfaces = []
        interfaceLocationTolerance = 0.001 # m
        
        for i in range(len(bodyComponents)-1):
            topComponent = bodyComponents[i]
            bottomComponent = bodyComponents[i+1]
            topInterfaceLoc = topComponent.getBottomInterfaceLocation()
            bottomInterfaceLoc = bottomComponent.getTopInterfaceLocation()

            if (topInterfaceLoc - bottomInterfaceLoc).length() < interfaceLocationTolerance:
                interfaceLocation = (topInterfaceLoc + bottomInterfaceLoc) / 2 # Average location is where the interface will be
                componentInterfaces.append(PlanarInterface(interfaceLocation, topComponent, bottomComponent))
            else:
                raise ValueError("Body Component Location mismatch {} ({}) bottom interface at {} vs {} ({}) top interface at {}. Current interface tolerance = 0.001m".format(\
                    topComponent.name, type(topComponent), topInterfaceLoc, bottomComponent.name, type(bottomComponent), bottomInterfaceLoc))

        return componentInterfaces

    @classmethod
    def sortByZLocation(cls, components) -> List[RocketComponent]:
        ''' 
            Sort the components in order from top to bottom, component.position.Z
            This function could be relocated somewhere more suitable, at the time of writing, it is only being used to order components before creating interfaces b/w them
        '''
        def getZPosition(component):
            try:
                return component.position.Z
            except AttributeError:
                zeroState = RigidBodyState()
                return component.getInertia(0, zeroState).CG.Z

        components.sort(key=getZPosition, reverse=True)
        return components

class FixedMass(RocketComponent):
    '''
        Base class for all fixed-mass rocket components
        Implements functionality to read/store inertia and position info from sim definition file
    '''
    def __init__(self, componentDictReader, rocket, stage):
        self.rocket = rocket
        self.stage = stage
        self.componentDictReader = componentDictReader
        self.name = componentDictReader.getDictName()
            
        mass = componentDictReader.getFloat("mass")

        # Position in simulation definition is relative to stage position
        self.position = componentDictReader.getVector("position") + stage.position # Store position relative to nosecone here
        # CG in simulation definition is relative to component position
        cg = componentDictReader.getVector("cg") + self.position  # Store cg location relative to nosecone here

        try:
            MOI = componentDictReader.getVector("MOI")
        except:
            MOI = Vector(mass*0.01, mass*0.01, mass*0.01) # Avoid having zero moments of inertia

        self.inertia = Inertia(MOI, cg, mass)
        self.zeroForce = ForceMomentSystem(Vector(0,0,0))

    def getInertia(self, time, state):
        return self.inertia

    def getMass(self, time):
        return self.inertia.mass

    def getCG(self, time):
        return self.inertia.CG

    def getAeroForce(self, rocketState, time, environment, CG):
        return self.zeroForce

class FixedForce(RocketComponent):
    def __init__(self, componentDictReader, rocket, stage):
        ''' A Zero-inertia component that applies a constant ForceMomentSystem to the rocket '''
        self.componentDictReader = componentDictReader
        self.rocket = rocket
        self.stage = stage
        self.name = componentDictReader.getDictName()

        # Object is just a force, inertia is zero
        self.inertia = Inertia(Vector(0,0,0), Vector(0,0,0), 0)

        force = componentDictReader.getVector("force")
        forceLocation = componentDictReader.getVector("position")
        moment = componentDictReader.getVector("moment")

        self.force = ForceMomentSystem(force, forceLocation, moment)
        
    def getInertia(self, time, state):
        return self.inertia

    @logForceResult
    def getAeroForce(self, rocketState, time, environment, rocketCG):
        return self.force

    def getLogHeader(self):
        return " {}FX(N) {}FY(N) {}FZ(N) {}MX(Nm) {}MY(Nm) {}MZ(Nm)".format(*[self.name]*6)

class AeroForce(RocketComponent):
    ''' A zero-Inertia component with constant aerodynamic coefficients '''
    # Object is just a force, inertia is zero
    inertia = Inertia(Vector(0,0,0), Vector(0,0,0), 0)

    def __init__(self, componentDictReader, rocket, stage):
        self.componentDictReader = componentDictReader
        self.rocket = rocket
        self.stage = stage
        self.name = componentDictReader.getDictName()

        self.position = componentDictReader.getVector("position")
        self.Aref = componentDictReader.getFloat("Aref")
        self.Lref = componentDictReader.getFloat("Lref")

        Cd = componentDictReader.getFloat("Cd")
        Cl = componentDictReader.getFloat("Cl")
        momentCoeffs = componentDictReader.getVector("momentCoeffs")

        self.aeroCoeffs = [ Cd, Cl, *momentCoeffs ]

    def getInertia(self, time, state):
        return self.inertia

    @logForceResult
    def getAeroForce(self, state, time, environment, rocketCG):
        return AeroFunctions.forceFromCoefficients(state, environment, *self.aeroCoeffs, self.position, self.Aref, self.Lref)

    def getLogHeader(self):
        return " {}FX(N) {}FY(N) {}FZ(N) {}MX(Nm) {}MY(Nm) {}MZ(Nm)".format(*[self.name]*6)

class AeroDamping(AeroForce):
    ''' A zero-inertia component with constant aerodynamic damping coefficients '''

    position = Vector(0,0,0)

    def __init__(self, componentDictReader, rocket, stage):
        self.componentDictReader = componentDictReader
        self.rocket = rocket
        self.stage = stage
        self.name = componentDictReader.getDictName()

        self.Aref = componentDictReader.getFloat("Aref")
        self.Lref = componentDictReader.getFloat("Lref")

        self.zDampingCoeffs = componentDictReader.getVector("zDampingCoeffs")
        self.yDampingCoeffs = componentDictReader.getVector("yDampingCoeffs")
        self.xDampingCoeffs = componentDictReader.getVector("xDampingCoeffs")
    
    @logForceResult
    def getAeroForce(self, state, time, environment, rocketCG):
        airspeed = max(AeroParameters.getLocalFrameAirVel(state, environment).length(), 0.0000001)
        redimConst = self.Lref / (2*airspeed)
        # Calculate moment coefficients from damping coefficients
        localFrameAngularVelocity = Vector(*state.angularVelocity)
        zMomentCoeff = self.zDampingCoeffs * localFrameAngularVelocity * redimConst
        yMomentCoeff = self.yDampingCoeffs * localFrameAngularVelocity * redimConst
        xMomentCoeff = self.xDampingCoeffs * localFrameAngularVelocity * redimConst
        momentCoeffs = [ xMomentCoeff, yMomentCoeff, zMomentCoeff ]

        return AeroFunctions.forceFromCoefficients(state, environment, 0, 0, *momentCoeffs, self.position, self.Aref, self.Lref)

class TabulatedAeroForce(AeroForce):
    ''' A zero-inertia component with aerodynamic coefficients that are tabulated according to one or more parameters (ex. AOA) '''

    def __init__(self, componentDictReader, rocket, stage):
        self.componentDictReader = componentDictReader
        self.rocket = rocket
        self.stage = stage
        self.name = componentDictReader.getDictName()

        self.position = componentDictReader.getVector("position")
        self.Aref = componentDictReader.getFloat("Aref")
        self.Lref = componentDictReader.getFloat("Lref")

        coefficientTableFilePath = componentDictReader.getString("filePath")
        self._loadCoefficients(coefficientTableFilePath)

    def _loadCoefficients(self, filePath):
        # Load first row to figure out what the columns mean
        with open(filePath) as f:
            columnNames = f.readline().strip().split(',')

        # Get functions that calculate the parameters used for interpolation
        # All these 'key'/parameter columns are expected to come before 'value' columns to be interpolated over
        self.parameterFunctions = []
        i = 0
        while i < len(columnNames):
            col = columnNames[i]
            if col in AeroParameters.stringToAeroFunctionMap:
                self.parameterFunctions.append(AeroParameters.stringToAeroFunctionMap[col])
            else:
                break
            i += 1

        # Continue parsing column names - aero coefficient names now            
        # This is the ordering expected by AeroFunctions.forceFromCoefficients
        aeroCoeffStrings = [ "CD", "CL", "CMx", "CMy", "CMz" ]
        self.aeroCoeffIndices = [] # Provides mapping between value column position in interpolation table & position in output aero coefficient list (ordered like aeroCoeffStrings above)
        while i < len(columnNames):
            coeff = columnNames[i]

            if coeff in aeroCoeffStrings:
                self.aeroCoeffIndices.append(aeroCoeffStrings.index(coeff))
                
            else:
                raise ValueError("ERROR: One of the following columns: {} did not match any of the expected columns names: Keys: {}, values: {}. \
                    Or was in the wrong order. All key columns must come BEFORE value columns.".format(columnNames, AeroParameters.stringToAeroFunctionMap.keys(), aeroCoeffStrings))
            i += 1

        # Load the data table to be interpolated
        dataTable = np.loadtxt(filePath, delimiter=',', skiprows=1)

        nKeyCols = len(self.parameterFunctions)
        keys = dataTable[:, 0:nKeyCols]
        aeroCoefficients = dataTable[:, nKeyCols:]

        if nKeyCols > 1:
            # Create n-dimensional interpolation function for aero coefficients
            self._interpAeroCoefficients = LinearNDInterpolator(keys, aeroCoefficients)
        else:
            # Save to use with MAPLEAF.Motion.linInterp
            self.keys = [ key[0] for key in keys ]
            self.values = aeroCoefficients

    def _getAeroCoefficients(self, state, environment):
        keys = AeroParameters.getAeroPropertiesList(self.parameterFunctions, state, environment)

        if len(keys) > 1:
            # Multi-dimensional linear interpolation
            interpolatedCoefficients = self._interpAeroCoefficients(keys)[0]
        else:
            # 1D linear interpolation
            interpolatedCoefficients = linInterp(self.keys, self.values, keys[0])

        aeroCoefficients = [0.0] * 5
        for i in range(len(interpolatedCoefficients)):
            indexInCoeffArray = self.aeroCoeffIndices[i]
            aeroCoefficients[indexInCoeffArray] = interpolatedCoefficients[i]

        return aeroCoefficients

    @logForceResult
    def getAeroForce(self, state, time, environment, rocketCG):
        aeroCoefficients = self._getAeroCoefficients(state, environment)
        return AeroFunctions.forceFromCoefficients(state, environment, *aeroCoefficients, self.position, self.Aref, self.Lref)

    def getLogHeader(self):
        return " {}FX(N) {}FY(N) {}FZ(N) {}MX(Nm) {}MY(Nm) {}MZ(Nm)".format(*[self.name]*6)

class TabulatedInertia(RocketComponent):
    ''' A zero-force component with time-varying tabulated inertia '''
    def __init__(self, componentDictReader, rocket, stage):
        self.rocket = rocket
        self.stage = stage
        self.componentDictReader = componentDictReader
        self.name = componentDictReader.getDictName()

        self.zeroForce = ForceMomentSystem(Vector(0,0,0))

        inertiaTableFilePath = componentDictReader.getString("filePath")
        self._parseInertiaTable(inertiaTableFilePath)

    def _parseInertiaTable(self, filePath):
        data = np.loadtxt(filePath, skiprows=1, delimiter=',')
        self.times = data[:, 0]
        self.inertiaData = data[:, 1:]

        # Check that the right number of columns is present
        if data.shape[1] != 8:
            raise ValueError("Wrong number of columns in inertia table: {}. Expecting 8 columns: \
                Time, Mass, CGx, CGy, CGz, MOIx, MOIy, MOIz")

    def getInertia(self, time, state):
        inertiaData = linInterp(self.times, self.inertiaData, time)
        # MOI is last three columns, CG is the three before that, and mass is column 0
        return Inertia(Vector(*inertiaData[-3:]), Vector(*inertiaData[1:4]), inertiaData[0])
    
    def getAeroForce(self, rocketState, time, environment, CG):
        return self.zeroForce

class FractionalJetDamping(RocketComponent):
    ''' A component to model Jet damping as per NASA's Two Stage to Orbit verification case '''

    # Object is just a force, inertia is zero
    inertia = Inertia(Vector(0,0,0), Vector(0,0,0), 0)

    def __init__(self, componentDictReader, rocket, stage):
        self.rocket = rocket
        self.stage = stage
        self.componentDictReader = componentDictReader
        self.name = componentDictReader.getDictName()
        
        self.dampingFraction = componentDictReader.getFloat("fraction")

    @logForceResult
    def getAeroForce(self, rocketState, time, environmentalConditions, rocketCG):
        # Only apply damping force if current stage's engine is firing
            # (Other stage's motors will have different exit planes)
        if time > self.stage.motor.ignitionTime and time < self.stage.engineShutOffTime:
            currentRocketInertia = self.rocket.getInertia(time, rocketState)
            
            # Differentiate rate of MOI change
            dt = 0.001
            nextRocketInertia = self.rocket.getInertia(time+dt, rocketState)            
            MOIChangeRate = (currentRocketInertia.MOI.X - nextRocketInertia.MOI.X) / dt

            dampingFactor = MOIChangeRate * self.dampingFraction
            
            angVel = rocketState.angularVelocity
            dampingMoment = Vector(-angVel.X*dampingFactor, -angVel.Y*dampingFactor, 0)

            return ForceMomentSystem(Vector(0,0,0), moment=dampingMoment)
        else:
            return ForceMomentSystem(Vector(0,0,0))

    def getInertia(self, time, state):
        return self.inertia

    def getLogHeader(self):
        return " {}FX(N) {}FY(N) {}FZ(N) {}MX(Nm) {}MY(Nm) {}MZ(Nm)".format(*[self.name]*6)
