import math

from . import AeroFunctions
from MAPLEAF.Motion import AeroParameters, ForceMomentSystem, Vector
from MAPLEAF.Rocket import BodyComponent, FixedMass
from MAPLEAF.Rocket.noseCone import (computeSubsonicPolyCoeffs,
                                     computeTransonicPolyCoeffs,
                                     getSupersonicPressureDragCoeff_Hoerner)
from MAPLEAF.Utilities import logForceResult

__all__ = [ "Transition", "BoatTail" ]

class Transition(FixedMass, BodyComponent):
    ''' Models a conical diameter transition (growing or shrinking) '''

    def __init__(self, *args):
        '''
            Two possible sets of inputs:  
            1. Initialization as a regular, dictionary-defined rocket component:  
                * args = (componentDictReader, rocket, stage)  
                * Expected classes: (`MAPLEAF.IO.SubDictReader`, `MAPLEAF.Rocket.Rocket`, `MAPLEAF.Rocket.Stage`)  
            2. Manual initialization:  
                * args = (startDiameter, endDiameter, length, position, inertia, rocket, stage, name, surfaceRoughness)  
                * Expected classes: (float, float, float, `MAPLEAF.Motion.Vector`, `MAPLEAF.Motion.Inertia`, `MAPLEAF.Rocket.Rocket`, `MAPLEAF.Rocket.Stage`, str, float)  
        '''
        if len(args) == 3:
            # Classic initialization from componentDictReader
            componentDictReader, rocket, stage = args
            FixedMass.__init__(self, componentDictReader, rocket, stage)

            self.length = componentDictReader.getFloat("length")
            self.startDiameter = componentDictReader.getFloat("startDiameter")
            self.endDiameter = componentDictReader.getFloat("endDiameter")
            self.surfaceRoughness = componentDictReader.tryGetFloat("surfaceRoughness", defaultValue=self.rocket.surfaceRoughness)
        else:
            # Initialization from parameters passed in
            self.startDiameter, self.endDiameter, self.length, self.position, self.inertia, self.rocket, self.stage, self.name, self.surfaceRoughness = args

        self._precomputeGeometry()

    def _precomputeGeometry(self):
        # Calculate basic areas
        self.topArea = self.startDiameter**2 * math.pi/4
        self.bottomArea = self.endDiameter**2 * math.pi/4
        self.frontalArea = self.topArea - self.bottomArea

        # Calculate surface area, volume, CP
        if self.length == 0:
            self.wettedArea = 0.0
            self.volume = 0.0
            self.CPLocation = self.position
        else:
            maxDiameter = max(self.startDiameter, self.endDiameter)
            minDiameter = min(self.startDiameter, self.endDiameter)

            hypotenuseSlope = ((maxDiameter - minDiameter) / self.length)
            # Boat tail is a truncated cone
            # Compute surface area of non-truncated cone starting at top of boattail
            baseConeHeight = maxDiameter / hypotenuseSlope
            baseHypotenuse = math.sqrt((maxDiameter/2)**2 + baseConeHeight**2)
            fullConeSurfaceArea = math.pi * (maxDiameter/2) * baseHypotenuse
            # Compute surface area of non-truncated cone starting at bottom of boattail
            tipConeHeight = minDiameter / hypotenuseSlope
            tipHypotenuse = math.sqrt((minDiameter/2)**2 + tipConeHeight**2)
            tipConeSurfaceArea = math.pi * (minDiameter/2) * tipHypotenuse
            # Surface area is the difference b/w the two
            self.wettedArea = fullConeSurfaceArea - tipConeSurfaceArea

            # Volume calculation uses same method as surface area calculation above
            fullConeVolume = self.topArea * baseConeHeight / 3
            tipConeVolume = self.bottomArea * tipConeHeight / 3
            self.volume = fullConeVolume - tipConeVolume

            # Calculate CP
            self.CPLocation = AeroFunctions.Barrowman_GetCPLocation(self.length, self.topArea, self.bottomArea, self.volume, self.position)

        # Calc aspect ratio
        if self.startDiameter == self.endDiameter:
            aspectRatio = 100 # Avoid division by zero
        else:
            aspectRatio = self.length / abs(self.startDiameter - self.endDiameter)

        # Precompute drag properties
        if self.endDiameter <= self.startDiameter:
            # Calculate Cd multiplier based on aspect ratio - Niskanen Eqn 3.88
            if aspectRatio < 1:
                self.CdAdjustmentFactor = 1
            elif aspectRatio < 3:
                self.CdAdjustmentFactor = (3 - aspectRatio) / 2
            else:
                self.CdAdjustmentFactor = 0
        else:
            if self.length == 0:
                coneHalfAngle = math.pi/2
            else:
                coneHalfAngle = math.atan(abs(self.startDiameter - self.endDiameter)/2 / self.length)
            self.SubsonicCdPolyCoeffs = computeSubsonicPolyCoeffs(coneHalfAngle)
            self.TransonicCdPolyCoeffs = computeTransonicPolyCoeffs(coneHalfAngle)

    def plotShape(self):
        import matplotlib.pyplot as plt
        Xvals = []
        Yvals = []

        forePos = self.position.Z
        aftPos = forePos - self.length
        foreRadius = self.startDiameter/2
        aftRadius = self.endDiameter/2

        Xvals.append(forePos)
        Yvals.append(foreRadius) # top right

        Xvals.append(aftPos)
        Yvals.append(aftRadius) # top left

        Xvals.append(aftPos)
        Yvals.append(-aftRadius) # bottom left

        Xvals.append(forePos)
        Yvals.append(-foreRadius) # bottom right

        Xvals.append(forePos)
        Yvals.append(foreRadius) # close in the shape
        plt.plot(Xvals, Yvals, color = 'k')

    @logForceResult
    def getAeroForce(self, rocketState, time, environment, CG) -> ForceMomentSystem:
        Mach = AeroParameters.getMachNumber(rocketState, environment)
        Aref = self.rocket.Aref
        
        #### Normal Force ####
        AOA = AeroParameters.getTotalAOA(rocketState, environment)
        CN = AeroFunctions.Barrowman_GetCN(AOA, Aref, self.topArea, self.bottomArea)

        #### Pressure Drag ####
        if self.startDiameter > self.endDiameter:
            # Pressure base drag
            Cd_base = AeroFunctions.getBaseDragCoefficient(Mach)
            Cd_pressure = Cd_base * self.CdAdjustmentFactor

        else:
            # Pressure drag calculated like a nose cone
            if Mach < 1:
                # Niskanen pg. 48 eq. 3.87 - Power law interpolation
                Cd_pressure = self.SubsonicCdPolyCoeffs[0] * Mach**self.SubsonicCdPolyCoeffs[1]

            elif Mach > 1 and Mach < 1.3:
                # Interpolate in transonic region - derived from Niskanen Appendix B, Eqns B.3 - B.6
                Cd_pressure = self.TransonicCdPolyCoeffs[0] + self.TransonicCdPolyCoeffs[1]*Mach +  \
                    self.TransonicCdPolyCoeffs[2]*Mach**2 + self.TransonicCdPolyCoeffs[3]*Mach**3
                
            else:
                Cd_pressure = getSupersonicPressureDragCoeff_Hoerner(self.coneHalfAngle, Mach)
        
        # Make reference are the rocket's, not this objects
        Cd_pressure *= self.frontalArea / Aref 

        #### Skin Friction Drag ####
        if self.wettedArea == 0:
            skinFrictionDragCoefficient = 0
            rollDampingMoment = Vector(0,0,0)
        else:
            skinFrictionDragCoefficient, rollDampingMoment = AeroFunctions.getCylindricalSkinFrictionDragCoefficientAndRollDampingMoment(rocketState, environment, \
                 self.length, Mach, self.surfaceRoughness, self.wettedArea, Aref, self.rocket.finenessRatio, self.rocket.fullyTurbulentBL)

        #### Total Drag ####
        Cd = Cd_pressure + skinFrictionDragCoefficient

        #### Assemble & return final force object ####
        return AeroFunctions.forceFromCdCN(rocketState, environment, Cd, CN, self.CPLocation, Aref, moment=rollDampingMoment)

    def getMaxDiameter(self):
        return max(self.startDiameter, self.endDiameter)

    def getRadius(self, distanceFromTip):
        return (distanceFromTip/self.length * (self.endDiameter - self.startDiameter) + self.startDiameter) / 2

class BoatTail(Transition):
    '''        
        Defines a conical boattail (aerodynamic properties quite similar to curved boattails, especially in supersonic flight)
        Always assumes it's at the bottom of a rocket.
        Modelled like a Transition object, but accounts for base drag.
    '''
    canConnectToComponentBelow = False 
    ''' Overrides attribute inherited from BodyComponent (through Transition), to indicate that this component must exist at the very bottom of a rocket '''

    @logForceResult
    def getAeroForce(self, rocketState, time, environment, CG) -> ForceMomentSystem:
        Mach = AeroParameters.getMachNumber(rocketState, environment)
        Aref = self.rocket.Aref
        
        #### Normal Force ####
        AOA = AeroParameters.getTotalAOA(rocketState, environment)
        CN = AeroFunctions.Barrowman_GetCN(AOA, Aref, self.topArea, self.bottomArea)

        #### Pressure Drag ####
        Cd_base = AeroFunctions.getBaseDragCoefficient(Mach)
        Cd_pressure = Cd_base * self.CdAdjustmentFactor
        Cd_pressure *= self.frontalArea / self.rocket.Aref

        noEngine = (self.stage.engineShutOffTime == None)
        if noEngine or time > self.stage.engineShutOffTime:
            # Add base drag if engine is off
            Cd_pressure += Cd_base * self.bottomArea / Aref

        #### Skin Friction Drag ####
        if self.wettedArea == 0:
            skinFrictionDragCoefficient = 0
            rollDampingMoment = Vector(0,0,0)
        else:
            skinFrictionDragCoefficient, rollDampingMoment = AeroFunctions.getCylindricalSkinFrictionDragCoefficientAndRollDampingMoment(rocketState, environment, \
                 self.length, Mach, self.surfaceRoughness, self.wettedArea, Aref, self.rocket.finenessRatio, self.rocket.fullyTurbulentBL)

        #### Total Drag ####
        Cd = Cd_pressure + skinFrictionDragCoefficient

        #### Assemble & return final force object ####
        return AeroFunctions.forceFromCdCN(rocketState, environment, Cd, CN, self.CPLocation, Aref, moment=rollDampingMoment)
