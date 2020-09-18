'''
Functions to calculate parameters relevant to aerodynamic calculations - 
    Mach/Reynolds numbers, AOA, local frame air velocity etc...

Used throughout the aeordynamics functions
'''

#TODO: This file needs to be split up neatly

import math

from MAPLEAF.Motion import (AeroParameters, ForceMomentSystem, Quaternion,
                            Vector)
from MAPLEAF.Utilities import cacheLastResult


def _getCPZ(componentForces) -> int:
    normalForce = Vector(componentForces.force.X, componentForces.force.Y, 0)
    normalForceMagnitude = normalForce.length()

    if normalForceMagnitude == 0.0:
        # Avoid division by zero, just return CG location
        return componentForces.location.Z

    normalForceDirection = normalForce.normalize()

    momentAxis = normalForceDirection.crossProduct(Vector(0,0,1))
    momentComponent = componentForces.moment * momentAxis

    return componentForces.location.Z - momentComponent/normalForceMagnitude


#### Constants ####
def getGamma():
    return 1.4

def getGasConstant():
    return 0.287



#### Skin Friction ####
def _subSonicSkinFrictionCompressiblityCorrectionFactor(Mach, smoothSurface):
    '''
        Inputs:
            *skinFrictionCoefficient: (float) - skin friction coefficient that has not been adjusted for compressiblity
            *MachNumber: (float)
            *smooth: (boolean) - True if smooth wall, false if rough wall
    '''
    # Subsonic correlation: Eqns 4-9, 4-10 (Barrowman)
    if smoothSurface:
        return 1 - 0.10 * Mach**2
    else:
        return 1 - 0.12 * Mach**2

def _supersonicSkinFrictionCompressiblityCorrectionFactor(Mach, smoothSurface):
    '''
    Inputs:
        *skinFrictionCoefficient: (float) - skin friction coefficient that has not been adjusted for compressiblity
        *MachNumber: (float)
        *smooth: (boolean) - True if smooth wall, false if rough wall
    '''
    # Eqns 4-12, 4-13 (Barrowman)
    if smoothSurface:
        # Barrowman 4-12, for an adiabatic wall. The 0.15 coefficient would be 0.0512 for a cooled wall
        return  1 / ((1 + 0.15*Mach**2)**(0.58))
    else:
        smoothTurbFactor =  1 / ((1 + 0.15*Mach**2)**0.58)
        roughTurbFactor =   1 /  (1 + 0.18*Mach**2) #TODO: Seems like this one is always smaller than smoothTurbFactor??? - Perhaps there's a better eqn out there
        return max(smoothTurbFactor, roughTurbFactor)

@cacheLastResult
def getSkinFrictionCoefficient(state, environment, length, Mach, surfaceRoughness, assumeFullyTurbulent=True):
    ''' 
        Calculates the skin friction drag coefficient (normalized by wetted area, not cross-sectional area).  
        Uses the methods described in (Barrowman, 1967, pg. 43-48) and in (Niskanen, 2009, pg. 43-46) 

        Used for skin friction calculations for all external rocket components

        Inputs:
          * state: (RigidBodyState)
          * environment: (EnvironmentalConditions)
          * length: (float) - reference length for the current rocket component, used for Reynolds number calculation
          * Mach: (float) - current Mach number
          * surfaceRoughness (float) - micrometers

        Outputs:
          * Returns the skin friction coefficient normalized by **wetted area**, must be converted to be normalized by cross-sectional area before summing with other coefficients.
    '''
    Re = AeroParameters.getReynoldsNumber(state, environment, length)
    smoothSurface = (surfaceRoughness == 0) # True or False

    if smoothSurface:
        # Very large (unreachable) number
        criticalRe = 1e100
    else:
        criticalRe = 51*(surfaceRoughness / length)**(-1.039) # Eqn 4-7 (Barrowman)

    # Get skin friction factor (Mach number independent)
    if Re > criticalRe:
        # Re is above Critical value
        # Friction determined by roughness
        skinFrictionCoefficient = 0.032 * (surfaceRoughness / length)**0.2
    elif assumeFullyTurbulent:
        if Re < 1e4:
            # Limit for low Reynolds numbers
            skinFrictionCoefficient = 0.0148 # Eqn 3.81 (Niskanen) - match up with turbulent flow correlation
        else:
            # Eqn from Niskanen
            skinFrictionCoefficient = 1 / ((1.5 * math.log(Re) - 5.6)**2)
    else:
        # Laminar / Transitional flow
        if Re < 1e4:
            # Limit for low Reynolds numbers
            skinFrictionCoefficient = 0.01328 # Match up with transitional flow eqn
        elif Re < 5e5:
            # Laminar flow
            skinFrictionCoefficient = 1.328 / math.sqrt(Re) # Eqn 4-2 (Barrowman)
        else:
            # Transitional Flow
            skinFrictionCoefficient = 1 / ((1.5 * math.log(Re) - 5.6)**2) - 1700/Re # Eqn 4-6 (Barrowman), subtraction term from from Prandtl
    
    # Calculate compressibility correction
    if Mach < 0.9: 
        # Subsonic
        compressibilityCorrectionFactor = _subSonicSkinFrictionCompressiblityCorrectionFactor(Mach, smoothSurface)
    elif Mach < 1.1:
        # Transonic
        subsonicFactor = _subSonicSkinFrictionCompressiblityCorrectionFactor(Mach, smoothSurface)
        supersonicFactor = _supersonicSkinFrictionCompressiblityCorrectionFactor(Mach, smoothSurface)
        # Linearly interpolate in the Mach 0.9-1.1 region - same method as that used in OpenRocket v15.03
        compressibilityCorrectionFactor = subsonicFactor + ((supersonicFactor - subsonicFactor)*(Mach - 0.9) / 0.2)
    else:
        # Supersonic
        compressibilityCorrectionFactor = _supersonicSkinFrictionCompressiblityCorrectionFactor(Mach, smoothSurface)

    # Return corrected factor
    return skinFrictionCoefficient*compressibilityCorrectionFactor

def getCylindricalSkinFrictionDragCoefficientAndRollDampingMoment(state, environment, length, Mach, surfaceRoughness, wettedArea, Aref, finenessRatio, assumeFullyTurbulent=True):
    ''' Equations from Barrowman section 4.0, Niskanen section 3.4 '''
    # Get a compressiblity-corrected flat plate skin friction drag coefficient, normalized by wetted area
    skinFrictionDragCoefficient = getSkinFrictionCoefficient(state, environment, length, Mach, surfaceRoughness, assumeFullyTurbulent)
    # Account for the fact that the rocket is cylindrical, and not a flat plate (Barrowman Eqn 4-16)
    skinFrictionDragCoefficient *= (1 + (0.5 / finenessRatio))
    # Rebase coefficient to the reference area
    skinFrictionDragCoefficient *= (wettedArea / Aref)

    # Approximate avg radius as the radius of a cylinder which would have the same wetted area + length 
    avgRadius = (wettedArea / length) / (2*math.pi)
    rollingSurfaceVel = avgRadius * state.angularVelocity.Z
    # Assume velocities are perpendicular
    airVel = AeroParameters.getLocalFrameAirVel(state, environment).length()
    # Use total velocity to redimensionalize coefficient
    totalVelSquared = airVel**2 + rollingSurfaceVel**2

    if totalVelSquared == 0:
        return skinFrictionDragCoefficient, Vector(0,0,0)
    else:
        totalSurfaceForce = skinFrictionDragCoefficient * 0.5 * totalVelSquared * environment.Density * Aref
        # Calculate roll damping component of total friction force using similar triangles
        rollDampingForce = totalSurfaceForce * (rollingSurfaceVel/math.sqrt(totalVelSquared))
        # Calculate resulting moment
        rollDampingMoment = Vector(0,0, -rollDampingForce*avgRadius)
    
    return skinFrictionDragCoefficient, rollDampingMoment



#### Leading edge and base drag ####
@cacheLastResult
def getBaseDragCoefficient(Mach):
    if Mach < 1:
        return 0.12 + 0.13*Mach*Mach
    else:
        return 0.25/Mach

@cacheLastResult
def getCylinderCrossFlowCd_ZeroBaseDrag(Mach):
    ''' 
        Calculate drag coefficient of a cylinder, omitting base drag 
        Method from Barrowman Eqns 4-17 to 4-19
        Similar method in Niskanen Eqn. 3.89
    '''
    if Mach < 0.9:
        return (1 - Mach*Mach)**-0.417 - 1
    elif Mach <= 1:
        return 1 - 1.5*(Mach - 0.9)
    else:
        return 1.214 - 0.502/Mach**2 + 0.1095/Mach**4 + .0231/Mach**6

@cacheLastResult
def getBluntBodyCd_ZeroBaseDrag(Mach):
    # Originally from Hoerner (pg. 15-3), same as Niskanen Eqns (In Appendix) B.1 and B.2
    if Mach < 1:
        qStageOverQ = 1 + (Mach**2/4) + (Mach**4/40)
    else:
        qStageOverQ = 1.84 - (0.76/(Mach**2)) + (0.166/(Mach**4)) + (0.035/(Mach**6))

    return 0.85 * qStageOverQ



#### Other ####
@cacheLastResult
def getDragToAxialForceFactor(AOA):
    ''' Expected input is radians angle of attack - converts dragCoefficient to axialForceCoefficient: openRocket documentation pg. 53 '''
    # Angle of attack will not be negative if obtained from getTotalAOA in AeroParameters
        # But can be negative when fins are calculating their angles of attack, hence the abs()
    AOA = math.degrees(abs(AOA))

    # Derived to match Niskanen pg. 53
    if AOA <= 17:
        return -0.000122124*AOA**3 + 0.003114164*AOA**2 + 1
    else:
        return 0.000006684*AOA**3 - 0.0010727204*AOA**2 + 0.030677323*AOA + 1.055660807


#### Creating ForceMomentSystems from Coefficients ####
def forceFromCdCN(rocketState, environment, Cd, CN, CPLocation, refArea, moment=None):
    ''' 
        Convenience function for Barrowman Aero methods
        Initialize ForceMomentSystem from aerodynamic coefficients Cd and CN
        Cd should NOT already be adjusted for angle of attack 
        Moment should be a dimensional moment vector
    '''
    angleOfAttack = AeroParameters.getTotalAOA(rocketState, environment)
    q = AeroParameters.getDynamicPressure(rocketState, environment)

    #### Axial Force ####
    CA = getDragToAxialForceFactor(angleOfAttack) * Cd
    axialForce = Vector(0, 0, -CA) #Axial force is in the negative Z-direction 

    #### Normal Force ####
    normalForce = AeroParameters.getNormalAeroForceDirection(rocketState, environment) * CN

    totalForce = (axialForce + normalForce) * refArea * q
    return ForceMomentSystem(totalForce, CPLocation, moment)

def forceFromCoefficients(rocketState, environment, Cd, Cl, CMx, CMy, CMz, CPLocation, refArea, refLength):
    ''' Initialize ForceMomentSystem from all aerodynamic coefficients '''
    q = AeroParameters.getDynamicPressure(rocketState, environment)
    if q == 0.0:
        # No force without dynamic pressure
        return ForceMomentSystem(Vector(0,0,0))
    nonDimConstant = q * refArea

    #### Drag ####
    localFrameAirVel = AeroParameters.getLocalFrameAirVel(rocketState, environment)
    dragDirection = localFrameAirVel.normalize()
    dragForce = dragDirection * Cd

    #### Lift ####
    # Find the lift force direction
        # Lift force will be in the same plane as the normalForceDirection & rocketFrameAirVel vectors
        # But, the lift force will be perpendicular to rocketFraeAirVel, whereas the normalForceDirection might form an acute angle with it
        # Rotate the normal force direction vector until it's perpendicular with rocketFrameAirVel
    normalForceDir = AeroParameters.getNormalAeroForceDirection(rocketState, environment)
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


#### Barrowman ####
def Barrowman_GetCPLocation(length, crossSectionalArea_top, crossSectionalArea_bottom, volume, topLocation=Vector(0,0,0)):
    ''' Get Cp location for an axisymmetric body component - Niskanen Eqn 3.28, expanded from Barrowman Eqn 3-88 '''
    if crossSectionalArea_top == crossSectionalArea_bottom:
        # Body tubes
        CpDistanceFromTop = length / 2
    else:
        # Nosecones and boat tails / transitions
        CpDistanceFromTop = (length * crossSectionalArea_bottom - volume) / (crossSectionalArea_bottom - crossSectionalArea_top)
    
    return Vector(topLocation.X, topLocation.Y, topLocation.Z - CpDistanceFromTop)  # For body tubes

def Barrowman_GetCN(AOA, Aref, crossSectionalArea_top, crossSectionalArea_bottom):
    ''' 
        Get CN for an axisymmetric body component - Niskanen Eqn 3.18, expanded from Barrowman Eqn 3-63 
        AOA expected in radians
    '''
    return 2 * math.sin(AOA) * (crossSectionalArea_bottom - crossSectionalArea_top) / Aref
