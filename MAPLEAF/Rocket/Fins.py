import copy
import math
from collections import namedtuple
from math import cos, radians, sqrt

import numpy as np

from . import AeroFunctions
from MAPLEAF.GNC import ActuatedSystem
from MAPLEAF.Motion import (AeroParameters, AngularVelocity, ForceMomentSystem, Interpolation,
                            Quaternion, Vector)
from MAPLEAF.Rocket import FixedMass
from MAPLEAF.Rocket.CythonFinFunctions import (getFinSliceAngleOfAttack,
                                               getSubsonicFinNormalForce,
                                               getSupersonicFinNormalForce)
from MAPLEAF.Utilities import cacheLastResult

__all__ = [ "FinSet", "Fin" ]

# This named tuple is used to pass data between a FinSet and its child Fins.
    # All the data in this tuple is used by all Fins in a FinSet to complete an aero calculation.
    # To avoid recomputing it over and over, the FinSet computes it all once, then passes the info to each of its fins.
PreComputedFinAeroData = namedtuple("PreComputedFinAeroData", [ "finVelWithoutRoll", "CPXPos", "totalDragCoefficient" ])

@cacheLastResult
def getFinCnAlpha_Subsonic_Barrowman(span, planformArea, beta, midChordSweep):
    ''' Implements Barrowman's equation for Fin CN_alpha (3-6), based on the work of Diederich and Gothert '''
    # Version from Niskanen Eqn 3.37
    AR = 2*span**2 / planformArea

    # Barrowman Eqn 3-6
    numer = 2 * math.pi * AR
    denom = 2 + math.sqrt(4 + (beta*AR/cos(midChordSweep))**2)
    return numer/denom

@cacheLastResult
def getBusemannCoefficients(Mach, beta, gamma=1.4):
    K1 = 2/beta #Supersonic correlations, Niskanen Eq. 3.41-3.44, Barrowman Appendix A (2a-2c)
    K2 = ((gamma + 1)*Mach**4 - 4*beta**2) / (4*beta**4)
    K3 = ((gamma + 1)*Mach**8 + (2*gamma**2 - 7*gamma -5)*Mach**6 + 10*(gamma+1)*Mach**4 + 8) / (6 * beta**7)
    return K1, K2, K3

class FinSet(FixedMass, ActuatedSystem):
    ''' Class represents a set of n identical fins, all at the same longitudinal location, arranged axisymmetrically. Fin orientations can be controlled by a `MAPLEAF.GNC.ControlSystems.ControlSystem` '''

    #### Initialization Functions ####
    def __init__(self, componentDictReader, rocket, stage):
        FixedMass.__init__(self, componentDictReader, rocket, stage)

        self.controlSystem = None   # Set by self.initializeActuators, if it is called by a control system being initialized ex: RocketControlSystem.__init__()
        self.actuatorList = None    # Set by self.initializeActuators, if it is called by a control system being initialized ex: RocketControlSystem.__init__()

        self.numFins = componentDictReader.getInt("numFins")
        self.firstFinAngle = componentDictReader.getFloat("firstFinAngle")

        # Set fin properties
        self.finAngle = componentDictReader.getFloat("finCantAngle")
        self.sweepAngle = math.radians(componentDictReader.getFloat("sweepAngle"))
        self.rootChord = componentDictReader.getFloat("rootChord")
        self.tipChord = componentDictReader.getFloat("tipChord")
        self.span = componentDictReader.getFloat("span")
        self.surfaceRoughness = componentDictReader.tryGetFloat("surfaceRoughness", defaultValue=self.rocket.surfaceRoughness)
        self.thickness = componentDictReader.getFloat("thickness")
        self.numFinSpanSteps = componentDictReader.getInt("numFinSpanSlicesForIntegration")
        
        # Leading Edge properties
        self.leadingEdgeShape = componentDictReader.getString("LeadingEdge.shape")
        if self.leadingEdgeShape == "Round":
            self.leadingEdgeRadius = componentDictReader.tryGetFloat("LeadingEdge.radius", defaultValue=self.thickness/100)
        elif self.leadingEdgeShape == "Blunt":
            self.leadingEdgeThickness = componentDictReader.tryGetFloat("LeadingEdge.thickness", defaultValue=self.thickness)
        else:
            raise ValueError("ERROR: Leading edge shape: {} not implemented. Use 'Round' or 'Blunt'".format(self.leadingEdgeShape))

        # Trailing Edge properties
        self.trailingEdgeShape = componentDictReader.getString("TrailingEdge.shape")
        if self.trailingEdgeShape == "Round":
            self.trailingEdgeRadius = componentDictReader.getFloat("TrailingEdge.radius")
        elif self.trailingEdgeShape == "Blunt":
            self.trailingEdgeThickness = componentDictReader.getFloat("TrailingEdge.thickness")
        elif self.trailingEdgeShape != "Tapered":
            raise ValueError("ERROR: Trailing edge shape: {} not implemented. Use 'Round', 'Blunt', or 'Tapered'".format(self.trailingEdgeShape))

        # Common fin properties for all child fins
        self._precomputeGeometry()
        
        # Initialize all the child fins in the self.finList list
        self._initChildFins(componentDictReader, rocket, stage)

    def initializeActuators(self, controlSystem):
        self.controlSystem = controlSystem

        # Initialize an actuator model for each fin
        ActuatedSystem.__init__(self, self.numFins)

    def _precomputeGeometry(self):
        self._calculateSweepAngles()

        ### Compute Other Simple Properties ####
        self.planformArea = self.span * (self.tipChord + self.rootChord) / 2
        self.wettedArea = 2*self.planformArea
        self.aspectRatio = (2*self.span)**2/self.planformArea # Aspect ratio of the wing that would be created by reflecting the fin about its root chord (Fleeman/Niskanen)
        self.stallAngle = 15 * (1 + 1.1/self.aspectRatio) # Compute stall angle from Hoerner, 'Fluid Dynamic Lift' for low aspect ratio wings

        # Get the body radius where the fin is mounted
        stageTopZ = self.stage.position.Z
        distanceFromTopOfStageToTopOfThisFinSet = stageTopZ - self.position.Z
        self.bodyRadius = self.stage.getRadius(distanceFromTopOfStageToTopOfThisFinSet)

        self._calculateInterferenceFactors()
        self._calculateMACProperties()
        self._splitFinsIntoSlices()

        self._precomputeSubsonicFinThicknessDrag()
        self._precomputeCPInterpolationPolynomial()
    
    def _calculateSweepAngles(self):
        ''' Compute Trailing Edge (TE) and Mid Chord sweep angles '''
        LEtipChordBehindRootChord = self.span * math.tan(self.sweepAngle) # Z-distance the front of the tip chord is behind the front of the root chord
        TETipChordBehindRootChord = self.tipChord + LEtipChordBehindRootChord - self.rootChord # Z-distance the back of the tip chord is behind the back of the root chord

        if(TETipChordBehindRootChord == 0):
            self.trailingEdgeSweep = 0.0
        elif(TETipChordBehindRootChord < 0):
            # Negative trailing edge sweep is a forward-swept rear
            self.trailingEdgeSweep = -1*math.atan2(abs(TETipChordBehindRootChord),self.span)
        else:
            self.trailingEdgeSweep = math.atan2(abs(TETipChordBehindRootChord), self.span)

        self.midChordSweep = (self.sweepAngle + self.trailingEdgeSweep) / 2

        self.dZ_dSpan_LE_neg = math.tan(self.sweepAngle) # -dZ/dSpan (leading edge slope)
        self.dZ_dSpan_TE_neg = math.tan(self.trailingEdgeSweep) # dZ/dSpan (trailing edge slope)

    def _calculateInterferenceFactors(self):
        # Interference w/ body tube
        # Accounts for additional normal forces generated by the fins, due to the presence of the rocket body
        # Niskanen Eqn 3.56, originally from Barrowman
        self.bodyOnFinInterferenceFactor = 1 + self.bodyRadius / (self.span + self.bodyRadius)

        # Interference b/w the fins
        # Niskanen Eqn 3.54, from MIL-HDBK-762
        # Accounts for reduced normal force generated by large numbers of fins in a group
        # Not expected to work well for more than 8 fins
        if self.numFins <= 4:
            self.finNumberInterferenceFactor = 1.0
        elif self.numFins <= 8:
            numberToFactorMap = {
                5: 0.948,
                6: 0.913,
                7: 0.854,
                8: 0.810,
            }
            self.finNumberInterferenceFactor = numberToFactorMap[self.finNumber]
        else:
            self.finNumberInterferenceFactor = 0.75

    def _calculateMACProperties(self):
        ''' Calculates MAC Length, MACY, and MACX - Niskanen (Eqn. 3.30-3.32) '''
        def LEdgeAtY(y): # Leading Edge at y - where y is the spanwise coordinate
            return self.dZ_dSpan_LE_neg*y
                    
        numSpanSteps = round(self.span / 0.0001) # One step per 0.1 mm
        finSpanStep = self.span / numSpanSteps

        MACIntegralSum = 0
        MACYPosIntegralSum = 0
        XMACLeadingEdgeIntegralSum = 0
        for i in range(numSpanSteps):
            y = i * finSpanStep + finSpanStep/2
            MACIntegralSum += ((self.getChord(y)**2)*finSpanStep) # Eqn 3.30, integrated numerically
            MACYPosIntegralSum += y * (self.getChord(y) * finSpanStep) # Eqn 3.31, integrated numerically
            XMACLeadingEdgeIntegralSum += LEdgeAtY(y) * self.getChord(y)*finSpanStep # Eqn 3.32, integrated numerically

        # Save results
        self.MACLength = MACIntegralSum/self.planformArea
        self.MACYPos = MACYPosIntegralSum/self.planformArea
        self.XMACLeadingEdge = XMACLeadingEdgeIntegralSum/self.planformArea

    def _splitFinsIntoSlices(self):
        ''' Precomputes fin area slices for normal force integration '''
        stepSize = self.span / self.numFinSpanSteps
        bodyRadius = self.bodyRadius
        
        self.spanSliceAreas = []
        self.spanSliceRadii = []
        for i in range(self.numFinSpanSteps):
            y = i*stepSize + stepSize*0.5
            self.spanSliceRadii.append(y + bodyRadius)
            self.spanSliceAreas.append(stepSize*self.getChord(y))

    def _precomputeSubsonicFinThicknessDrag(self):
        ''' Precompute the subsonic thickness drag coefficient (Barrowman Eqn 4-36) '''
        sigma = (self.aspectRatio/2) * (self.thickness / self.rootChord)**(1/3) # Barrowman Eqn 4-35 b
        CD_TT_star = 1.15 * (self.thickness/self.rootChord)**(5/3) * (1.61 + sigma - sqrt((sigma - 1.43)**2 + 0.578))
        
        # Get skin friction coefficient at Mach 1
        mach1State = copy.deepcopy(self.rocket.rigidBody.state)
        mach1State.velocity = Vector(0,0,340.3) # Get speed up around Mach 1
        environment = self.rocket.environment.getAirProperties(self.rocket.rigidBody.state.position, 0)
        CD_f_star = AeroFunctions.getSkinFrictionCoefficient(mach1State, environment, self.MACLength, 1.0, self.surfaceRoughness, self.rocket.fullyTurbulentBL)
        
        # Final result
        self.subsonicFinThicknessK = cos(self.midChordSweep)**2 + (((CD_TT_star/CD_f_star - 4*cos(self.midChordSweep)*(self.thickness/self.rootChord)) / (120 * cos(self.midChordSweep)**2 * (self.thickness/self.rootChord)**4))**2)**(1/3)

    def _precomputeCPInterpolationPolynomial(self):
        ''' Precompute Interpolation polynomial coefficients for getCPXLocation function '''
        AR = self.aspectRatio
        f_2 = (AR * (3)**0.5 - 0.67) / (2 * (3)**0.5 * AR - 1)
        f_prime_2_t1 = (2 * (3)**-0.5 * AR) * (2 * AR * (3)**0.5 - 1) 
        f_prime_2_t2 = ((3)**0.5 * AR - 0.67) * (AR * 4 * (3)**-0.5)
        f_prime_2_denom = (2 * AR * (3)**0.5 -1)**2
        f_prime_2 = (f_prime_2_t1 - f_prime_2_t2) / f_prime_2_denom
        
        a_mat = [
                [0.5**5, 0.5**4, 0.5**3, 0.5**2, 0.5, 1],
                [5*(0.5)**4, 4*(0.5)**3, 3*(0.5)**2, 2*(0.5), 1, 0],
                [2**5, 2**4, 2**3, 2**2, 2, 1],
                [5*(2)**4, 4*(2)**3, 3*(2)**2, 2*2, 1, 0],
                [20*2**3, 12*2**2, 6*2, 2, 0, 0],
                [60*2**2, 24*2, 6, 0, 0, 0],
                ]
        
        b_vec = [0.25, 0, f_2, f_prime_2, 0, 0]
        
        # Save results
        self.x = np.linalg.solve(a_mat, b_vec)   

    def _initChildFins(self, componentDictReader, rocket, stage):
        self.finList = []
        
        finSeparationAngle = 360/self.numFins
        firstFinAngle = self.firstFinAngle

        # Create child fins
        for i in range(self.numFins):
            # Figure out what direction the fin points in
            spanwiseUnitX = math.cos(math.radians(firstFinAngle + i*finSeparationAngle))
            spanwiseUnitY = math.sin(math.radians(firstFinAngle + i*finSeparationAngle))
            spanWiseDirection = Vector(spanwiseUnitX, spanwiseUnitY, 0)

            #Initialize each fin separately and keep in a list
            self.finList.append(Fin(componentDictReader, self, spanWiseDirection, rocket, stage))

    def getLogHeader(self):
        header = " {}FX(N) {}FY(N) {}FZ(N) {}MX(Nm) {}MY(Nm) {}MZ(Nm)".format(*[self.name]*6)
        
        if self.controlSystem != None:
            for finNumber in range(1, self.numFins+1):
                header += " {}Fin{}Angle(deg)".format(self.name, finNumber)

        return header

    ### Functions used during simulation ###
    def getAeroForce(self, rocketState, time, environment, CG):
        #### If control system exists, use actuator deflections 1:1 to set fin angles ####
        if self.controlSystem != None:
            # Update fin angles
            for i in range(self.numFins):
                self.finList[i].finAngle = self.actuatorList[i].getDeflection(time)

        #### Pre-calculate common properties for all child Fins ####
        precomputedData = self._getPreComputedFinAeroData(rocketState, environment, CG)

        #### Add up forces from all child Fins ####
        aeroForce = ForceMomentSystem(Vector(0,0,0), self.position)
        for fin in self.finList:
            aeroForce += fin.getAeroForce(rocketState, time, environment, CG, precomputedData)

        # TODO: Correct for sub/transonic rolling moment fin-fin interference from a high number of fins
        
        # Apply fin-body interference factor to total forces in the normal (X/Y) directions and moments
        totalInterferenceFactor = self.bodyOnFinInterferenceFactor * self.finNumberInterferenceFactor
        aeroForce.force.X *= totalInterferenceFactor
        aeroForce.force.Y *= totalInterferenceFactor
        aeroForce.moment.X *= totalInterferenceFactor
        aeroForce.moment.Y *= totalInterferenceFactor          

        #### Log results ####
        forceLogLine = " {:>10.4f} {:>10.4f}".format(aeroForce.force, aeroForce.moment)
        if self.controlSystem != None:
            for i in range(len(self.actuatorList)):
                forceLogLine += " {:>6.4}".format(self.finList[i].finAngle)
        self.rocket.appendToForceLogLine(forceLogLine)

        return aeroForce

    def _getPreComputedFinAeroData(self, rocketState, environment, CG):
        #General Info ---------------------------------------------------------------------------------------------------------------------
        Aref = self.rocket.Aref
        Mach = AeroParameters.getMachNumber(rocketState, environment)
        dynamicPressure = AeroParameters.getDynamicPressure(rocketState, environment)

        # Skin Friction Drag -------------------------------------------------------------------------------------------------------------
        
        skinFrictionCoefficient = AeroFunctions.getSkinFrictionCoefficient(rocketState, environment, self.MACLength, Mach, self.surfaceRoughness, self.rocket.fullyTurbulentBL)    
        # Adjust to the rocket reference area (skinFrictionCoefficient is based on wetted area)
        skinFrictionDragCoefficient = skinFrictionCoefficient*(self.wettedArea / Aref)
        # Correct for additional surface area due to fin thickness - Niskanen Eqn 3.85
        skinFrictionDragCoefficient *= (1 + 2*self.thickness/self.MACLength)
        
        # Pressure Drag ------------------------------------------------------------------------------------------------------------------

        # Leading edge drag coefficient
        if self.leadingEdgeShape == "Round":
            leadingEdgeCd = AeroFunctions.getCylinderCrossFlowCd_ZeroBaseDrag(Mach)
            LEthickness = self.leadingEdgeRadius*2
        elif self.leadingEdgeShape == "Blunt":
            leadingEdgeCd = AeroFunctions.getBluntBodyCd_ZeroBaseDrag(Mach)
            LEthickness = self.leadingEdgeThickness

        # Adjust for leading edge angle and convert reference area to rocket reference area - Barrowman Eqn 4-22
        leadingEdgeCdAdjustmentFactor = LEthickness * self.span * cos(self.sweepAngle)**2 / Aref
        leadingEdgeCd *= leadingEdgeCdAdjustmentFactor
            
        # Trailing edge drag coefficient - simpler method from Niskanen section 3.4.4. Corrected to use only the trailing edge area, not the full fin frontal area
            # more intricate method available in Barrowman
        baseDragCd = AeroFunctions.getBaseDragCoefficient(Mach)
        if self.trailingEdgeShape == "Tapered":
            TEthickness = 0 # Zero base drag
        elif self.trailingEdgeShape == "Round":
            TEthickness = self.trailingEdgeRadius # 0.5 base drag
        elif self.trailingEdgeShape == "Blunt":
            TEthickness = self.trailingEdgeThickness # Full base drag
        
        # Convert to standard rocket reference Area
        trailingEdgeCd = baseDragCd * self.span*TEthickness/Aref

        # Thickness / Wave Drag
        #TODO: This section doesn't seem to be working quite right
        if Mach <= 1:
            # Thickness drag, subsonic
            thicknessDrag = 4*skinFrictionCoefficient*((self.thickness/self.rootChord)*cos(self.midChordSweep) + \
                (30 * (self.thickness/self.rootChord)**4 * cos(self.midChordSweep)**2) /  \
                    (self.subsonicFinThicknessK - Mach**2 * cos(self.midChordSweep)**2)**(3/2))
        else:
            # Supersonic wave drag
            # Using simplistic method from Hoerner - assumes diamond profile (pg 17-12, Eqn 29)
            # TODO: Implement method from Barrowman's FIN program
            thicknessDrag = 2.3*self.aspectRatio * (self.thickness / self.MACLength)**2

        thicknessDrag *= self.planformArea/Aref

        pressureDragCoefficient = leadingEdgeCd + trailingEdgeCd + thicknessDrag
                        
        # Total Drag --------------------------------------------------------------------------------------------------------------------
        
        totalDragCoefficient = pressureDragCoefficient + skinFrictionDragCoefficient

        localFrameRocketVelocity = AeroParameters.getLocalFrameAirVel(rocketState, environment)
        axialPositionRelCG = self.position - CG
        finVelocityDueToRocketPitchYaw = rocketState.angularVelocity.crossProduct(axialPositionRelCG) 
        airVelRelativeToFin = localFrameRocketVelocity - finVelocityDueToRocketPitchYaw # The negative puts it in the wind frame
        CPXPos = self._getCPXPos(Mach)

        #### Transfer info to fins ####
        return PreComputedFinAeroData(airVelRelativeToFin, CPXPos, totalDragCoefficient)

    def _getCPXPos(self, Mach):
        ''' 
            Finds the chordwise position of the center of pressure, relative to the tip of the root chord (self.position) 
            Methods from Tactical missile design by Fleeman through Niskanen
        '''
        if Mach < 0.5:
            XF = self.XMACLeadingEdge + 0.25*self.MACLength #As per open rocket documentation

        elif Mach >= 2.0:
            beta = AeroParameters.getBeta(Mach)
            CPCalc = lambda AR, B : (AR*B - 0.67) / (2*AR*B - 1) #Eq 3.35 of open rocket documentation (greater than M = 2)
            XF = self.MACLength*CPCalc(self.aspectRatio, beta) + self.XMACLeadingEdge #As per open rocket documentation

        else:
            #Between the two extremes is a polynomial curve fit
            #Fifth order polynomial fit as per Niskanen Eqn 3.36 (Originally from Fleeman)
                # Old code: XF = np.polyval(self.x, Mach)*self.MACLength + self.XMACLeadingEdge
                # Evaluate polynomial manually for maximum speed (8.5x improvement for 6 coeffs)
                    # Speed test in test/speedTests/PolyvalSpeed.py
            polyval = 0
            nTerms = len(self.x)
            for i in range(nTerms):
                polyval += self.x[i]*Mach**(nTerms-1-i)

            XF = polyval*self.MACLength + self.XMACLeadingEdge # Polyval defines the fraction of the MAC that the Cp is behind the leading edge

        return XF

    def getChord(self, y):
        ''' Returns chord at y, where y==0 is at the root chord, and y==span is at the tip chord '''
        return (self.tipChord - self.rootChord) * (y/self.span) + self.rootChord

    def plotShape(self):
        import matplotlib.pyplot as plt

        BodyTubeDiameter = self.bodyRadius*2
        SubComponentPos = self.position.Z
        span = self.span
        rootChord = self.rootChord
        tipChord = self.tipChord
        sweepAngle = self.sweepAngle
        Xvals = []
        Yvals = []
        Xvals.append(SubComponentPos)
        Yvals.append(BodyTubeDiameter/2)
        Xvals.append(Xvals[0]-span*math.tan(sweepAngle))
        Yvals.append(Yvals[0]+span)
        Xvals.append(Xvals[1]-tipChord)
        Yvals.append(Yvals[1])
        Xvals.append(Xvals[0]-rootChord)
        Yvals.append(Yvals[0])
        Xvals.append(Xvals[0])
        Yvals.append(Yvals[0])
        plt.plot(Xvals,Yvals, color = 'k')

        if self.numFins == 4:
            Yvals = [-1*i for i in Yvals]
            plt.plot(Xvals,Yvals, color = 'k')
            Yvals = [0*i for i in Yvals]
            plt.plot(Xvals,Yvals, color = 'k')

        elif self.numFins == 3:
            Yvals = []
            Yvals.append(-BodyTubeDiameter/2*math.sin(math.radians(30)))
            Yvals.append(-(BodyTubeDiameter/2 + span)*math.sin(math.radians(30)))
            Yvals.append(Yvals[1])
            Yvals.append(Yvals[0])
            Yvals.append(Yvals[0])
            plt.plot (Xvals, Yvals, color = 'k')

class Fin(FixedMass):
    ''' Represents a single fin in a FinSet '''

    def __init__(self, componentDictReader, parentFinSet, spanwiseDirection, rocket, stage):

        FixedMass.__init__(self, componentDictReader, rocket, stage)

        self.finSet = parentFinSet # type: FinSet
        ''' Reference to the parent fin set '''

        self.spanSliceAreas = self.finSet.spanSliceAreas
        self.spanSliceRadii = self.finSet.spanSliceRadii

        self.finAngle = self.finSet.finAngle
        self.stallAngle = self.finSet.stallAngle

        self.span = self.finSet.span
        self.planformArea = self.finSet.planformArea
        self.midChordSweep = self.finSet.midChordSweep
        
        # Unit vector parallel to the fin's shaft and/or fin's spanwise direction, normal to the rocket's body tube
        self.spanwiseDirection = spanwiseDirection

        # Vector normal to the fin surface, in the X/Y Plane, when self.finAngle == 0
        self.undeflectedFinNormal = Vector(self.spanwiseDirection[1], -self.spanwiseDirection[0], 0)

        # Spanwise component of Center of Pressure (CP) location
        self.CPSpanWisePosition = self.spanwiseDirection*(self.finSet.MACYPos + self.finSet.bodyRadius)

    def _barrowmanAeroFunc(self, rocketState, time, environment, precomputedData, CG = Vector(0,0,0), finDeflectionAngle = None):
        '''
            Precomputed Data is a named tuple (PreComputedFinAeroData) which contains data/results from the parts of the fin aerodynamics calculation
                that are common to all fins in a FinSet (drag calculations mostly). These calculations are performed at the FinSet level.
            Only the parts of the Fin Aero Computation that change from fin to fin (normal force mostly, since different fins can have different angles of attack) are computed here
        '''
        Mach = AeroParameters.getMachNumber(rocketState, environment)
        dynamicPressure = AeroParameters.getDynamicPressure(rocketState, environment)

        if finDeflectionAngle == None:
            finDeflectionAngle = self.finAngle # Adjusted by parent finset during each timestep, when the FinSet is controlled

        # Unpack precomputedData
        airVelRelativeToFin, CPXPos, totalDragCoefficient = precomputedData

        #### Compute normal force-----------------------------------------------------------------------------------------------------------------
           
        # Find fin normal vector after fin deflection       
        finDeflectionRotation = Quaternion(axisOfRotation=self.spanwiseDirection, angle=radians(finDeflectionAngle))
        finNormal = finDeflectionRotation.rotate(self.undeflectedFinNormal)
        # Get the tangential velocity component vector, per unit radial distance from the rocket centerline
        rollAngVel = AngularVelocity(0, 0, rocketState.angularVelocity.Z)
        unitSpanTangentialAirVelocity = rollAngVel.crossProduct(self.spanwiseDirection)*(-1)
        
        def subsonicNormalForce(Mach): # Subsonic linear method
            tempBeta = AeroParameters.getBeta(Mach)
            CnAlpha = getFinCnAlpha_Subsonic_Barrowman(self.span, self.planformArea, tempBeta, self.midChordSweep)
            return getSubsonicFinNormalForce(airVelRelativeToFin, unitSpanTangentialAirVelocity, finNormal, self.spanwiseDirection, self.CPSpanWisePosition.length(), CnAlpha, self)

        def supersonicNormalForce(Mach): # Supersonic Busemann method
            gamma = AeroFunctions.getGamma()
            tempBeta = AeroParameters.getBeta(Mach)
            K1, K2, K3 = getBusemannCoefficients(Mach, tempBeta, gamma)
            return getSupersonicFinNormalForce(airVelRelativeToFin, unitSpanTangentialAirVelocity, finNormal, self.spanwiseDirection, self.CPSpanWisePosition.length(), K1, K2, K3, self)

        if Mach <= 0.8:
            normalForceMagnitude, finMoment = subsonicNormalForce(Mach)

        elif Mach < 1.5:
            # Interpolate in transonic region
            # TODO: Do this with less function evaluations? Perhaps precompute AOA and Mach combinations and simply interpolate? Lazy precompute? Cython?
            x1, x2 = 0.8, 1.5 # Start, end pts of interpolated region
            dx = 0.001

            # Find normal force and derivative at start of interpolation interval
            f_x1, m_x1 = subsonicNormalForce(x1)
            f_x12, m_x12 = subsonicNormalForce(x1+dx)

            # Find normal force and derivative at end of interpolation interval
            f_x2, m_x2 = supersonicNormalForce(x2)
            f_x22, m_x22 = supersonicNormalForce(x2+dx)

            normalForceMagnitude = Interpolation.cubicInterp(Mach, x1, x2, f_x1, f_x2, f_x12, f_x22, dx)
            finMoment = Interpolation.cubicInterp(Mach, x1, x2, m_x1, m_x2, m_x12, m_x22, dx)
        else:
            normalForceMagnitude, finMoment = supersonicNormalForce(Mach)
             
        # Complete redimensionalization of normal force coefficients by multiplying by dynamic pressure
        # Direct the normal force along the fin's normal direction
        normalForce = normalForceMagnitude * dynamicPressure * finNormal
        finMoment *= dynamicPressure
                
        #### Get axial force-----------------------------------------------------------------------------------------------------------------------
        
        avgAOA = getFinSliceAngleOfAttack(self.spanSliceRadii[round(len(self.spanSliceAreas)/2)], airVelRelativeToFin, unitSpanTangentialAirVelocity, finNormal, self.spanwiseDirection, self.stallAngle) # Approximate average fin AOA
        totalAxialForceCoefficient = AeroFunctions.getDragToAxialForceFactor(avgAOA) * totalDragCoefficient
        axialForceMagnitude = totalAxialForceCoefficient * self.rocket.Aref * dynamicPressure
        axialForceDirection = self.spanwiseDirection.crossProduct(finNormal)
        axialForce = axialForceDirection*axialForceMagnitude
        
        #### Get CP Location ----------------------------------------------------------------------------------------------------------------------

        CPChordWisePosition = self.position - Vector(0, 0, CPXPos) # Ignoring the change in CP position due to fin deflection for now
        globalCP = self.CPSpanWisePosition + CPChordWisePosition

        #### Assemble total force moment system objects--------------------------------------------------------------------------------------------

        totalForce = normalForce + axialForce
        return ForceMomentSystem(totalForce, globalCP, moment=Vector(0, 0, finMoment)), globalCP

    def getAeroForce(self, rocketState, time, environment, CG, precomputedData):
        [ aeroForce, CP ] = self._barrowmanAeroFunc(rocketState, time, environment, precomputedData, CG)
        return aeroForce
