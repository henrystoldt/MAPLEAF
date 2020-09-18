import math

import numpy as np

from MAPLEAF.Motion import AeroParameters, Interpolation
from MAPLEAF.Rocket import AeroFunctions, BodyComponent, FixedMass
from MAPLEAF.Utilities import logForceResult

__all__ = [ "NoseCone" ]

#TODO: Implement Barrowman's second-order shock expansion method for pressure drag at high mach numbers

def calcSonicPressureDragCoeff_Cone(AR: float) -> float:
    ''' AR: Apect/fineness ratio of the nosecone: Barrowman Eqn 4-42 '''
    return 0.88 / ((AR + 0.7)**1.29)

def calcSonicPressureDragCoeff_Ogive(AR:float) -> float:
    ''' AR: Apect/fineness ratio of the nosecone:  Barrowman Eqn 4-43 '''
    return 0.88 / ((AR + 1)**2.22)

def calculatePrandtlFactorCorrection(wettedArea, sonicSkinFrictionCoeff, rocketFinenessRatio, Aref, sonicPressureDragCoeff):
    ''' Calculate the correction constant K: Barrowman Eqn 4-44 '''
    return 1 + (6 * wettedArea * sonicSkinFrictionCoeff / (rocketFinenessRatio**3 * Aref * sonicPressureDragCoeff))**(5/3)

def getSubsonicPressureDragCoeff_Barrowman(wettedArea, skinFrictionCoeff, rocketFinenessRatio, Aref, K, Mach):
    ''' Barrowman Eqn 4-41 '''
    return 6 * wettedArea * skinFrictionCoeff / (rocketFinenessRatio**3 * Aref * (K - Mach**2)**0.6)

def getSupersonicPressureDragCoeff_Hoerner(coneHalfAngle, Mach):
    ''' Niskanen Eqn B.4, Hoerner pg. 16-18 to 16-20 '''
    return 2.1 * (math.sin(coneHalfAngle))**2 + 0.5 * math.sin(coneHalfAngle) / math.sqrt(Mach**2 - 1)

def computeSubsonicPolyCoeffs(coneHalfAngle):
    ''' 
        Computes coefficients for a power law drag relationship up to Mach 1 (Niskanen Appendix B eq. B.5)
        Pass in half angle in radians
    '''
    gamma = AeroFunctions.getGamma()
    dCd_dMa_M1 = 4 / (gamma + 1) * (1 - 0.5 * math.sin(coneHalfAngle))
    Cd_M1 = math.sin(coneHalfAngle)

    return [ Cd_M1, (dCd_dMa_M1/Cd_M1) ]

def computeTransonicPolyCoeffs(coneHalfAngle):
    ''' 
        Computes coefficients for a cubic drag interpolation based on Mach Number b/w Mach 1 and 1.3 (Niskanen Appendix B eq. B.5)
        Pass in half angle in radians
    '''
    gamma = AeroFunctions.getGamma()
    dCd_dMa_M1 = 4 / (gamma + 1) * (1 - 0.5 * math.sin(coneHalfAngle))
    Cd_M1 = math.sin(coneHalfAngle)
    
    Cd_M13 = getSupersonicPressureDragCoeff_Hoerner(coneHalfAngle, 1.3)
    Cd_M1301 = getSupersonicPressureDragCoeff_Hoerner(coneHalfAngle, 1.301)
    dCd_dMa_M13 = (Cd_M1301 - Cd_M13) / 0.001

    return Interpolation.calculateCubicInterpCoefficients(1.0, 1.3, Cd_M1, Cd_M13, dCd_dMa_M1, dCd_dMa_M13)

class NoseCone(FixedMass, BodyComponent):
    ''' Represent a Tangent Ogive Nosecone '''

    canConnectToComponentAbove = False # Overrides attribute from BodyComponent -> Nosecone must be at top of rocket

    #### Initialization Functions ####
    def __init__(self, componentDictReader, rocket, stage):
        FixedMass.__init__(self, componentDictReader, rocket, stage)
        
        self.aspectRatio = componentDictReader.getFloat("aspectRatio")
        ''' Length over diameter - aka fineness ratio (of exposed surface area) '''

        self.baseDiameter = componentDictReader.getFloat("baseDiameter")
        ''' Diameter at base of nosecone (m) '''

        self.length = self.aspectRatio * self.baseDiameter
        ''' Length from tip to tail, of external surface (m) '''

        self.shape = componentDictReader.getString("shape")
        ''' Description of nosecone shape - something like 'Ogive' or 'Cone' '''
        
        self.surfaceRoughness = componentDictReader.tryGetFloat("surfaceRoughness", defaultValue=self.rocket.surfaceRoughness)
        ''' (m) '''

        self._precomputeGeometry()

    def _precomputeGeometry(self):
        self.length = self.baseDiameter * self.aspectRatio
        self.volume = self._getVolume()
        self.planformArea = self._getPlanformArea()
        self.wettedArea = self._getWettedArea()

        # Using Barrowman's method, the CP Location is a constant
        self.baseArea = self.baseDiameter**2 * math.pi / 4
        self.CPLocation = AeroFunctions.Barrowman_GetCPLocation(self.length, 0, self.baseArea, self.volume)

        # Precompute coefficients for pressure drag interpolation-------------------------------------------        
        #  Niskanen 3.4 and Appendix B
        self.coneHalfAngle = abs(math.atan((self.baseDiameter/2) / self.length))

        self.SubsonicCdPolyCoeffs = computeSubsonicPolyCoeffs(self.coneHalfAngle)
        self.TransonicCdPolyCoeffs = computeTransonicPolyCoeffs(self.coneHalfAngle)

    def _getVolume(self):

        if self.shape == "tangentOgive":
            #https://www.rocketryforum.com/attachments/nosecone_eqn2-pdf.336812/
            baseRadius = self.baseDiameter / 2
            length = self.baseDiameter * self.aspectRatio

            noseconeRadiusOfCurvature = (baseRadius**2 + length**2) / (2*baseRadius)
        
            return math.pi * ( length * noseconeRadiusOfCurvature**2 - length**3 / 3 - (noseconeRadiusOfCurvature - baseRadius) * 
                noseconeRadiusOfCurvature**2 * (math.asin(length / noseconeRadiusOfCurvature)))
        
        elif self.shape == "cone":
            return math.pi * (self.baseDiameter)**2 * self.length / 3

        elif self.shape == "halfSeries":
            #https://www.rocketryforum.com/attachments/nosecone_eqn2-pdf.336812/
            baseRadius = self.baseDiameter / 2
            length = self.baseDiameter * self.aspectRatio

            return math.pi * baseRadius**2 * length / 2

        else:
            raise TypeError("The nosecone shape {} is not recognized".format(self.shape))

    def _getPlanformArea(self):

        if self.shape == "tangentOgive":
            baseRadius = self.baseDiameter / 2
            length = self.baseDiameter * self.aspectRatio

            noseconeRadiusOfCurvature = (baseRadius**2 + length**2) / (2*baseRadius)

            arcSectorAngle = math.asin(length / noseconeRadiusOfCurvature)
          
            arcSectorArea = arcSectorAngle * noseconeRadiusOfCurvature**2 / 2

            return 2 * (arcSectorArea - (length * (noseconeRadiusOfCurvature - baseRadius) / 2))

        elif self.shape == "cone":
            return self.baseDiameter * self.length / 2

        else:
            raise TypeError("The nosecone shape {} is not recognized".format(self.shape))

    def _getWettedArea(self):

        if self.shape == "tangentOgive":
            #TODO Analytical Solution: https://www.rocketryforum.com/attachments/nosecone_eqn2-pdf.336812/
            baseRadius = self.baseDiameter / 2
            length = self.baseDiameter * self.aspectRatio

            noseconeRadiusOfCurvature = (baseRadius**2 + length**2) / (2*baseRadius)
            a = noseconeRadiusOfCurvature

            numSteps = 2000
            stepSize = length / numSteps
            wettedArea = 0

            for x in range(numSteps + 1):
                wettedArea += 2 * math.pi * ((math.sqrt(a**2 - (length - x*stepSize)**2)) + baseRadius - a) * stepSize

            return wettedArea

        elif self.shape == "cone":
            baseRad = self.baseDiameter/2
            return math.pi * baseRad * math.sqrt(baseRad**2 + self.length**2)

        else:
            raise TypeError("The nosecone shape {} is not recognized".format(self.shape))

    #### Operation Functions ####
    @logForceResult
    def getAeroForce(self, rocketState, time, environment, CG):
        Aref = self.rocket.Aref
        Mach = AeroParameters.getMachNumber(rocketState, environment)

        # Normal Force --------------------------------------------------------------------------------
        # TODO: Account for rate of pitch/yaw rotation in AOA calculation? Or do separate Pitch/Yaw damping moments?
        AOA = AeroParameters.getTotalAOA(rocketState, environment)
        normalForceCoefficient = AeroFunctions.Barrowman_GetCN(AOA, Aref, 0, self.baseArea)

        # Drag Force ---------------------------------------------------------------------------------------------
        #### Skin Friction ####
        skinFrictionDragCoefficient, rollDampingMoment = AeroFunctions.getCylindricalSkinFrictionDragCoefficientAndRollDampingMoment(rocketState, environment, self.length, Mach,  \
            self.surfaceRoughness, self.wettedArea, Aref, self.rocket.finenessRatio, self.rocket.fullyTurbulentBL)
        
        #### Pressure ####
        pressureDragCoefficient = self._getCd_Pressure(Mach)

        totalDragCoefficient = pressureDragCoefficient + skinFrictionDragCoefficient

        # Damping moments --------------------------------------------------------------------------------------
        # Roll damping due to skin friction
        dampingMoments = rollDampingMoment

        # Combine forces and return total -------------------------------------------------------------------------------------
        return AeroFunctions.forceFromCdCN(rocketState, environment, totalDragCoefficient, normalForceCoefficient, self.CPLocation, Aref, moment=dampingMoments)

    def _getCd_Pressure(self, Mach):
        if Mach < 1:
            # Niskanen pg. 48 eq. 3.87 - Power law interpolation
            pressureDragCoefficient = self.SubsonicCdPolyCoeffs[0] * Mach**self.SubsonicCdPolyCoeffs[1]

        elif Mach > 1 and Mach < 1.3:
            # Interpolate in transonic region - derived from Niskanen Appendix B, Eqns B.3 - B.6
            pressureDragCoefficient = self.TransonicCdPolyCoeffs[0] + self.TransonicCdPolyCoeffs[1]*Mach +  \
                self.TransonicCdPolyCoeffs[2]*Mach**2 + self.TransonicCdPolyCoeffs[3]*Mach**3
            
        else:
            pressureDragCoefficient = getSupersonicPressureDragCoeff_Hoerner(self.coneHalfAngle, Mach)

        return pressureDragCoefficient * self.baseArea/self.rocket.Aref

    def getMaxDiameter(self):
        return self.baseDiameter

    def getRadius(self, distanceFromTip: float) -> float:
        if self.shape == "cone":
            return (self.baseDiameter/2) * (distanceFromTip/self.length)

        elif self.shape == "tangentOgive":
            curveRadius = ((self.baseDiameter/2)**2 + self.length**2) / (self.baseDiameter)
            yComponent = math.sqrt(curveRadius**2 - (self.length - distanceFromTip)**2)
            return yComponent - (curveRadius - (self.baseDiameter/2))

    def plotShape(self) -> None:
        import matplotlib.pyplot as plt
        
        SubComponentPos = self.position.Z
        length = self.baseDiameter * self.aspectRatio
        ConeRad = self.baseDiameter/2
        Rho = (ConeRad**2 + length**2)/(2*ConeRad) # values for tangent ogive equation
        Xvals = list(np.linspace(0,length, num=100)) # create 50 element tuple from 0 to length

        Yvals = [] # create y list

        for x in Xvals:
            # populate y list
            Yvals.append(self.getRadius(x))
        
        Xvals = [-1*i + SubComponentPos for i in Xvals] # these were positive. Need them to be negative to point correct way

        # top line, body tube to tip
        XvalsTop = list(Xvals) 
        YvalsTop = list(Yvals)
        XvalsTop.reverse()
        YvalsTop.reverse()

        # bottom line, tip to body tube
        XvalsBot = list(Xvals)
        YvalsBot = list(Yvals)
        YvalsBot = [-1*i for i in YvalsBot] # so the yvals are negative

        # put it all together now
        Xvals = XvalsTop + XvalsBot
        Yvals = YvalsTop + YvalsBot
        plt.plot(Xvals,Yvals, color = 'k')                 
