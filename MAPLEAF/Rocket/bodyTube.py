
import math

from . import AeroFunctions
from MAPLEAF.Motion import AeroParameters, Vector
from MAPLEAF.Rocket import BodyComponent, FixedMass
from MAPLEAF.Utilities import logForceResult

__all__ = [ "BodyTube" ]

class BodyTube(FixedMass, BodyComponent):
    ''' Represent a cylindrical body tube '''
    
    #### Init Functions ####
    def __init__(self, componentDictReader, rocket, stage):
        FixedMass.__init__(self, componentDictReader, rocket, stage)

        self.length = componentDictReader.getFloat("length")
        self.outerDiameter = componentDictReader.getFloat("outerDiameter")
        self.surfaceRoughness = componentDictReader.tryGetFloat("surfaceRoughness", defaultValue=self.rocket.surfaceRoughness)

        # Tell the rocket/stage what its diameter is
        self.rocket.bodyTubeDiameter = self.outerDiameter
        self.stage.bodyTubeDiameter = self.outerDiameter

        self._precomputeGeometry()

    def _precomputeGeometry(self):
        self.volume = math.pi * self.outerDiameter**2 / 4 * self.length
        self.planformArea = self.outerDiameter * self.length
        self.wettedArea = math.pi * self.outerDiameter * self.length
        
        self.CPLocation = self.position - Vector(0,0,self.length/2)

    #### Operational Functions ####
    @logForceResult
    def getAeroForce(self, rocketState, time, environment, CG):
        Aref = self.rocket.Aref

        # Normal Force ----------------------------------------------------------------------------------------
        AOA = AeroParameters.getTotalAOA(rocketState, environment)
        # Niskanen Eqn 3.26 - originally from Galejs
        normalForceCoefficient = 1.1 * (math.sin(AOA))**2
        normalForceCoefficient *= self.planformArea / Aref

        # Drag -----------------------------------------------------------------------------------------------
        # Skin Friction
        Mach = AeroParameters.getMachNumber(rocketState, environment)
        skinFrictionDragCoefficient, rollDampingMoment = AeroFunctions.getCylindricalSkinFrictionDragCoefficientAndRollDampingMoment(rocketState, environment, self.length, Mach, self.surfaceRoughness, \
            self.wettedArea, Aref, self.rocket.finenessRatio, self.rocket.fullyTurbulentBL)
        
        #There is no pressure drag associated with bodytubes - skin friction drag is total drag

        # Damping moments --------------------------------------------------------------------------------------
        dampingMoments = self._computeLongitudinalDampingMoments(rocketState, environment, CG)
        
        # Roll damping due to skin friction
        dampingMoments += rollDampingMoment

        # Compute & dimensionalize total-------------------------------------------------------------------------
        return AeroFunctions.forceFromCdCN(rocketState, environment, skinFrictionDragCoefficient, normalForceCoefficient, self.CPLocation, Aref, moment=dampingMoments)

    def _computeLongitudinalDampingMoments(self, rocketState, environment, CoR, nSegments=25):
        ''' CoR = Center of Rotation (usually the rocket's CG) '''
        # TODO: Check if there's an nice analytical solution to this integral

        # Eqn (3.57) from Niskanen, converted to vector form and integrated numerically
        # Numerical integration: Divide body tube into segments
            # Works for body tubes in any position, rotating on any axis
        dL = self.length / nSegments
        dA = dL * self.outerDiameter
        totalDampingMoment = Vector(0,0,0)

        for i in range(nSegments):
            # Center of current segment of body tube
            segmentCentroid = self.position + Vector(0,0, -dL*i - dL/2)
            
            # Vector from center of rotation to segment centroid
            psi = segmentCentroid - CoR 

            # Velocity due to rotation is angular velocity cross distance (from center of rotation) vector
            segmentVelocity = rocketState.angularVelocity.crossProduct(psi)
            
            # Re-dimensionalizing coefficient required squared velocity - while preserving sign
            sqVel = Vector( segmentVelocity.X*abs(segmentVelocity.X), segmentVelocity.Y*abs(segmentVelocity.Y), \
                 segmentVelocity.Z*abs(segmentVelocity.Z) )

            # Moment is distance vector cross force vector
            dM = -psi.crossProduct(sqVel)
            totalDampingMoment += dM

        # All terms of the integral summation must be multiplied by this constant:
            # 1.1 is drag coefficient of a cylinder in crossflow
            # 1/2 and density are from re-dimensionalizing the coefficient
            # Outerdiameter is part of area calculation
        integrationConstantMultiple = (1.1/2)*environment.Density*dA
        totalDampingMoment *= integrationConstantMultiple

        return totalDampingMoment

    def getMaxDiameter(self):
        return self.outerDiameter

    def getRadius(self, _):
        return self.outerDiameter/2

    def plotShape(self):
        import matplotlib.pyplot as plt
        
        # Assume body tube is on the Z-axis
        tipZ = self.position.Z
        tailZ = tipZ - self.length
        radius = self.outerDiameter/2

        Xvals = []
        Yvals = []
        Xvals.append(tipZ)
        Yvals.append(radius) # top right

        Xvals.append(tipZ)
        Yvals.append(-radius) # bottom right

        Xvals.append(tailZ)
        Yvals.append(-radius) # bottom left 

        Xvals.append(tailZ)
        Yvals.append(radius) # top left

        Xvals.append(tipZ)
        Yvals.append(radius) # close the square
        plt.plot(Xvals, Yvals, color = 'k')
