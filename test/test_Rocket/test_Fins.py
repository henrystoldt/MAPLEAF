
#Created by: Declan Quinn
#May 2019

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import math
import unittest
from math import radians

from MAPLEAF.IO import SimDefinition, SubDictReader
from MAPLEAF.Motion import (AeroParameters, AngularVelocity, Quaternion,
                            RigidBodyState, Vector)
from MAPLEAF.Rocket import FinSet, Rocket
from MAPLEAF.Rocket.CythonFinFunctions import getFinSliceAngleOfAttack


class TestFinSet(unittest.TestCase):
    def setUp(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/test3.mapleaf", silent=True)
        rocketDictReader = SubDictReader("Rocket", simDef)
        self.rocket = Rocket(rocketDictReader, silent=True)

        simDef = SimDefinition("MAPLEAF/Examples/Simulations/test6.mapleaf", silent=True)
        rocketDictReader = SubDictReader("Rocket", simDef)
        self.rocket2 = Rocket(rocketDictReader, silent=True)
        self.finSet1 = self.rocket2.stages[0].getComponentsOfType(FinSet)[0]
        self.finSet2 = self.rocket2.stages[0].getComponentsOfType(FinSet)[1]

        self.dummyVelocity1 = Vector(0, 0, 50)
        self.dummyVelocity2 = Vector(1, 0, 20)
        self.dummyVelocity3 = Vector(0, 0, -100)

        self.dummyOrientation1 = Quaternion(Vector(1, 0, 0), math.radians(2))
        self.dummyOrientation2 = Quaternion(Vector(1, 0, 0), math.radians(-2))
        self.dummyOrientation3 = Quaternion(Vector(0, 1, 0), math.radians(2))
        self.dummyOrientation4 = Quaternion(Vector(1, 0, 0), 0)
        self.dummyOrientation5 = Quaternion(Vector(1, 1, 0), math.radians(2))
        self.dummyOrientation6 = Quaternion(Vector(1,0,0), math.radians(90))

        self.dummyAngularVelocity1 = AngularVelocity(rotationVector = Vector(0,0,0))
        self.dummyAngularVelocity2 = AngularVelocity(rotationVector = Vector(0,0,1))
        self.dummyAngularVelocity3 = AngularVelocity(rotationVector = Vector(0,0,-1))
        self.dummyAngularVelocity4 = AngularVelocity(rotationVector = Vector(1,0,0))
        self.dummyAngularVelocity5 = AngularVelocity(rotationVector = Vector(-1,0,0))
        self.dummyAngularVelocity6 = AngularVelocity(rotationVector = Vector(0,1,0))
        self.dummyAngularVelocity7 = AngularVelocity(rotationVector = Vector(0,-1,0))
        self.dummyAngularVelocity8 = AngularVelocity(rotationVector = Vector(1,1,0))

        self.currentConditions = self.rocket2.environment.getAirProperties(Vector(0,0,200)) # m

        self.rocketState1 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState2 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(1, 0, 0), math.radians(2)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState3 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 500), Quaternion(Vector(1, 0, 0), math.radians(2)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState4 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, -200), Quaternion(Vector(1, 0, 0), math.radians(180)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState5 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, -200), Quaternion(Vector(1, 0, 0), math.radians(178)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState6 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, -200), Quaternion(Vector(1, 0, 0), math.radians(182)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState7 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, -200), Quaternion(Vector(1, 0, 0), math.radians(90)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState8 = RigidBodyState(Vector(0, 0, 200), Vector(20.04, -0.12, -52.78), Quaternion(Vector(0, 1, 0), math.radians(90)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState9 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(1, 0, 0), math.radians(-2)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState10 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 1)))
        self.rocketState11 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(0, 1, 0), math.radians(2)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState12 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(0, 1, 0), math.radians(-2)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState13 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 500), Quaternion(Vector(1, 0, 0), math.radians(4)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState14 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 500), Quaternion(Vector(1, 0, 0), math.radians(6)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState15 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 500), Quaternion(Vector(1, 0, 0), math.radians(8)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState16 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 344), Quaternion(Vector(1, 0, 0), math.radians(0)), AngularVelocity(rotationVector=Vector(0, 0, 0)))

    #### Fins ####

    def test_getFinVelocityDueToRocketPitchYaw(self):
        finSet = self.finSet1

        def getFinVelWithoutRoll(finSet, angVel):
            # Create dummy rocket state
            rotatingState = RigidBodyState(Vector(0,0,0), Vector(0,0,0), Quaternion(1,0,0,0), angVel)
            return finSet._getPreComputedFinAeroData(rotatingState, self.currentConditions, Vector(0,0,0))

        # Compute some sample results
        inducedVelocity1, _, _ = getFinVelWithoutRoll(finSet, self.dummyAngularVelocity1)
        inducedVelocity3, _, _ = getFinVelWithoutRoll(finSet, self.dummyAngularVelocity4)
        inducedVelocity5, _, _ = getFinVelWithoutRoll(finSet, self.dummyAngularVelocity5)
        inducedVelocity7, _, _ = getFinVelWithoutRoll(finSet, self.dummyAngularVelocity6)
        inducedVelocity9, _, _ = getFinVelWithoutRoll(finSet, self.dummyAngularVelocity7)
        inducedVelocity11, _, _ = getFinVelWithoutRoll(finSet, self.dummyAngularVelocity8)
        
        # Check results
        self.almostEqualVectors(inducedVelocity1, Vector(0,0,0))
        self.almostEqualVectors(inducedVelocity3, Vector(0,-0.762,0))
        self.almostEqualVectors(inducedVelocity5, Vector(0,0.762,0))
        self.almostEqualVectors(inducedVelocity7, Vector(0.762,0,0))
        self.almostEqualVectors(inducedVelocity9, Vector(-0.762,0,0))
        self.almostEqualVectors(inducedVelocity11, Vector(0.762,-0.762,0))

    def getFinSliceArgs(self, vel, orientation, angVel, fin1, finSlicePosition=0):
        rocketState = RigidBodyState(Vector(0,0,0), vel, orientation, angVel)
        CG = self.rocket2.getCG(0, rocketState)

        rocketVelocity = AeroParameters.getLocalFrameAirVel(rocketState, self.currentConditions)

        # Add fin velocities due to motion of the rocket
        velocityDueToRocketPitchYaw = rocketState.angularVelocity.crossProduct(fin1.position - CG)*(-1) #The negative puts it in the wind frame

        #We need to find the angle between the rocket velocity vector and the plane described by the fin deflection angle 
        #and the shaft normal
        finNormal = fin1.spanwiseDirection.crossProduct(Vector(0, 0, 1))
        finDeflectionAngle = fin1.finAngle # Adjusted by parent finset during each timestep
        if(finDeflectionAngle != 0):
            rotation = Quaternion(axisOfRotation = fin1.spanwiseDirection, angle=math.radians(finDeflectionAngle))
            finNormal = rotation.rotate(finNormal)

        finUnitSpanTangentialVelocity = rocketState.angularVelocity.crossProduct(fin1.spanwiseDirection)*(-1)
        finVelWithoutRoll = rocketVelocity + velocityDueToRocketPitchYaw

        return finSlicePosition, finVelWithoutRoll, finUnitSpanTangentialVelocity, finNormal, fin1.spanwiseDirection, fin1.stallAngle

    def test_getFinSliceAngleOfAttack(self):
        finSet = self.finSet1
        fin1, fin2, fin3, fin4 = finSet.finList

        # Tilted about a single axis
        AoA1 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation1, self.dummyAngularVelocity1, fin1) )
        AoA2 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation1, self.dummyAngularVelocity1, fin2) )
        AoA3 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation1, self.dummyAngularVelocity1, fin3) )
        AoA4 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation1, self.dummyAngularVelocity1, fin4) )
        self.assertAlmostEqual(AoA1, radians(2))
        self.assertAlmostEqual(AoA2, radians(0))
        self.assertAlmostEqual(AoA3, radians(-2))
        self.assertAlmostEqual(AoA4, radians(0))

        # Tilted about a single axis
        AoA5 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation2, self.dummyAngularVelocity1, fin1) )
        AoA6 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation2, self.dummyAngularVelocity1, fin2) )
        AoA7 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation2, self.dummyAngularVelocity1, fin3) )
        AoA8 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation2, self.dummyAngularVelocity1, fin4) )
        self.assertAlmostEqual(AoA5, radians(-2))
        self.assertAlmostEqual(AoA6, radians(0))
        self.assertAlmostEqual(AoA7, radians(2))
        self.assertAlmostEqual(AoA8, radians(0))
        
        # Vel = Vector(0, 0, 50), orientation = Quaternion(Vector(1, 1, 0), math.radians(2)), AngVel = AngularVelocity(rotationVector = Vector(0,0,0))
        # Case tilted about diagonal axis by 2 degrees
        AoA9 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation5, self.dummyAngularVelocity1, fin1) )
        AoA10 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation5, self.dummyAngularVelocity1, fin2) )
        AoA11 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation5, self.dummyAngularVelocity1, fin3) )
        AoA12 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation5, self.dummyAngularVelocity1, fin4) )
        self.assertAlmostEqual(AoA9, radians(1.414069933721))
        self.assertAlmostEqual(AoA10, radians(1.414069933721))
        self.assertAlmostEqual(AoA11, radians(-1.414069933721))
        self.assertAlmostEqual(AoA12, radians(-1.414069933721))

        #Dummy angular velocity 2 has a 1 rad /sec rotation about z
        # Checking effect of tangential/roll velocity
        AoA13 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation4, self.dummyAngularVelocity2, fin1, 0.0762 ) )
        AoA14 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation4, self.dummyAngularVelocity2, fin2, 0.0762 ) )
        AoA15 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation1, self.dummyAngularVelocity2, fin1, 0.0762 ) )
        AoA16 = getFinSliceAngleOfAttack( *self.getFinSliceArgs(self.dummyVelocity1, self.dummyOrientation1, self.dummyAngularVelocity2, fin2, 0.0762 ) )        
        self.assertAlmostEqual(AoA13, radians(0.08731870037654005))
        self.assertAlmostEqual(AoA14, radians(0.08731870037654005))
        self.assertAlmostEqual(AoA15, radians(2.0872608))
        self.assertAlmostEqual(AoA16, radians(0.08731870037654005))

    # Saving test data for now
        # def test_getFinNormalAeroForceDirection(self):
        #     finSet = self.rocket2.stages[0].components[2]
        #     fin1 = finSet.finList[0]
        #     fin2 = finSet.finList[1]
        #     fin3 = finSet.finList[2]
        #     fin4 = finSet.finList[3]
        #     forceDirectionVector1 = fin1.getFinNormalAeroForceDirection(0)
        #     correctForceDirectionVector1 = Vector(0,-1,0)
        #     forceDirectionVector2 = fin2.getFinNormalAeroForceDirection(0)
        #     correctForceDirectionVector2 = Vector(1,0,0)
        #     forceDirectionVector3 = fin3.getFinNormalAeroForceDirection(0)
        #     correctForceDirectionVector3 = Vector(0,1,0)
        #     forceDirectionVector4 = fin4.getFinNormalAeroForceDirection(0)
        #     correctForceDirectionVector4 = Vector(-1,0,0)
        #     forceDirectionVector5 = fin1.getFinNormalAeroForceDirection(2)
        #     correctForceDirectionVector5 = Vector(0,-0.999390827,-0.034899496)
        #     forceDirectionVector6 = fin2.getFinNormalAeroForceDirection(2)
        #     correctForceDirectionVector6 = Vector(0.999390827,0,-0.034899496)
        #     forceDirectionVector7 = fin3.getFinNormalAeroForceDirection(2)
        #     correctForceDirectionVector7 = Vector(0,0.999390827,-0.034899496)
        #     forceDirectionVector8 = fin4.getFinNormalAeroForceDirection(2)
        #     correctForceDirectionVector8 = Vector(-0.999390827,0,-0.034899496)
        #     forceDirectionVector9 = fin1.getFinNormalAeroForceDirection(-10)
        #     correctForceDirectionVector9 = Vector(0,-0.98480775,0.1736482)
        #     forceDirectionVector10 = fin2.getFinNormalAeroForceDirection(-10)
        #     correctForceDirectionVector10 = Vector(0.98480775,0,0.1736482)
        #     forceDirectionVector11 = fin3.getFinNormalAeroForceDirection(-10)
        #     correctForceDirectionVector11 = Vector(0,0.98480775,0.1736482)
        #     forceDirectionVector12 = fin4.getFinNormalAeroForceDirection(-10)
        #     correctForceDirectionVector12 = Vector(-0.98480775,0,0.1736482)
        #     self.almostEqualVectors(forceDirectionVector1, correctForceDirectionVector1)
        #     self.almostEqualVectors(forceDirectionVector2, correctForceDirectionVector2)
        #     self.almostEqualVectors(forceDirectionVector3, correctForceDirectionVector3)
        #     self.almostEqualVectors(forceDirectionVector4, correctForceDirectionVector4)
        #     self.almostEqualVectors(forceDirectionVector5, correctForceDirectionVector5)
        #     self.almostEqualVectors(forceDirectionVector6, correctForceDirectionVector6)
        #     self.almostEqualVectors(forceDirectionVector7, correctForceDirectionVector7)
        #     self.almostEqualVectors(forceDirectionVector8, correctForceDirectionVector8)
        #     self.almostEqualVectors(forceDirectionVector9, correctForceDirectionVector9)
        #     self.almostEqualVectors(forceDirectionVector10, correctForceDirectionVector10)
        #     self.almostEqualVectors(forceDirectionVector11, correctForceDirectionVector11)
        #     self.almostEqualVectors(forceDirectionVector12, correctForceDirectionVector12)

    def test_splitFinIntoSlices(self):
        # Check that slice areas add up to total fin area
        areaSum = sum(self.finSet1.spanSliceAreas)
        self.assertAlmostEqual(areaSum, self.finSet1.planformArea)

        areaSum2 = sum(self.finSet2.spanSliceAreas)
        self.assertAlmostEqual(areaSum2, self.finSet2.planformArea)

    def test_getChord(self):
        # Check root chord, tip chord, and halfway
        self.assertAlmostEqual(self.finSet1.getChord(0), self.finSet1.rootChord)
        self.assertAlmostEqual(self.finSet1.getChord(self.finSet1.span), self.finSet1.tipChord)
        self.assertAlmostEqual(self.finSet1.getChord(self.finSet1.span/2), (self.finSet1.tipChord+self.finSet1.rootChord)/2)

    def test_interferenceFactors(self):
        expectedBodyOnFinInterferenceFactor = 1 + 0.0762/(0.075 + 0.0762)
        self.assertAlmostEqual(expectedBodyOnFinInterferenceFactor, self.finSet1.bodyOnFinInterferenceFactor)

        expectedFinNumberInterferenceFactor = 1.0
        self.assertAlmostEqual(expectedFinNumberInterferenceFactor, self.finSet1.finNumberInterferenceFactor)

    def test_planformArea(self):
        self.assertAlmostEqual(self.finSet1.planformArea, 0.005625)
        self.assertAlmostEqual(self.finSet2.planformArea, 0.03193542)

    def test_trailingEdgeSweep(self):
        # Forward swept trailing edge have negative sweep angles
        self.assertAlmostEqual(self.finSet1.trailingEdgeSweep, -math.radians(5.103909), 4)
        self.assertAlmostEqual(self.finSet2.trailingEdgeSweep, -math.radians(28.610459), 4)

    def test_MACLength(self):
        # Analytical MAC calculation for trapezoidal wing: https://www.airfieldmodels.com/information_source/math_and_science_of_model_aircraft/formulas/mean_aerodynamic_chord.htm
        def analyticalMAC(rc, t):
            return rc * (2/3) * ((1 + t + t**2)) / (1 + t)

        rc1 = self.finSet1.rootChord # Root Chord
        t1 = self.finSet1.tipChord / self.finSet1.rootChord # Taper Ratio
        self.assertAlmostEqual(self.finSet1.MACLength, analyticalMAC(rc1, t1), 3)

        rc2 = self.finSet2.rootChord
        t2 = self.finSet2.tipChord / self.finSet2.rootChord # Taper Ratio
        self.assertAlmostEqual(self.finSet2.MACLength, analyticalMAC(rc2, t2), 4)

    def test_MACYPos(self):
        def analyticalMACY(rootChord, tipChord, span):
            # https://aviation.stackexchange.com/questions/36978/where-is-the-lateral-position-of-the-aerodynamic-center
            # Trapezoidal fins only
            return (span/3) * (rootChord + 2*tipChord)/(rootChord + tipChord)

        r, t, s = self.finSet1.rootChord, self.finSet1.tipChord, self.finSet1.span
        self.assertAlmostEqual(self.finSet1.MACYPos, analyticalMACY(r, t, s), 4)

        r, t, s = self.finSet2.rootChord, self.finSet2.tipChord, self.finSet2.span
        self.assertAlmostEqual(self.finSet2.MACYPos, analyticalMACY(r, t, s), 4)

    def test_XMACLeadingEdge(self):
        def analyticalXMAC(rootChord, tipChord, span, leadingEdgeSweep, area):
            # Manual integration
            c1 = math.tan(leadingEdgeSweep) / area
            term1 = (tipChord - rootChord)*span**3 / (span*3)
            term2 = rootChord * span**2 / 2
            return c1*(term1 + term2)

        r, t, s, LES, a = self.finSet1.rootChord, self.finSet1.tipChord, self.finSet1.span, self.finSet1.sweepAngle, self.finSet1.planformArea
        self.assertAlmostEqual(self.finSet1.XMACLeadingEdge, analyticalXMAC(r, t, s, LES, a), 4)

        r, t, s, LES, a = self.finSet2.rootChord, self.finSet2.tipChord, self.finSet2.span, self.finSet2.sweepAngle, self.finSet2.planformArea
        self.assertAlmostEqual(self.finSet2.XMACLeadingEdge, analyticalXMAC(r, t, s, LES, a), 4)

    def test_getCP(self):
        fins = self.finSet2
        
        def analyticalMAC(rc, taperRatio):
            return rc * (2/3) * ((1 + taperRatio + taperRatio**2)) / (1 + taperRatio)

        def analyticalXMAC(rootChord, tipChord, span, leadingEdgeSweep, area):
            # Manual integration
            c1 = math.tan(leadingEdgeSweep) / area
            term1 = (tipChord - rootChord)*span**3 / (span*3)
            term2 = rootChord * span**2 / 2
            return c1*(term1 + term2)

        def planformArea(rootChord, tipChord, span):
            return (rootChord + tipChord) * span/2

        rootChord = 0.3048
        tipChord = 0.1524
        span = 0.1397
        taperRatio = tipChord/rootChord
        pArea = planformArea(rootChord, tipChord, span)
        LESweep = math.radians(28.61)
        MACLength = analyticalMAC(rootChord, taperRatio)
        XMAC = analyticalXMAC(rootChord, tipChord, span, LESweep, pArea)

        subsonic = fins._getCPXPos(0.2)
        analyticalSubsonic = XMAC + 0.25*MACLength
        self.assertAlmostEqual(subsonic, analyticalSubsonic)

        # Check that amounts line up at interpolation boundaries
        # Check that amounts line up at M = 0.5
        pre = fins._getCPXPos(0.49999999999)
        post = fins._getCPXPos(0.50000000001)
        self.assertAlmostEqual(pre, post)

        # Check that amounts line up at M = 2
        pre = fins._getCPXPos(1.99999999999)
        post = fins._getCPXPos(2.00000000001)
        self.assertAlmostEqual(pre, post)

        supersonic = fins._getCPXPos(5)
        analyticalSupersonic = XMAC + 0.5*MACLength
        self.assertAlmostEqual(supersonic, analyticalSupersonic, 2)

    # def test_finOpenRocketAeroCoefficients(self):
    #     #Subsonic cases, fin deflections and angles of attack checks

    #     #rocket State 2 has a positive AoA about the x-axis
    #     #rocket state 9 has a negative AoA about the x-axis
    #     #rocket state 1 has no AoA
    #     #rocket state 11 has a positive AoA about the y-axis
    #     #rocket state 12 has a negative AoA about the y-axis

    #     finSet = self.finSet2
    #     setUpAeroFunc = finSet.getAppliedForce
    #     aeroFunc1 = finSet.finList[0]._barrowmanAeroFunc
    #     aeroFunc3 = finSet.finList[1]._barrowmanAeroFunc
    #     time = 10

    #     setUpAeroFunc(self.rocketState2, time, self.currentConditions, self.rocket.getCG(0, self.rocketState2))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState2, self.currentConditions, self.rocket.getCG(0, self.rocketState2))
    #     [aeroForce1, globalCP] = aeroFunc1(self.rocketState2, time, self.currentConditions, preComp, CG=self.rocket.getCG(0, self.rocketState2), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState9, time, self.currentConditions, self.rocket.getCG(0, self.rocketState9))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState9, self.currentConditions, self.rocket.getCG(0, self.rocketState9))
    #     [aeroForce2, globalCP] = aeroFunc1(self.rocketState9, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState9), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState1, time, self.currentConditions, self.rocket.getCG(0, self.rocketState1))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState1, self.currentConditions, self.rocket.getCG(0, self.rocketState1))
    #     [aeroForce3, globalCP] = aeroFunc1(self.rocketState1, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState1), finDeflectionAngle=-2)
    #     setUpAeroFunc(self.rocketState11, time, self.currentConditions, self.rocket.getCG(0, self.rocketState11))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState11, self.currentConditions, self.rocket.getCG(0, self.rocketState11))
    #     [aeroForce7, globalCP] = aeroFunc3(self.rocketState11, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState11), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState12, time, self.currentConditions, self.rocket.getCG(0, self.rocketState12))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState12, self.currentConditions, self.rocket.getCG(0, self.rocketState12))
    #     [aeroForce8, globalCP] = aeroFunc3(self.rocketState12, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState12), finDeflectionAngle=0)

    #     # Grab new aero function, this time for the second fin
    #     aeroFunc2 = finSet.finList[1]._barrowmanAeroFunc
    #     setUpAeroFunc(self.rocketState2, time, self.currentConditions, self.rocket.getCG(0, self.rocketState2))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState2, self.currentConditions, self.rocket.getCG(0, self.rocketState2))
    #     [aeroForce4, globalCP] = aeroFunc2(self.rocketState2, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState2), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState9, time, self.currentConditions, self.rocket.getCG(0, self.rocketState9))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState9, self.currentConditions, self.rocket.getCG(0, self.rocketState9))
    #     [aeroForce5, globalCP] = aeroFunc2(self.rocketState9, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState9), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState1, time, self.currentConditions, self.rocket.getCG(0, self.rocketState1))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState1, self.currentConditions, self.rocket.getCG(0, self.rocketState1))
    #     [aeroForce6, globalCP] = aeroFunc2(self.rocketState1, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState1), finDeflectionAngle=-2)

    #     self.almostEqualVectors(aeroForce1.force, Vector(0,-136.66897,-13.679302),2)
    #     self.almostEqualVectors(aeroForce2.force, Vector(0,136.66897,-13.679302),2)
    #     self.almostEqualVectors(aeroForce3.force, Vector(0,136.10832,-18.4406479),2)
    #     self.almostEqualVectors(aeroForce4.force, Vector(0,0,-13.5240509), 4)
    #     self.almostEqualVectors(aeroForce5.force, Vector(0,0,-13.5240509), 4)
    #     self.almostEqualVectors(aeroForce6.force, Vector(-136.58572,0,-18.440647),0)
    #     self.almostEqualVectors(aeroForce7.force, Vector(136.66897,0,-13.67930),0)
    #     self.almostEqualVectors(aeroForce8.force, Vector(-136.66897,0,-13.679302),0)

    #     #Subsonic cases, rocket state 10 has a roll velocity about z of 1 rad/sec with no angle of attack
    #     aeroFunc1 = finSet.finList[0]._barrowmanAeroFunc
    #     setUpAeroFunc(self.rocketState10, time, self.currentConditions, self.rocket.getCG(0, self.rocketState10))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState10, self.currentConditions, self.rocket.getCG(0, self.rocketState10))
    #     [aeroForce1, globalCP] = aeroFunc1(self.rocketState10, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState10), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState10, time, self.currentConditions, self.rocket.getCG(0, self.rocketState10))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState10, self.currentConditions, self.rocket.getCG(0, self.rocketState10))
    #     [aeroForce2, globalCP] = aeroFunc1(self.rocketState10, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState10), finDeflectionAngle=-2)
    #     setUpAeroFunc(self.rocketState10, time, self.currentConditions, self.rocket.getCG(0, self.rocketState10))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState10, self.currentConditions, self.rocket.getCG(0, self.rocketState10))
    #     [aeroForce3, globalCP] = aeroFunc1(self.rocketState10, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState10), finDeflectionAngle=2)
    #     self.almostEqualVectors(aeroForce1.force, Vector(0,-2.65997761,-13.5242116),0)
    #     self.almostEqualVectors(aeroForce2.force, Vector(0,133.450280,-18.338751),0)
    #     self.almostEqualVectors(aeroForce3.force, Vector(0,-138.76635,-18.54279078),0)

    #     #Supersonic cases
    #     #Compares to Spyder script pretty well
    #     finSet2 = self.finSet1
    #     aeroFunc2 = finSet2.finList[0]._barrowmanAeroFunc
    #     setUpAeroFunc2 = finSet2.getAppliedForce
    #     aeroFunc1 = finSet.finList[0]._barrowmanAeroFunc
    #     setUpAeroFunc(self.rocketState3, time, self.currentConditions, self.rocket.getCG(0, self.rocketState3))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState3, self.currentConditions, self.rocket.getCG(0, self.rocketState3))
    #     [aeroForce1, globalCP1] = aeroFunc1(self.rocketState3, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState3), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState3, time, self.currentConditions, self.rocket.getCG(0, self.rocketState3))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState3, self.currentConditions, self.rocket.getCG(0, self.rocketState3))
    #     [aeroForce2, globalCP2] = aeroFunc1(self.rocketState3, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState3), finDeflectionAngle=-2)
    #     setUpAeroFunc(self.rocketState3, time, self.currentConditions, self.rocket.getCG(0, self.rocketState3))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState3, self.currentConditions, self.rocket.getCG(0, self.rocketState3))
    #     [aeroForce3, globalCP3] = aeroFunc1(self.rocketState3, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState3), finDeflectionAngle=2)
    #     setUpAeroFunc(self.rocketState13, time, self.currentConditions, self.rocket.getCG(0, self.rocketState13))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState13, self.currentConditions, self.rocket.getCG(0, self.rocketState13))
    #     [aeroForce4, globalCP4] = aeroFunc1(self.rocketState13, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState13), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState14, time, self.currentConditions, self.rocket.getCG(0, self.rocketState14))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState14, self.currentConditions, self.rocket.getCG(0, self.rocketState14))
    #     [aeroForce5, globalCP5] = aeroFunc1(self.rocketState14, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState14), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState15, time, self.currentConditions, self.rocket.getCG(0, self.rocketState15))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState15, self.currentConditions, self.rocket.getCG(0, self.rocketState15))
    #     [aeroForce6, globalCP6] = aeroFunc1(self.rocketState15, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState15), finDeflectionAngle=0)
    #     setUpAeroFunc2(self.rocketState3, time, self.currentConditions, self.rocket.getCG(0, self.rocketState3))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState3, self.currentConditions, self.rocket.getCG(0, self.rocketState3))
    #     [aeroForce7, globalCP7] = aeroFunc2(self.rocketState3, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState3), finDeflectionAngle=0)
    #     self.almostEqualVectors(aeroForce1.force, Vector(0,-510.5234148,-113.96738),0)
    #     self.almostEqualVectors(aeroForce2.force, Vector(0,-3.932263,-112.60528),0)
    #     self.almostEqualVectors(aeroForce3.force, Vector(0,-1050.9578,-154.17925),0)
    #     self.almostEqualVectors(aeroForce4.force, Vector(0,-1055.69838,-117.40743),0)
    #     self.almostEqualVectors(aeroForce5.force, Vector(0,-1647.4697,-122.33358),0)
    #     self.almostEqualVectors(aeroForce6.force, Vector(0,-2297.7823,-128.0853),0)
    #     self.almostEqualVectors(globalCP1, Vector(0.154036,0,-2.9352),0)
    #     self.almostEqualVectors(globalCP6, Vector(0.154036,0,-2.9352),0)
    #     self.almostEqualVectors(globalCP7, Vector(0.111796,0,-0.82028539),0)

    #     #Transonic cpcases
    #     aeroFunc1 = finSet.finList[0]._barrowmanAeroFunc
    #     setUpAeroFunc(self.rocketState1, time, self.currentConditions, self.rocket.getCG(0, self.rocketState1))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState1, self.currentConditions, self.rocket.getCG(0, self.rocketState1))
    #     [aeroForce1, globalCP1] = aeroFunc1(self.rocketState1, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState1), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState16, time, self.currentConditions, self.rocket.getCG(0, self.rocketState16))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState16, self.currentConditions, self.rocket.getCG(0, self.rocketState16))
    #     [aeroForce2, globalCP2] = aeroFunc1(self.rocketState16, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState16), finDeflectionAngle=0)
    #     setUpAeroFunc(self.rocketState15, time, self.currentConditions, self.rocket.getCG(0, self.rocketState15))
    #     preComp = finSet._getPreComputedFinAeroData(self.rocketState15, self.currentConditions, self.rocket.getCG(0, self.rocketState15))
    #     [aeroForce3, globalCP3] = aeroFunc1(self.rocketState15, time, self.currentConditions, preComp, self.rocket.getCG(0, self.rocketState15), finDeflectionAngle=0)
    #     self.almostEqualVectors(globalCP1, Vector(0.15404,0,-2.885),2)
    #     self.almostEqualVectors(globalCP2, Vector(0.15404,0,-2.913),2)
    #     self.almostEqualVectors(globalCP3, Vector(0.15404,0,-2.935),2)

    #### Utilities ####
    def almostEqualVectors(self, Vector1, Vector2, n=7):
        self.assertAlmostEqual(Vector1.X, Vector2.X, n)
        self.assertAlmostEqual(Vector1.Y, Vector2.Y, n)
        self.assertAlmostEqual(Vector1.Z, Vector2.Z, n)

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
