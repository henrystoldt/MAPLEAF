
#Created by: Declan Quinn
#May 2019

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import math
import unittest

from MAPLEAF.ENV import Environment
from MAPLEAF.IO import SimDefinition, SubDictReader
from MAPLEAF.Motion import AngularVelocity, Quaternion, RigidBodyState, Vector
from MAPLEAF.Rocket import NoseCone, Rocket


class TestNosecone(unittest.TestCase):
    def setUp(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/test3.mapleaf")
        rocketDictReader = SubDictReader("Rocket", simDef)
        self.rocket = Rocket(rocketDictReader)

        self.environment = Environment(silent=True)
        self.currentConditions = self.environment.getAirProperties(Vector(0,0,200)) # m

        self.rocketState1 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState2 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(1, 0, 0), math.radians(2)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState3 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 500), Quaternion(Vector(1, 0, 0), math.radians(2)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState4 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, -200), Quaternion(Vector(1, 0, 0), math.radians(180)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState5 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, -200), Quaternion(Vector(1, 0, 0), math.radians(178)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState6 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, -200), Quaternion(Vector(1, 0, 0), math.radians(182)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState7 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, -200), Quaternion(Vector(1, 0, 0), math.radians(90)), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        self.rocketState8 = RigidBodyState(Vector(0, 0, 200), Vector(20.04, -0.12, -52.78), Quaternion(Vector(0, 1, 0), math.radians(90)), AngularVelocity(rotationVector=Vector(0, 0, 0)))

    def test_noseconeGetPlanformArea(self):
        nosecone = self.rocket.stages[0].getComponentsOfType(NoseCone)[0]
        self.assertAlmostEqual(nosecone._getPlanformArea(),0.077573818,3) #From solidworks
    
    # def test_noseconeOpenRocketAeroCoefficients(self):
    #     nosecone = self.rocket.stages[0].getComponentsOfType(Nosecone)[0]

    #     aeroForce = nosecone.getAppliedForce(self.rocketState1, 0, self.currentConditions, self.rocket.getCG(0, self.rocketState1))
    #     normalForceDirection = AeroParameters.getNormalAeroForceDirection(self.rocketState1, self.currentConditions)
    #     axialForceDirection = Vector(0, 0, -1) #By definition of axial force
    #     normalForceHandCalc = 0
    #     axialForceHandCalc = 26.832023606232575
    #     CpWRTNoseconeTip = Vector(0, 0, -0.47615415152) #Planform centroid
    #     normalForce = normalForceDirection.__mul__(normalForceHandCalc)
    #     axialForce = axialForceDirection.__mul__(axialForceHandCalc)
    #     appliedNormalForce = ForceMomentSystem(normalForce, CpWRTNoseconeTip)
    #     appliedAxialForce = ForceMomentSystem(axialForce, CpWRTNoseconeTip)
    #     correctAeroForce = appliedNormalForce + appliedAxialForce
        
    #     assertForceMomentSystemsAlmostEqual(self, aeroForce, correctAeroForce)

    #     aeroForce = nosecone.getAppliedForce(self.rocketState2, 0, self.currentConditions, self.rocket.getCG(0, self.rocketState2))
    #     normalForceDirection = AeroParameters.getNormalAeroForceDirection(self.rocketState2, self.currentConditions)
    #     axialForceDirection = Vector(0, 0, -1) #By definition of axial force
    #     normalForceHandCalc = 30.618783938108784
    #     axialForceHandCalc = 27.140046221672183
    #     CpWRTNoseconeTip = Vector(0, 0, -0.47615415152) #Planform centroid
    #     normalForce = normalForceDirection.__mul__(normalForceHandCalc)
    #     axialForce = axialForceDirection.__mul__(axialForceHandCalc)
    #     appliedNormalForce = ForceMomentSystem(normalForce, CpWRTNoseconeTip)
    #     appliedAxialForce = ForceMomentSystem(axialForce, CpWRTNoseconeTip)
    #     correctAeroForce = appliedNormalForce + appliedAxialForce
        
    #     assertForceMomentSystemsAlmostEqual(self, aeroForce, correctAeroForce)

    #     aeroForce = nosecone.getAppliedForce(self.rocketState3, 0, self.currentConditions, self.rocket.getCG(0, self.rocketState3))
    #     normalForceDirection = AeroParameters.getNormalAeroForceDirection(self.rocketState3, self.currentConditions)
    #     axialForceDirection = Vector(0, 0, -1) #By definition of axial force
    #     normalForceHandCalc = 191.3673996131799
    #     axialForceHandCalc = 334.44731852792586
    #     CpWRTNoseconeTip = Vector(0, 0, -0.47615415152) #Planform centroid
    #     normalForce = normalForceDirection.__mul__(normalForceHandCalc)
    #     axialForce = axialForceDirection.__mul__(axialForceHandCalc)
    #     appliedNormalForce = ForceMomentSystem(normalForce, CpWRTNoseconeTip)
    #     appliedAxialForce = ForceMomentSystem(axialForce, CpWRTNoseconeTip)
    #     correctAeroForce = appliedNormalForce + appliedAxialForce
        
    #     assertForceMomentSystemsAlmostEqual(self, aeroForce, correctAeroForce)

    #     aeroForce = nosecone.getAppliedForce(self.rocketState4, 0, self.currentConditions, self.rocket.getCG(0, self.rocketState4))
    #     normalForceDirection = AeroParameters.getNormalAeroForceDirection(self.rocketState4, self.currentConditions)
    #     axialForceDirection = Vector(0, 0, -1) #By definition of axial force
    #     normalForceHandCalc = 0
    #     axialForceHandCalc = 26.832023606232575
    #     CpWRTNoseconeTip = Vector(0, 0, -0.47615415152) #Planform centroid
    #     normalForce = normalForceDirection.__mul__(normalForceHandCalc)
    #     axialForce = axialForceDirection.__mul__(axialForceHandCalc)
    #     appliedNormalForce = ForceMomentSystem(normalForce, CpWRTNoseconeTip)
    #     appliedAxialForce = ForceMomentSystem(axialForce, CpWRTNoseconeTip)
    #     correctAeroForce = appliedNormalForce + appliedAxialForce
        
    #     assertForceMomentSystemsAlmostEqual(self, aeroForce, correctAeroForce)

    #     aeroForce = nosecone.getAppliedForce(self.rocketState5, 0, self.currentConditions, self.rocket.getCG(0, self.rocketState5))
    #     normalForceDirection = AeroParameters.getNormalAeroForceDirection(self.rocketState5, self.currentConditions)
    #     axialForceDirection = Vector(0, 0, -1) #By definition of axial force
    #     normalForceHandCalc = 30.618783938108784
    #     axialForceHandCalc = 27.140046221672183
    #     CpWRTNoseconeTip = Vector(0, 0, -0.47615415152) #Planform centroid
    #     normalForce = normalForceDirection.__mul__(normalForceHandCalc)
    #     axialForce = axialForceDirection.__mul__(axialForceHandCalc)
    #     appliedNormalForce = ForceMomentSystem(normalForce, CpWRTNoseconeTip)
    #     appliedAxialForce = ForceMomentSystem(axialForce, CpWRTNoseconeTip)
    #     correctAeroForce = appliedNormalForce + appliedAxialForce
        
    #     assertForceMomentSystemsAlmostEqual(self, aeroForce, correctAeroForce)

    #     aeroForce = nosecone.getAppliedForce(self.rocketState6, 0, self.currentConditions, self.rocket.getCG(0, self.rocketState6))
    #     normalForceDirection = AeroParameters.getNormalAeroForceDirection(self.rocketState6, self.currentConditions)
    #     axialForceDirection = Vector(0, 0, -1) #By definition of axial force
    #     normalForceHandCalc = 30.618783938108784
    #     axialForceHandCalc = 27.140046221672183
    #     CpWRTNoseconeTip = Vector(0, 0, -0.47615415152) #Planform centroid
    #     normalForce = normalForceDirection.__mul__(normalForceHandCalc)
    #     axialForce = axialForceDirection.__mul__(axialForceHandCalc)
    #     appliedNormalForce = ForceMomentSystem(normalForce, CpWRTNoseconeTip)
    #     appliedAxialForce = ForceMomentSystem(axialForce, CpWRTNoseconeTip)
    #     correctAeroForce = appliedNormalForce + appliedAxialForce
        
    #     assertForceMomentSystemsAlmostEqual(self, aeroForce, correctAeroForce)

    #     aeroForce = nosecone.getAppliedForce(self.rocketState7, 0, self.currentConditions, self.rocket.getCG(0, self.rocketState7))
    #     normalForceDirection = AeroParameters.getNormalAeroForceDirection(self.rocketState7, self.currentConditions)
    #     axialForceDirection = Vector(0, 0, -1) #By definition of axial force
    #     normalForceHandCalc = 877.27104
    #     axialForceHandCalc = 0
    #     CpWRTNoseconeTip = Vector(0, 0, -0.47615415152) #Planform centroid
    #     normalForce = normalForceDirection.__mul__(normalForceHandCalc)
    #     axialForce = axialForceDirection.__mul__(axialForceHandCalc)
    #     appliedNormalForce = ForceMomentSystem(normalForce, CpWRTNoseconeTip)
    #     appliedAxialForce = ForceMomentSystem(axialForce, CpWRTNoseconeTip)
    #     correctAeroForce = appliedNormalForce + appliedAxialForce
        
    #     assertForceMomentSystemsAlmostEqual(self, aeroForce, correctAeroForce, 0)

    #     aeroForce = nosecone.getAppliedForce(self.rocketState8, 0, self.currentConditions, self.rocket.getCG(0, self.rocketState8))
    #     normalForceDirection = AeroParameters.getNormalAeroForceDirection(self.rocketState8, self.currentConditions)
    #     axialForceDirection = Vector(0, 0, -1) #By definition of axial force
    #     normalForceHandCalc = 65.6
    #     axialForceHandCalc = 0
    #     CpWRTNoseconeTip = Vector(0, 0, -0.47615415152) #Planform centroid
    #     normalForce = normalForceDirection.__mul__(normalForceHandCalc)
    #     axialForce = axialForceDirection.__mul__(axialForceHandCalc)
    #     appliedNormalForce = ForceMomentSystem(normalForce, CpWRTNoseconeTip)
    #     appliedAxialForce = ForceMomentSystem(axialForce, CpWRTNoseconeTip)
    #     correctAeroForce = appliedNormalForce + appliedAxialForce
        
    #     assertForceMomentSystemsAlmostEqual(self, aeroForce, correctAeroForce, -1)

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
