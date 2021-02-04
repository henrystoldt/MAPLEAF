import unittest

import numpy as np

from MAPLEAF.IO import SimDefinition
from MAPLEAF.Main import Simulation
from MAPLEAF.Motion import AngularVelocity, Quaternion, RigidBodyState, Vector
from MAPLEAF.Motion.Integration import AdaptiveIntegrator, ClassicalIntegrator


class TestRocketControlSystem(unittest.TestCase):
    def setUp(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/Canards.mapleaf", silent=True)

        simDef.setValue("SimControl.loggingLevel", "0")
        simDef.setValue("Rocket.ControlSystem.MomentController.gainTableFilePath", "MAPLEAF/Examples/TabulatedData/testPIDControlLaw.txt")
        simDef.setValue("Rocket.Sustainer.Canards.Actuators.deflectionTablePath","MAPLEAF/Examples/TabulatedData/testFinDeflectionLaw.txt")
        
        simDef.removeKey("Rocket.ControlSystem.FlightPlan.filePath")
        
        simRunner = Simulation(simDefinition=simDef, silent=True)
        self.rocket = simRunner.createRocket()
        
    def test_runControlLoop(self):
        # Basic spin case
        pos = Vector(0,0,0)
        vel = Vector(0,0,0)
        orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=-0.01)

        angularVelocity = AngularVelocity(axisOfRotation=Vector(0,0,1), angularVel=0)
        rigidBodyState = RigidBodyState(pos, vel, orientation, angularVelocity)
        self.rocket.rigidBody.state = rigidBodyState

        expectedAngleError = np.array([ 0, 0, 0.01 ])

        dt = 1
        ExpectedPIDCoeffs = [[ 4, 5, 6 ], [ 4, 5, 6 ], [ 7.5, 8.5, 9.5 ]]

        ExpectedDer = expectedAngleError / dt
        ExpectedIntegral = expectedAngleError * dt / 2

        ExpectedM = []
        for i in range(3):
            moment = ExpectedPIDCoeffs[i][0]*expectedAngleError[i] + ExpectedPIDCoeffs[i][1]*ExpectedIntegral[i] + ExpectedPIDCoeffs[i][2]*ExpectedDer[i]
            ExpectedM.append(moment)

        # Replace keyFunctions with these ones that return fixed values
        def fakeMach(*args):
            # MachNum = 0.1
            return 0.1

        def fakeAltitude(*args):
            # Altitude = 0.5
            return 0.5

        # Fake functions in ScheduledGainPIDMomentController
        self.rocket.controlSystem.momentController.keyFunctionList = [ fakeMach, fakeAltitude ]
        # Fake functions in TableInterpolatingActuatorController
        self.rocket.controlSystem.controlledSystem.actuatorController.keyFunctionList = [ fakeMach, fakeAltitude ]

        # Calculated in spreadsheet: test/linearFinDeflTestDataGenerator.ods
        ExpectedFinDefl = [ 2.1625, 6.225, 10.2875, 14.35 ]
        self.rocket.controlSystem.runControlLoopIfRequired(1, rigidBodyState, 'fakeEnvironment')

        calculatedFinDefl = [ actuator.targetDeflection for actuator in self.rocket.controlSystem.controlledSystem.actuatorList ]

        for i in range(3):
            self.assertAlmostEqual(ExpectedFinDefl[i], calculatedFinDefl[i])

    def test_AddAndRemoveTimeSteppingConstraint(self):
        simDefinition = SimDefinition("MAPLEAF/Examples/Simulations/Canards.mapleaf", silent=True)

        simDefinition.setValue("SimControl.loggingLevel", "0")
        # Set time step incompatible with a fixed control system update rate, and a larger initial time step
        simDefinition.setValue("SimControl.timeDiscretization", "RK45Adaptive")
        simDefinition.setValue("SimControl.timeStep", "0.02")
        simDefinition.setValue("TimeStepAdaptation.controller", "PID")
        # Make the recovery system deploy immediately
        simDefinition.setValue("Rocket.Sustainer.RecoverySystem.stage1Trigger", "Time")
        simDefinition.setValue("Rocket.Sustainer.RecoverySystem.stage1TriggerValue", "0.005")
        simDefinition.setValue("Rocket.Sustainer.RecoverySystem.stage1DelayTime", "0")

        # Set a fixed control system update rate
        simDefinition.setValue("Rocket.ControlSystem.updateRate", "100")

        simRunner = Simulation(simDefinition=simDefinition)
        rocket = simRunner.createRocket()

        # Check that the time step has been changed to be fixed, and is 0.01 seconds
        self.assertTrue(isinstance(rocket.rigidBody.integrate, ClassicalIntegrator)) # As oppopsed to AdaptiveIntegrator
        self.assertEqual(rocket.rigidBody.integrate.method, "RK4") # Check it's been switched from RK45Adaptive

        integrationResult = rocket.timeStep(0.01) # Recovery system should deploy during this time step        
        rocket.simEventDetector.triggerEvents()
            # Time stepping should revert to the original method
        self.assertAlmostEqual(integrationResult.timeStepAdaptationFactor, 1.0)
        self.assertAlmostEqual(integrationResult.dt, 0.01)

        # Check that original time stepping method is back
        self.assertTrue(isinstance(rocket.rigidBody.integrate, AdaptiveIntegrator))
        self.assertEqual(rocket.rigidBody.integrate.method, "RK45Adaptive")

        integrationResult2 = rocket.timeStep(0.01)        
        self.assertTrue(abs(integrationResult2.timeStepAdaptationFactor - 1.0) > 0.00000000000000001)
        self.assertEqual(integrationResult2.dt, 0.01)

        # Time step not currently adjusted back

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
