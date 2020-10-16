#Created by: Declan Quinn
#Jan 2020

#To run tests:
#In this file: [test_StandardAtmosphere.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import unittest

from MAPLEAF.Main import Simulation
from MAPLEAF.Motion import (AngularVelocity, Quaternion, RigidBodyState,
                            RigidBodyState_3DoF, Vector)


class TestRecoverySystem(unittest.TestCase):
    def setUp(self):
        # Init rocket + environment
        simRunner = Simulation(simDefinitionFilePath="MAPLEAF/Examples/Simulations/Recovery.mapleaf", silent=True)
        self.recoveryRocket = simRunner.createRocket()

        self.rSys = self.recoveryRocket.recoverySystem
        self.environment = self.recoveryRocket.environment

        velDown = Vector(0, 0, -100)
        velUp = Vector(0, 0, 100)
        pos = Vector(0, 0, 500)
        orientation = Quaternion(axisOfRotation=Vector(0,0,1), angle=0)
        angVel = AngularVelocity(rotationVector=Vector(0,0,0))
        
        self.descendingState = RigidBodyState(pos, velDown, orientation, angVel)
        self.descendingState_3DoF = RigidBodyState_3DoF(pos, velDown)
        self.descendingTime = 2

        self.ascendingState = RigidBodyState(pos, velUp, orientation, angVel)
        self.ascendingState_3DoF = RigidBodyState_3DoF(pos, velUp)
        self.ascendingTime = 2

    def test_RecoverySystemInit(self):
        self.assertNotEqual(self.recoveryRocket.recoverySystem, None)

        rSys = self.recoveryRocket.recoverySystem

        self.assertEqual(rSys.numStages, 3)
        self.assertEqual(rSys.stageTriggers, [ None, "Apogee", "Altitude", "Time" ])
        self.assertEqual(rSys.stageTriggerValues, [ None, None, 300, 301 ])
        self.assertEqual(rSys.chuteAreas, [ 0, 2, 9, 15 ])
        self.assertEqual(rSys.chuteCds, [ 0, 1.5, 1.51, 1.52])
        self.assertEqual(rSys.delayTimes, [ 0, 2, 0, 0 ])
        self.assertEqual(rSys.currentStage, 0)
        self.assertEqual(rSys.nextStageDeployTime, None)

    def test_Aero(self):
        #3DoF test
        self.rSys._deployNextStage()
        self.rSys._deployNextStage()
        self.rSys._deployNextStage()
        testState = self.descendingState
        airProps = self.environment.getAirProperties(Vector(0,0,0))

        expectedForce = Vector(0, 0, 1.52 * 100**2 * airProps.Density * 15 / 2)
        calculatedForce = self.rSys.getAppliedForce(testState, self.descendingTime, airProps, Vector(0,0,0))
        self.almostEqualVectors(calculatedForce.force, expectedForce)

    def test_ApogeeDeployment(self):
        # Get forces during ascent, make sure chute does not deploy
        self.recoveryRocket.rigidBody.time = self.descendingTime
        self.recoveryRocket.rigidBody.state = self.ascendingState
        self.recoveryRocket.timeStep(0.01)
        # Check that recovery stage has not deployed
        self.assertEqual(self.rSys.currentStage, 0)
        # And is not scheduled to deploy
        self.assertEqual(self.recoveryRocket.simEventDetector.conditionValues[0], None) 

        # Run take timestep to recognize Apogee
        self.recoveryRocket.rigidBody.time = self.descendingTime
        self.recoveryRocket.rigidBody.state = self.descendingState
        self.recoveryRocket.timeStep(0.01)
        # Run again, after delayTime to deploy next stage
        self.recoveryRocket.rigidBody.time += 2
        self.recoveryRocket.timeStep(0.01)
        self.recoveryRocket.simEventDetector.triggerEvents()
        self.assertEqual(self.rSys.currentStage, 1)

    def test_AltitudeDeployment(self):
        # Deploy first stage to get second stage ready
        self.rSys._deployNextStage() # This also switches to 3DoF mode

        # Remove event detector effects of first recovery stage (being skipped)
        self.recoveryRocket.simEventDetector.callbackFunctions = [ self.recoveryRocket.simEventDetector.callbackFunctions[1] ]
        self.recoveryRocket.simEventDetector.conditionsEvalFunctions = [ self.recoveryRocket.simEventDetector.conditionsEvalFunctions[1] ]
        self.recoveryRocket.simEventDetector.conditionValues = [ self.recoveryRocket.simEventDetector.conditionValues[1] ]
        self.recoveryRocket.simEventDetector.triggerDelays = [ self.recoveryRocket.simEventDetector.triggerDelays[1] ]

        # Get forces during ascent, make sure chute does not deploy
        self.ascendingState_3DoF.position = Vector(0,0,300)
        self.recoveryRocket.rigidBody.state = self.ascendingState_3DoF
        self.recoveryRocket.rigidBody.time = self.descendingTime
        self.recoveryRocket.timeStep(0.01)
        # Check that recovery stage has not deployed
        self.assertEqual(self.rSys.currentStage, 1)

        # Get forces during descent, but too high up, make sure chute does not deploy
        self.descendingState_3DoF.position = Vector(0,0,302)
        self.recoveryRocket.rigidBody.state = self.descendingState_3DoF
        self.recoveryRocket.rigidBody.time = self.descendingTime
        self.recoveryRocket.timeStep(0.01)
        # Check that recovery stage has deployed
        self.assertEqual(self.rSys.currentStage, 1)

        # Should now deploy
        self.descendingState_3DoF.position = Vector(0,0,300)
        self.recoveryRocket.rigidBody.state = self.descendingState_3DoF
        self.recoveryRocket.rigidBody.time = self.descendingTime
        self.recoveryRocket.timeStep(0.01)
        # Check that recovery stage has deployed
        self.assertEqual(self.rSys.currentStage, 2)

    def test_TimeDeployment(self):
        # Deploy first/second stages to get third stage ready
        self.rSys._deployNextStage() # Also switches to 3DoF
        self.rSys._deployNextStage()

        self.recoveryRocket.simEventDetector.callbackFunctions = [ self.recoveryRocket.simEventDetector.callbackFunctions[2] ]
        self.recoveryRocket.simEventDetector.conditionsEvalFunctions = [ self.recoveryRocket.simEventDetector.conditionsEvalFunctions[2] ]
        self.recoveryRocket.simEventDetector.conditionValues = [ self.recoveryRocket.simEventDetector.conditionValues[2] ]
        self.recoveryRocket.simEventDetector.triggerDelays = [ self.recoveryRocket.simEventDetector.triggerDelays[2] ]

        # Make sure chute does not deploy early
        self.recoveryRocket.rigidBody.time = 300
        self.recoveryRocket.rigidBody.state = self.descendingState_3DoF
        self.recoveryRocket.timeStep(0.01)
        # Check that recovery stage has not deployed
        self.assertEqual(self.rSys.currentStage, 2)

        # Run getAppliedForce once to recognize Altitude
        self.recoveryRocket.rigidBody.time = 301
        self.recoveryRocket.rigidBody.state = self.descendingState_3DoF
        self.recoveryRocket.timeStep(0.01)
        # Check that stage has deployed immediately (no delay)
        self.assertEqual(self.rSys.currentStage, 3)

    #### Utilities ####
    def almostEqualVectors(self, Vector1, Vector2, n=7):
        self.assertAlmostEqual(Vector1.X, Vector2.X, n)
        self.assertAlmostEqual(Vector1.Y, Vector2.Y, n)
        self.assertAlmostEqual(Vector1.Z, Vector2.Z, n)

#If this file is run by itself, run the tests above
if __name__ == '__main__':
    unittest.main()
