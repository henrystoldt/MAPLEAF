from MAPLEAF.Motion import AeroParameters, ForceMomentSystem, Vector
from MAPLEAF.Rocket import EventTypes, FixedMass
from MAPLEAF.Utilities import logForceResult

__pdoc__ = {
    'RecoverySystem._deployNextStage': True
}

__all__ = [ "RecoverySystem" ]

class RecoverySystem(FixedMass):
    ''' Represents a recovery system with an arbitrary number of stages '''

    #### Init Functions ####
    def __init__(self, componentDictReader, rocket, stage):
        FixedMass.__init__(self, componentDictReader, rocket, stage)
        
        self.numStages = componentDictReader.getInt("numStages")

        # Set rocket properties
        rocket.recoverySystem = self

        self._initRecoveryStages()

    def _initRecoveryStages(self):
        self.stageTriggers = [ None, ]
        self.stageTriggerValues = [ None, ]
        self.chuteAreas = [ 0, ]
        self.chuteCds = [ 0, ]
        self.delayTimes = [ 0, ]

        for i in range(1,self.numStages+1):
            self.stageTriggers.append(self.componentDictReader.getString("stage{}Trigger".format(i)))
            try:
                self.stageTriggerValues.append(self.componentDictReader.getFloat("stage{}TriggerValue".format(i)))
            except KeyError:
                self.stageTriggerValues.append(None)
                
            self.chuteAreas.append(self.componentDictReader.getFloat("stage{}ChuteArea".format(i)))
            self.chuteCds.append(self.componentDictReader.getFloat("stage{}Cd".format(i)))
            self.delayTimes.append(self.componentDictReader.getFloat("stage{}DelayTime".format(i)))

        self.currentStage = -1
        self.nextStageDeployTime = None

        # Initialize by deploying stage 0 (no chutes)
        # Sets up trigger function for the real first chute
        self._deployNextStage()

    #### Operational Functions ####
    @logForceResult
    def getAeroForce(self, rocketState, time, environment, CG):
        '''
            Calculates force/moment applied by the recovery system using a simple drag coefficient + area model
        '''
        #### Calculate Aero Force ####
        if self.currentStage == 0:
            # No recovery system deployed yet
            return ForceMomentSystem(Vector(0,0,0))
        else:
            # 3DoF force-only aero
            airVel = AeroParameters.getAirVelRelativeToVehicle(rocketState, environment)
            dragForceMagnitude = self.chuteCds[self.currentStage] * self.chuteAreas[self.currentStage] * AeroParameters.getDynamicPressure(rocketState, environment)
            totalForce = airVel.normalize() * dragForceMagnitude
            return ForceMomentSystem(totalForce)

    def getLogHeader(self):
        return " {}FX(N) {}FY(N) {}FZ(N) {}MX(Nm) {}MY(Nm) {}MZ(Nm)".format(*["Recovery"]*6)

    def _deployNextStage(self):
        ''' 
            Deploys the next stage of the recovery system  
            If appropriate, will set the simulation mode to 3DoF.
            If appropriate, will set up the trigger to deploy the next stage of the recovery system, by calling this function again.
            If this is the final recovery system stage, records the main chute deployment time.
        '''
        self.currentStage += 1
        if self.currentStage > 0:
            print("Deployed Recovery Stage {}".format(self.currentStage))
            
        if self.currentStage == 1:
            self.rocket.isUnderChute = True
            self.rocket._switchTo3DoF()

        if self.currentStage == self.numStages:
            self.rocket.mainChuteDeployTime = self.rocket.rigidBody.time

        if self.currentStage < self.numStages:
            nextTrigger = self.stageTriggers[self.currentStage + 1]
            nextTriggerValue = self.stageTriggerValues[self.currentStage + 1]
            nextTriggerDelay = self.delayTimes[self.currentStage + 1]

            # Translate trigger conditions into language simEventDetector can understand
            if nextTrigger == "Apogee":
                nextTrigger = EventTypes.Apogee
            if nextTrigger == "Altitude":
                nextTrigger = EventTypes.DescendingThroughAltitude
            if nextTrigger == "Time":
                nextTrigger = EventTypes.TimeReached

            # Set the trigger
            self.rocket.simEventDetector.subscribeToEvent(nextTrigger, self._deployNextStage, eventTriggerValue=nextTriggerValue, triggerDelay=nextTriggerDelay)
