# Created by: Henry Stoldt
# April 2020

'''
Generalized event detector (Apogee, motor burnout etc...) for a single vehicle.  
Used to trigger stage separations and recovery system deployments.
'''

from enum import Enum
from typing import Tuple

__all__ = [ "EventTypes", "SimEventDetector" ]

class EventTypes(Enum):
    Apogee = "apogee"
    AscendingThroughAltitude = "ascendingThroughAltitude"
    DescendingThroughAltitude = "descendingThroughAltitude"
    MotorBurnout = "motorBurnout"
    TimeReached = "timeReached"

class SimEventDetector():
    '''
        Each sim event detector detects events on a single rocket 
    '''

    def __init__(self, rocket):
        self.rocket = rocket

        self.callbackFunctions = []
        self.conditionsEvalFunctions = []
        self.conditionValues = []
        self.triggerDelays = []

    def subscribeToEvent(self, eventType, callbackFunction, eventTriggerValue=None, triggerDelay=0):
        '''
            Inputs:
                eventType:          "apogee", "ascendingThroughAltitude", "descendingThroughAltitude", 'motorBurnout" or "timeReached"
                    Can pass in a member of the EventType Enum defined above, if desired
                callbackfunction:   function to call when the desired event occurs. Must have no arguments
                eventTriggerValue:  None, or a trigger parameter for the eventType - an altitude or a time
                triggerDelay:       (numeric) the amount of time, in seconds, to wait before triggering an event after a trigger condition has been met

            Note:
                Motor Burnout always triggers at the burnout time of the LOWEST stage's motor!
        '''
        # Define map between eventType strings and checker functions
        stringToEvalFnMap = {
            EventTypes.Apogee.value:                    self._isAfterApogee,
            EventTypes.AscendingThroughAltitude.value:  self._isAboveAltitude,
            EventTypes.DescendingThroughAltitude.value: self._isBelowAltitude,
            EventTypes.MotorBurnout.value:              self._isBottomStageMotorBurnedOut,
            EventTypes.TimeReached.value:               self._timeReached
        }

        # Make sure eventType is a string
        if not isinstance(eventType, str):
            eventType = eventType.value # If for example EventType.Apogee is passed in, EventType.Apogee.value retrieves "apogee" from the EventType enum
        
        self.callbackFunctions.append(callbackFunction)
        self.conditionValues.append(eventTriggerValue)
        self.conditionsEvalFunctions.append(stringToEvalFnMap[eventType])
        self.triggerDelays.append(triggerDelay)

    def triggerEvents(self) -> Tuple[float, bool]:
        ''' Checks if any of the events that this SimEventDetector is supposed to detect have happened. If so, triggers their callback functions.
            Returns the estimated time to the next event, and whether that event happens at a set time (time-deterministic) or not (non-time-deterministic - like an altitude condition)
                These return values are used to influence time step adaptation to accurately resolve discrete event timing
        '''
        # Save indices of conditions that have been triggered, will now be removed
        indicesToRemove = []
        
        estimatedTimeToNextEvent = 1e10
        nextEventTimeDeterministic = False
        
        # Precomputed to pass to sub-functions (required for altitude conditions)
        ENUState = self.rocket.environment.convertStateToENUFrame(self.rocket.rigidBody.state)

        # Check each callback condition
        for i in range(len(self.callbackFunctions)):
            # If a conditions has come true
            eventOccurred, timeToOccurrence, timeDeterministic = self.conditionsEvalFunctions[i](self.conditionValues[i], ENUState)

            # Get info about the next event to occur
            if timeToOccurrence < estimatedTimeToNextEvent and not eventOccurred:
                estimatedTimeToNextEvent = timeToOccurrence
                nextEventTimeDeterministic = timeDeterministic

            if eventOccurred:
                if self.triggerDelays[i] == 0:
                    # Call its function
                    self.callbackFunctions[i]()
                else:
                    # Schedule the event to happen in triggerDelay seconds
                    triggerTime = self.rocket.rigidBody.time + self.triggerDelays[i]
                    self.subscribeToEvent(EventTypes.TimeReached, self.callbackFunctions[i], triggerTime)

                # And mark it for deletion
                indicesToRemove.append(i)

        # Delete all of the events that were triggered - working in reverse to ensure the indices of the items aren't changed before we delete them
        for i in range(len(indicesToRemove)):
            delI = len(indicesToRemove)-1-i
            indexToRemove = indicesToRemove[delI]
            
            del self.callbackFunctions[indexToRemove]
            del self.conditionValues[indexToRemove]
            del self.conditionsEvalFunctions[indexToRemove]
            del self.triggerDelays[indexToRemove]

        
        # Save velocity and time for next timestep (if we're trying to detect apogee)
            # Doing this here to avoid problems calling the apogee function multiple times in a single time step
        self.lastVelocity = ENUState.velocity
        self.lastTime = self.rocket.rigidBody.time

        return estimatedTimeToNextEvent, nextEventTimeDeterministic
    
    ### Event evaluation functions - each is expected to have a single parameter, return True/False ###
    def _isAfterApogee(self, _, ENUState):
        # Time > 1.0 condition here to avoid setting off events if sliding slightly down before engine lights
        eventOccurred = ENUState.velocity.Z <= 0 and self.rocket.rigidBody.time > 1.0

        timeToOccurrence = 1e10
        try:
            accel = (ENUState.velocity.Z - self.lastVelocity.Z) / (self.rocket.rigidBody.time - self.lastTime)
            if accel < 0 and ENUState.velocity.Z > 0:
                timeToOccurrence = -ENUState.velocity.Z / accel
        except (AttributeError, ZeroDivisionError):
            pass # Haven't saved a velocity/time yet / calling function twice in a single time step

        return eventOccurred, timeToOccurrence, False

    def _isAboveAltitude(self, altitude, ENUState):
        eventOccurred = ENUState.position.Z >= altitude and ENUState.velocity.Z > 0

        if ENUState.velocity.Z > 0:
            timeToOccurrence = (altitude - ENUState.position.Z) / ENUState.velocity.Z
        else:
            timeToOccurrence = 1e10

        return eventOccurred, timeToOccurrence, False

    def _isBelowAltitude(self, altitude, ENUState):
        eventOccurred = ENUState.position.Z <= altitude and ENUState.velocity.Z < 0

        if ENUState.velocity.Z < 0:
            timeToOccurrence = (altitude - ENUState.position.Z) / ENUState.velocity.Z
        else:
            timeToOccurrence = 1e10

        return eventOccurred, timeToOccurrence, False

    def _isBottomStageMotorBurnedOut(self, _, ENUState):
        eventOccurred = self.rocket.rigidBody.time >= self.rocket.stages[-1].engineShutOffTime
        timeToOccurrence = self.rocket.stages[-1].engineShutOffTime - self.rocket.rigidBody.time
        return eventOccurred, timeToOccurrence, True

    def _timeReached(self, time, ENUState):
        eventOccurred = self.rocket.rigidBody.time >= time
        timeToOccurrence = time - self.rocket.rigidBody.time
        return eventOccurred, timeToOccurrence, True
