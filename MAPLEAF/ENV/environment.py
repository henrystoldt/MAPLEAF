''' Main wrapper and data classes that tie together all of the environmental models and are queried by instances of `MAPLEAF.Rocket.Rocket` '''

import math
from collections import namedtuple
from typing import Union

from MAPLEAF.ENV import (LaunchRail, atmosphericModelFactory,
                         earthModelFactory, meanWindModelFactory,
                         turbulenceModelFactory)
from MAPLEAF.IO import SubDictReader, defaultConfigValues
from MAPLEAF.Motion import (AngularVelocity, ForceMomentSystem, Quaternion,
                            RigidBodyState, RigidBodyState_3DoF, Vector)

__all__ = [ "EnvironmentalConditions", "Environment" ]

# This named tuple is the object used to return information from the Environmental models to the rocket
    # It is subsequently passed to all rocket objects, who use it in their calculations if they have
    # relevant calculations to do (aerodynamics, air-breathing propulsion)
EnvironmentalConditions = namedtuple(
    "EnvironmentalConditions",
    [
        "ASLAltitude",
        "Temp",
        "Pressure",
        "Density",
        "DynamicViscosity",
        "Wind",
        "MeanWind",
        "TurbWind",
    ],
)

class Environment():
    '''
        Class wraps Wind Models, Atmospheric properties models, and earth/gravity models, presenting a 
        single interface for communication with flight vehicles
    '''
    
    def __init__(self, simDefinition=None, silent=False):
        ''' Sets up the Wind, Atmospheric, and Earth models requested in the Sim Definition file '''
        self.launchRail = None

        if simDefinition != None:
            # Whenever we're running a real simulation, should always end up here
            envDictReader = SubDictReader("Environment", simDefinition)

            self.meanWindModel = meanWindModelFactory(simDefinition, silent=silent)
            self.turbulenceModel = turbulenceModelFactory(simDefinition, silent=silent)
            self.atmosphericModel = atmosphericModelFactory(envDictReader=envDictReader)
            self.earthModel = earthModelFactory(envDictReader)

            self.launchSiteElevation = envDictReader.tryGetFloat("LaunchSite.elevation")
            self.launchSiteLatitude = envDictReader.tryGetFloat("LaunchSite.latitude")
            self.launchSiteLongitude = envDictReader.tryGetFloat("LaunchSite.longitude")

            # Check if being launched from a launch rail
            launchRailLength = envDictReader.getFloat("LaunchSite.railLength")
            
            if launchRailLength > 0:
                # Initialize a launch rail, aligned with the rocket's initial direction
                initialRocketPosition_towerFrame = envDictReader.getVector("Rocket.position")

                # Check whether precise initial orientation has been specified
                rotationAxis = envDictReader.tryGetVector("Rocket.rotationAxis", defaultValue=None)
                if rotationAxis != None:
                    rotationAngle = math.radians(envDictReader.getFloat("Rocket.rotationAngle"))
                    initOrientation = Quaternion(rotationAxis, rotationAngle)
                else:
                    # Calculate initial orientation quaternion in launch tower frame
                    initialDirection = self.rocketDictReader.getVector("initialDirection").normalize()
                    angleFromVertical = Vector(0,0,1).angle(initialDirection)
                    rotationAxis = Vector(0,0,1).crossProduct(initialDirection)
                    initOrientation = Quaternion(rotationAxis, angleFromVertical)

                launchTowerState_local = RigidBodyState(position=initialRocketPosition_towerFrame, orientation=initOrientation)
                launchTowerState_global = self.earthModel.convertIntoGlobalFrame(launchTowerState_local, self.launchSiteLatitude, self.launchSiteLongitude)
                towerDirection_global = launchTowerState_global.orientation.rotate(Vector(0, 0, 1))                
                self.launchRail = LaunchRail(launchTowerState_global.position, towerDirection_global, launchRailLength, earthRotationRate=self.earthModel.rotationRate)
                
        else:
            # Construct default environment from default parameters when no sim definition is passed in
            # Need additional default value here in case a SimDefinition is not passed in (otherwise SimDefinition takes care of default values)
            self.meanWindModel = meanWindModelFactory()
            self.turbulenceModel = None
            self.atmosphericModel = atmosphericModelFactory(defaultConfigValues["Environment.AtmosphericPropertiesModel"])
            self.earthModel = earthModelFactory()

            self.launchSiteElevation = float(defaultConfigValues["Environment.LaunchSite.elevation"])
            self.launchSiteLatitude = float(defaultConfigValues["Environment.LaunchSite.latitude"])
            self.launchSiteLongitude = float(defaultConfigValues["Environment.LaunchSite.longitude"])

    def convertInitialStateToGlobalFrame(self, initialStateInLaunchTowerFrame):
        ''' 
            Used to convert the rocket's initial kinematic state from the launch tower frame (in which it is defined) and 
                into the global inertia frame

            Takes a rigid body state defined in the launch tower frame and converts it to the global frame 
            For a flat earth model, this just adjusts for the altitude of the launch site,
            but for a rotating earth model, this modifies every part of the rigid body state:
                1. Position is redefined relative to the center of the earth (Acoording to lat/lon)  
                2. The velocity of earth's rotation is added to the velocity  
                3. The orientation is redefined relative to the global frame  
                4. The angular velocity of the earth is added to the rocket's initial state  

            .. note:: If a launch rail is being used, set the launch-tower-frame angular velocity to zero before doing the conversion
        '''
        # In all cases, first redefine the present state relative to sea level
        initialStateInLaunchTowerFrame.position += Vector(0,0,self.launchSiteElevation)

        # Set launch-tower-frame angular velocity to zero if using a launch rail
        if self.launchRail != None:
            initialStateInLaunchTowerFrame.angularVelocity = AngularVelocity(0,0,0)

        # Call the current earth model's conversion function
        return self.earthModel.convertIntoGlobalFrame(initialStateInLaunchTowerFrame, self.launchSiteLatitude, self.launchSiteLongitude)

    def convertStateToENUFrame(self, globalFrameState: Union[RigidBodyState, RigidBodyState_3DoF]) -> Union[RigidBodyState, RigidBodyState_3DoF]:
        altitude = self.earthModel.getAltitude(*globalFrameState.position)
        position = Vector(0, 0, altitude) # Frame moves with the aircraft so x/y are always zero
        
        inertialToENURotation = self.earthModel.getInertialToENUFrameRotation(*globalFrameState.position)
        ENUToGlobalRotation = inertialToENURotation.conjugate()
        velocity = ENUToGlobalRotation.rotate(globalFrameState.velocity)

        try:
            orientation = ENUToGlobalRotation * globalFrameState.orientation
            angVel = ENUToGlobalRotation.rotate(globalFrameState.angularVelocity)
            return RigidBodyState(position, velocity, orientation, angVel)
        except:
            return RigidBodyState_3DoF(position, velocity)

    #### Get all air/atmospheric properties ####
    def getAirProperties(self, position: Vector, time=None) -> EnvironmentalConditions:
        ''' Pass in a vector representing the aircraft's position in the global inertial frame of reference '''
        # ASL Altitude is calculated differently depending on whether the earth is modelled as flat, round, or ellipsoidal
        ASLAltitude = self.earthModel.getAltitude(*position) 
        # Ground level is remain constant as that of the launch site
        AGLAltitude = ASLAltitude - self.launchSiteElevation

        # Get wind based on AGL, in North-East-Up frame
            # TODO: This is only desirable for low-altitude flights, should switch to ASL altitude once out of the atmospheric boundary layer
        meanWind, turbWind = self.getWind(AGLAltitude, time)
        
        # Add earth's rotation speed to the mean wind
        # For No/Flat Earth models, the rotationRate will be zero and this will have no effect
        distanceFromRotationAxis = math.sqrt(position.X**2 + position.Y**2)
        earthRotationSpeed = self.earthModel.rotationRate * distanceFromRotationAxis
        # Earth's surface rotates towards the east, so add the rotation velocity to the East component (X) of the local ENU frame
        meanWind = meanWind + Vector(earthRotationSpeed,0,0)

        # Wind is calculated in the North-East-Up frame, and needs to be rotated into the global inertial frame
        # For No/Flat Earth models, the rotation will return a zero-rotation Quaternion, so this has no effect
        orientationOfENUFrameInGlobalFrame = self.earthModel.getInertialToENUFrameRotation(*position)
        meanWind = orientationOfENUFrameInGlobalFrame.rotate(meanWind)
        turbWind = orientationOfENUFrameInGlobalFrame.rotate(turbWind)

        # Now calculate total wind in global frame
        totalWind = meanWind + turbWind

        # Add air properties based on ASL, return the result
        return EnvironmentalConditions(
            ASLAltitude,
            *self.atmosphericModel.getAirProperties(ASLAltitude, time), # Asterisk unpacks the returned values
            totalWind, 
            meanWind, 
            turbWind
        )

    #### Wind properties ####
    def getWind(self, AGLElevation, time=None):
        ''' Returns wind in the North-East-Up (Y-X-Z) frame '''
        meanWindVel = self.meanWindModel.getMeanWind(AGLElevation)
        
        if self.turbulenceModel != None:
            turbWindVel = self.turbulenceModel.getTurbVelocity(AGLElevation, meanWindVel, time)
        else:
            turbWindVel = Vector(0,0,0)

        return meanWindVel, turbWindVel

    #### Wrappers for launch rail functions ####    
    def applyLaunchRailMotionConstraints(self, state, time):
        if self.launchRail == None:
            # Majority of the time we'll be off the rail and this will run
            return state

        onLaunchRail, adjustedState = self.launchRail.applyLaunchTowerMotionConstraint(state, time)
        
        # Delete rail if we've left it
        if not onLaunchRail:
            print("Launch rail cleared")
            self.launchRail = None

        return adjustedState

    def applyLaunchTowerForce(self, state, time, unadjustedForce):
        if self.launchRail == None:
            # Majority of the time we'll be off the rail and this will run
            return unadjustedForce
        
        return self.launchRail.applyLaunchTowerForce(state, time, unadjustedForce)

    #### Gravity ####
    def getGravityForce(self, inertia, state) -> ForceMomentSystem:
        '''
            Inputs:
                inertia: (`MAPLEAF.Motion.Inertia`)
                state:   (`MAPLEAF.Motion.RigidBodyState`/`MAPLEAF.Motion.RigidBodyState_3DoF`)

            Returns:
                gravityForce: (ForceMomentSystem) defined in the rocket's local frame
        '''
        # Get gravity force in the global frame
        gravityForce = self.earthModel.getGravityForce(inertia, state)
        
        try:
            # Convert to local frame if in 6DoF simulation
            gravityForce = state.orientation.conjugate().rotate(gravityForce) # rotate into local frame when in 6DoF mode
        except AttributeError:
            pass # Don't do anything in 3DoF mode (No local frame exists)

        return ForceMomentSystem(gravityForce, inertia.CG)
