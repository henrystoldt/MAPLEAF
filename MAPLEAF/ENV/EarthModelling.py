''' These classes model the earth's gravity and perform some coordinate transformations between the global inertial frame and the ENU/launch tower frame '''

from abc import ABC, abstractmethod
from math import atan2, cos, degrees, pi, radians, sin, sqrt
from typing import Union

import numpy as np

from MAPLEAF.IO import getAbsoluteFilePath
from MAPLEAF.Motion import (Inertia, Quaternion, RigidBodyState,
                            RigidBodyState_3DoF, Vector)

__all__ = [ "earthModelFactory", "NoEarth", "FlatEarth", "SphericalEarth", "WGS84" ]

class EarthModel(ABC):
    ''' Interface for all earth models '''
    # All earth models will have these default values
    rotationRate = 0

    @abstractmethod
    def getGravityForce(self, inertia: Inertia, state: Union[RigidBodyState, RigidBodyState_3DoF]) -> Vector:
        ''' Return gravity force vector in the global inertial frame '''
        return

    @abstractmethod
    def getInertialToENUFrameRotation(self, x: float, y: float, z: float) -> Quaternion:
        ''' 
            Return Quaternion defining the rotation from the global inertial frame 
            to the North-East-Up frame defined on the surface under the air vehicle's position
        '''
        return

    def getInertialToNEDFrameRotation(self, x, y, z):
        inertialToENURotation = self.getInertialToENUFrameRotation(x, y, z)
        ENUToNED = Quaternion(axisOfRotation=Vector(1,1,0), angle=pi)
        return inertialToENURotation * ENUToNED

    @abstractmethod
    def getAltitude(self, x: float, y: float, z: float) -> float:
        ''' Given the aircraft coordinates in the global inertial frame, should return the altitude above SEA level (ASL) - to be used in wind calculations '''
        return

    def convertIntoGlobalFrame(self, 
        launchTowerFrameState: Union[RigidBodyState, RigidBodyState_3DoF], 
        latitude: float, 
        longitude: float, 
    ) -> Union[RigidBodyState, RigidBodyState_3DoF]:
        ''' 
            Should take a RigidBodyState defined in the launch tower frame (fixed to the earth's surface), where 
            position.Z has been redefined relative to sea level (instead of ground-level), and convert it into
            the global inertial frame.
            Exception is the Angular Velocity, since it is defined in the vehicle's local frame. If the earth model is
                rotating, add the earth's angular velocity to it (after conversion into the local frame)
        '''
        return

def earthModelFactory(envDictReader=None) -> EarthModel:
    ''' Provide an envDictReader (`MAPLEAF.IO.SubDictReader`). If none is provided, returns a FlatEarth model '''
    if envDictReader == None:
        return FlatEarth()

    earthModel = envDictReader.getString("EarthModel")

    if earthModel == "Flat":
        return FlatEarth()
    elif earthModel == "Round":
        return SphericalEarth()
    elif earthModel == "WGS84":
        return WGS84()
    elif earthModel == "None":
        return NoEarth()
    else:
        raise NotImplementedError("Earth model: {} not found. Try 'Flat', 'Round' or 'WGS84'".format(earthModel))

class NoEarth(EarthModel):
    rotationRate = 0 # rad/s
    noRotation = Quaternion(1,0,0,0)
    noForce = Vector(0,0,0)

    def getGravityForce(self, inertia, state):
        return self.noForce

    def getInertialToENUFrameRotation(self, x, y, z):
        return self.noRotation

    def getAltitude(self, x, y, z):
        return z

    def convertIntoGlobalFrame(self, launchTowerFrameState, _, __):
        ''' Launch tower frame is global inertial frame '''
        return launchTowerFrameState

class FlatEarth(NoEarth):
    # GM = 398600.5 # km^3/s^2 Geocentric gravitational constant (Mass of atmosphere included)
    GM = 398600.5e9 # m^3/s^2

    def getGravityForce(self, inertia, state):
        ''' Returns a gravity force vector in the global frame '''
        gravityDirection = Vector(0, 0, -1) # Down in the launch tower frame

        distanceFromCenterOfEarth = state.position.Z + 6371000 # Adding earth's average radius
        gravityMagnitude = inertia.mass * self.GM / distanceFromCenterOfEarth**2 # Checked that this equation gives results similar to the 
            # US Standard Atmosphere - offset a bit depending on what earth radius is used. Current 6371 km gives slightly lower values that USTDA.

        return gravityMagnitude * gravityDirection

class SphericalEarth(EarthModel):
    ''' Models a non-rotating, uniform, spherical earth '''
    radius = 6371007.1809 # m - from NESC-RP-12-007770, Volume II
    rotationRate = 7.292115e-5 # rad/s angular velocity - from WGS84 model. Defined WRT stars (not our sun)
    # GM = 398600.5 # km^3/s^2 Geocentric gravitational constant (Mass of atmosphere included)
    GM = 3.98600436e14 # m^3/s^2- from NESC-RP-12-007770, Volume II

    def geodeticToCartesian(self, lat, lon, height, timeOffset=0):
        '''
            Converts from geodetic coordinates to cartesian coordinates, assuming the surface is a sphere.
            
            Resulting Z axis goes through north pole, X axis goes through prime meridian at the equator, and 
                the Y axis is perpendicular to both in such a way as to form a right-handed coordinate system

            Inputs:
                lat:    (float) latitude - degrees
                lon:    (float) longitude - degrees
                height: (float) ASL (usually m)

            Returns:
                x:      (float) meters, relative to center of sphere
                y:      ' '
                z:      ' '
        '''
        lat = radians(lat)
        lon = radians(lon)

        # Adjust for rotation rate
        lon += self.rotationRate * timeOffset

        radius = self.radius + height

        x = radius * cos(lat) * cos(lon)
        y = radius * cos(lat) * sin(lon)
        z = radius * sin(lat)
        return x, y, z

    def cartesianToGeodetic(self, x, y, z, timeOffset=0):
        ''' Convert from cartesian to lat/lon/height coordinates, where height is ASL '''
        p = sqrt(x*x + y*y)
        
        lon = atan2(y, x)

        # Adjust for rotation rate
        lon -= self.rotationRate * timeOffset

        lat = atan2(z, p)
        height = sqrt(x*x + y*y + z*z) - self.radius
        
        return degrees(lat), degrees(lon), height

    def getGravityForce(self, inertia, state):
        ''' Returns a gravity force vector in the global frame '''
        gravityDirection = -(state.position.normalize()) # Gravity pulls toward the earth's center of mass, which is the origin
        
        distanceFromCenterOfEarth = state.position.length()
        gravityMagnitude = inertia.mass * self.GM / distanceFromCenterOfEarth**2 # Checked that this equation gives results similar to the 
            # US Standard Atmosphere - offset a bit depending on what earth radius is used. Current 6371 km gives slightly lower values that USTDA.

        return gravityMagnitude * gravityDirection

    def getInertialToENUFrameRotation(self, x, y, z):
        ''' Returns a Quaternion that defines the rotation b/w the global inertia frame and the local, surface-normal North-East-Up (y-x-z) frame '''
        # Time offset not necessary, since the x, y, z coordinates are inertial, and we are not 
            # interested in finding a particular location on the surface of the earth,
            # we are just interested in finding the surface normal under a x/y/z location. 
            # The result is independent of the earth's rotation.
        lat, lon, height = self.cartesianToGeodetic(x, y, z)

        # Rotation between inertial frame and ENU frame will be composed in two steps
        # Step 1: Rotate about inertial/initial Z-axis to Y-axis in such a way, that after rotation 2, 
            # it will be pointing North
        rot1Angle = radians(lon + 90)
        rot1 = Quaternion(axisOfRotation=Vector(0,0,1), angle=rot1Angle)
        
        # Step 2: Rotate about the local x-axis to match the latitude
        rot2Angle = radians(90 - lat)
        rot2 = Quaternion(axisOfRotation=Vector(1,0,0), angle=rot2Angle)

        # Sequentially combine rotations by multiplying them
        return rot1 * rot2

    def getAltitude(self, x, y, z):
        # Height is independent of the earth's rotation
        _, __, height = self.cartesianToGeodetic(x, y, z)
        return height

    def convertIntoGlobalFrame(self, launchTowerFrameState, lat, lon):
        ### Position ###
        height = launchTowerFrameState.position.Z # ASL altitude
        globalFramePosition = Vector(*self.geodeticToCartesian(lat, lon, height))
        
        ### Velocity ###
        # Find orientation of launch tower frame relative to global frame
        launchTowerToGlobalFrame = self.getInertialToENUFrameRotation(*globalFramePosition)

        # Rotate velocity accordingly
        rotatedVelocity = launchTowerToGlobalFrame.rotate(launchTowerFrameState.velocity)
        # Add earth's surface velocity
        earthAngVel = Vector(0, 0, self.rotationRate)
        velocityDueToEarthRotation = earthAngVel.crossProduct(globalFramePosition)
        globalFrameVelocity = rotatedVelocity + velocityDueToEarthRotation

        try:  # 6DoF Case           
            ### Orientation ###
            globalFrameOrientation = launchTowerToGlobalFrame * launchTowerFrameState.orientation

            ### Angular Velocity ###
            # Angular velocity is defined in the vehicle's local frame, so the conversion needs to go the other way
            earthAngVel_RocketFrame = globalFrameOrientation.conjugate().rotate(earthAngVel)
            localFrame_adjustedAngVel = launchTowerFrameState.angularVelocity + earthAngVel_RocketFrame

            return RigidBodyState( 
                globalFramePosition, 
                globalFrameVelocity, 
                globalFrameOrientation, 
                localFrame_adjustedAngVel
                )
        
        except AttributeError: # 3DoF Case
            return RigidBodyState_3DoF(globalFramePosition, globalFrameVelocity)             

class WGS84(SphericalEarth):
    ''' 
        Models a rotating, ellipsoid earth, with non-uniform gravity 
        Inherits the getInertialToENUFrameRotation function from SphericalEarth, otherwise overrides everything
    '''
    # Set up/calculate the WGS84 Ellipsoid parameters and derived values
    a = 6378137             # m, semi-major axis
    f = 1/298.257223563     # Flattening (derived from C_20)
    b = a * (1-f)           # m, semi-minor axis = 6 356 752.314 140
    e2 = f*(2-f)            # First eccentricity squared (e^2) = 6.694 379 990 14 e-3
    eP2 = e2 / ((1-f)**2)   # Second eccentricity squared (e'^2) = 6.739 496 742 28 e-3s
    eP = sqrt(eP2)          # Second eccentricity (e')

    def __init__(self):
        # Read gravity coefficients from table
        # Columns are: Degree Order C S (Where C and S are the coefficients)
        coeffPath = getAbsoluteFilePath('./MAPLEAF/ENV/sphericalHarmonicGravityCoeffs.txt')
        gravityCoeffs = np.loadtxt(coeffPath, skiprows=2)
        
        # Convert that table into nested dictionary form, to make C and S coefficients easily accessible by degree and order
        Ccoeffs = {}
        Scoeffs = {}
        for row in gravityCoeffs:
            degree, order, C, S, = row
            
            if degree not in Ccoeffs:
                # Create subdictionary for each degree if this is the first time we're encountering that degree
                Ccoeffs[degree] = {}
                Scoeffs[degree] = {}

            # Save coefficient from this row
            Ccoeffs[degree][order] = C
            Scoeffs[degree][order] = S

        # Save results
        # Now C_{2,3} is accessible as self.C[2][3], same for self.S
        self.C = Ccoeffs
        self.S = Scoeffs

    def geodeticToCartesian(self, lat, lon, height, timeOffset=0):
        '''        
            Converts from geodetic coordinates to cartesian coordinates, assuming the surface is a sphere.
            
            Resulting Z axis goes through north pole, X axis goes through prime meridian at the equator, and 
                the Y axis is perpendicular to both in such a way as to form a right-handed coordinate system
            
            Method adapted from: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates

            Note: In other references, it is common to set \phi = lat, \lamda = lon

            Inputs:
                lat:        (float) latitude - degrees
                lon:        (float) longitude - degrees
                height:     (float) ASL - m
                timeOffset  (float) Offset added to results based on earth's rotation rate.
                                At timeOffset=0, this inertial frame is aligned with the standard earth-centered-earth-fixed (ECEF) frame

            Returns:
                x:      (float) meters, relative to center of ellipsoid
                y:      ' '
                z:      ' '
        '''
        lat = radians(lat)
        lon = radians(lon)

        # Adjust for earth's rate of rotation
        lon += self.rotationRate * timeOffset

        N = self.a / sqrt(1 - self.e2*sin(lat)**2)

        x = (N + height)*cos(lat)*cos(lon)
        y = (N + height)*cos(lat)*sin(lon)
        z = ((self.b**2 / self.a**2)*N + height) * sin(lat)

        return x, y, z

    def cartesianToGeodetic(self, x, y, z, timeOffset=0):
        '''
            Inputs in m, sec
            Convert from earth-centered inertial coordinates to WGS84 geodetic coordinates
            Method adapted from: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates

            At timeOffset=0, the inertial frame is aligned with the standard earth-centered-earth-fixed (ECEF) frame

            Returns latitude, longitude, height in degrees, m
        '''
        lon = atan2(y, x) # Longitude is easy to find

        # Adjust longitude for earth's rotation
        lon -= self.rotationRate * timeOffset

        ### Compute latitude and height normal to surface w/ Newton-Raphson method ###
        p = sqrt(x*x + y*y)
        Kold = 1/(1 - self.e2)
        Korig = Kold

        while True:
            ci = ((p*p + (1-self.e2)*z*z * Kold**2)**(1.5)) / (self.a * self.e2)
            Knew = 1 + (p*p + (1 - self.e2)*z*z * Kold**3) / (ci - p*p)
            
            # End iterations once delta per iteration < 1e-10
            if abs(Knew - Kold) < 1e-10:
                break

            Kold = Knew

        k = Knew

        lat = atan2(k*z, p)

        h = (1/self.e2)*(1/k - 1/Korig)*sqrt(p*p + z*z*k*k)

        return degrees(lat), degrees(lon), h

    # Re-use conversions to/from spherical coordinates from SphericalEarth
        # Useful for Spherical Harmonic Gravity model calculations, which are performed in spherical coordinates
    cartesianToSpherical = SphericalEarth.cartesianToGeodetic
    sphericalToCartesian = SphericalEarth.geodeticToCartesian

    def getGravityForce(self, inertia, state):
        ''' 
            Get gravity using the J2 model (just a single spherical harmonic coefficient accounting for the earth's oblateness) 
            Method from NESC-RP-12-00770, Volume II, pg.51, Eqn 29-31
        '''
        # J2 Model just uses the first spherical harmonic coefficient
        J2 = 0.00108262982
        re = self.a # Equatorial radius (Semi-major axis of ellipsoid)

        x, y, z = state.position[0], state.position[1], state.position[2]
        r = state.position.length()
        
        mu = self.GM

        # Precompute common parts of equation
        frac = (3*J2 * re*re) / (2 * r**4)
        multiplier = -mu*inertia.mass / r**3

        xForce = x * multiplier * (1 - frac*(5*z*z - r*r))
        yForce = xForce * y/x
        zForce = z * multiplier * (1 - frac*(5*z*z - 3*r*r))

        return Vector(xForce, yForce, zForce)
