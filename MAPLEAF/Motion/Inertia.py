# Created by: Henry Stoldt & Declan Quinn
# April 2020

from MAPLEAF.Motion import Vector

__all__ = [ "Inertia" ]

class Inertia():

    ___slots__ = [ "MOI", "MOICentroidLocation", "mass", "CG" ]

    def __init__(self, MOI, MOICentroidLocation, mass, CG=None, componentLocation=Vector(0,0,0)):
        """ 
            * MOIVector: Ixx, Iyy, Izz  
            * MOICentroidLocation: The point (in the rocket frame) about which Ixx, Iyy, and Izz were calculated.   
        """
        self.MOI = MOI
        self.MOICentroidLocation = MOICentroidLocation + componentLocation
        self.mass = mass
        
        if CG == None:
            self.CG = MOICentroidLocation + componentLocation
        else:
            self.CG = CG + componentLocation

    def checkDefinedAboutCG(self, inertiasList):
        '''
            Called by the functions that add/combine inertias.
            Checks that every inertia to be added up has MOIs defined about its CG.
            Throws a ValueError if this is not the case
        '''
        for inertia in inertiasList:
            if inertia.MOICentroidLocation != inertia.CG:
                raise ValueError("At least one of the inertias to be added has an MOI not defined about its CG: {}".format(inertia))

    def combineInertiasAboutPoint(self, inertiasList, Point):
        """
            Version of the combineInertias function below, which can combine Inertias about a point which is not coincident with the combined CG of the resulting combined object.
            This changes the resulting moments of inertia.

            Inputs:
                inertiasList:   List of other inertia objects. Can also be empty list, then parallel axis theorem is just applied to the present inertia object
                Point:          Point about which summed inertias will be calculated
            Returns:
                New instance of Inertia, with MOI defined about Point
        """    
        # Initialize variables to hold results
        totalMass = 0
        totalCG = Vector(0,0,0)
        totalMOI = Vector(0,0,0)

        # Add the current object to the list of Inertia objects to be combined
        if self not in inertiasList:
            inertiasList.append(self)

        self.checkDefinedAboutCG(inertiasList)

        # Add up all the inertias in the list
        for inertia in inertiasList:
            # Add mass and mass-weighted CG
            totalMass += inertia.mass
            totalCG += inertia.CG * inertia.mass

            ### Add inertia using parallel axis theorem ###
            
            # Add current inertia
            totalMOI += inertia.MOI
            # Add adjustment for different axis location
            distanceFromPoint = Point - inertia.MOICentroidLocation
            xAxesDistSqr = distanceFromPoint.Y*distanceFromPoint.Y + distanceFromPoint.Z*distanceFromPoint.Z
            yAxesDistSqr = distanceFromPoint.X*distanceFromPoint.X + distanceFromPoint.Z*distanceFromPoint.Z
            zAxesDistSqr = distanceFromPoint.X*distanceFromPoint.X + distanceFromPoint.Y*distanceFromPoint.Y
            totalMOI.X += inertia.mass*xAxesDistSqr
            totalMOI.Y += inertia.mass*yAxesDistSqr
            totalMOI.Z += inertia.mass*zAxesDistSqr

        # Compute actual CG by dividing out the mass
        totalCG = totalCG / totalMass

        return Inertia(totalMOI, Point, totalMass, totalCG)

    def combineInertias(self, inertiasList):
        """
            Combines inertias about the CG of the combined object
            Each component inertia must be defined about the component's CG
        """
        # Add the current object to the list of Inertia objects to be combined
        if self not in inertiasList:
            inertiasList.append(self)

        self.checkDefinedAboutCG(inertiasList)

        # Calculate combined CG
        totalMass = 0
        totalCG = Vector(0,0,0)

        for inertia in inertiasList:
            totalMass += inertia.mass
            totalCG += inertia.CG * inertia.mass # Adding mass-weighted CG

        if totalMass > 0:
            totalCG = totalCG / totalMass # Compute actual CG by dividing out the total mass
        else:
            totalCG = Vector(0,0,0)

        # Calculate combined MOI about combined CG
        totalMOI = Vector(0,0,0)
        for inertia in inertiasList:
            totalMOI += inertia.MOI # Add current inertia
            # Add adjustment for different axis location - parallel axis theorem
            distanceFromPoint = totalCG - inertia.MOICentroidLocation
            xAxesDistSqr = distanceFromPoint.Y*distanceFromPoint.Y + distanceFromPoint.Z*distanceFromPoint.Z
            yAxesDistSqr = distanceFromPoint.X*distanceFromPoint.X + distanceFromPoint.Z*distanceFromPoint.Z
            zAxesDistSqr = distanceFromPoint.X*distanceFromPoint.X + distanceFromPoint.Y*distanceFromPoint.Y
            totalMOI.X += inertia.mass*xAxesDistSqr
            totalMOI.Y += inertia.mass*yAxesDistSqr
            totalMOI.Z += inertia.mass*zAxesDistSqr

        return Inertia(totalMOI, totalCG, totalMass)

    def __add__(self, inertia2):
        return self.combineInertias([inertia2])

    def __str__(self):
        ''' Get string representation, used by str() and print() '''
        if self.MOICentroidLocation == self.CG:
            return 'MOI=({}) calculated about CG=({}) Mass=({}) '
        else:
            return 'MOI=({}) calculated about non-CG Point=({}) CG=({}) Mass=({})'

    def __eq__(self, inertia2):
        try:
            c1 = self.MOI == inertia2.MOI
            c2 = self.MOICentroidLocation == inertia2.MOICentroidLocation
            c3 = self.mass == inertia2.mass
            c4 = self.CG == inertia2.CG
            if c1 and c2 and c3 and c4:
                return True
            else:
                return False

        except AttributeError:
            return False