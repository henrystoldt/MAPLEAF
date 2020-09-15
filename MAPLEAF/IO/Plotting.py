# Created by: Henry Stoldt
# April 2020

'''
Functions to create plots and animations of simulations results.
'''

import re
from bisect import bisect_right
from collections import OrderedDict
from math import radians
from statistics import mean, stdev

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import pandas as pd

from MAPLEAF.Motion import interpolateRigidBodyStates, linInterpWeights
from MAPLEAF.IO import RocketFlight
from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import Vector

# For all plots for publications: (match publication font)
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["font.size"] = "10"
plt.rcParams["font.weight"] = "normal"

logFileCache = OrderedDict()
''' Will keep the contents of the last n log files loaded cached, to avoid re-loading repeatedly when making lots of plots '''

numberOfFilesToKeepCached = 10
''' Number of log files to keep cached '''

### Plot data from log files ###
def plotFromLogFiles(logFilePaths, plotDefinitionString, showPlot=True):
    if logFilePaths == None:
        print("ERROR: can't plot {} - must log data to plot it".format(plotDefinitionString))
        return

    if plotDefinitionString == "Velocity":
        # Prevents Velocity from also plotting angularVelocity
        columns = [ "VelocityX(m/s)", "VelocityY(m/s)", "VelocityZ(m/s)" ]  # Removes Angular Velocities from plot (otherwise plots all columns containing "Velocity")
    elif '&' in plotDefinitionString:
        # Split 'col' at each '&' symbol - this allows for plotting multiple things on a single graph
        columns = plotDefinitionString.split('&')
    else:
        columns = plotDefinitionString

    # Create a figure and axes that all data will be plotted
    fig, ax = plt.subplots(figsize=(6,4))

    plottedCols = [] # Keep track of whether any data matching "col" has been found
    print("Plotting log columns containing '{}'".format(columns))
    for logFilePath in logFilePaths:
        plottedCols += tryPlottingFromLog(logFilePath, columns, ax=ax, showPlot=False, columnsToExclude=plottedCols)
        
    if len(plottedCols) == 0:
        print("ERROR: no column names containing {} were found in the logFiles {}".format(plotDefinitionString, logFilePaths))    
    elif showPlot:        
        plt.show()

def tryPlottingFromLog(logPath, columnSpecs, columnsToExclude=[], ax=None, showPlot=True):
    '''
        Load the file at logPath, which should be either a simulationLog or a forceEvaluationLog
        Plots any columns whose names a) contain any of the strings in "columnNames", or b) match the regex defined by any of the strings in "columnNames" vs. time

        Returns True if a matching column was found and plotted, False if not

        Only data from the uppermost stage is plotted
    '''
    if ax == None:
        fig, ax = plt.subplots(figsize=(6,4))

    # Plot any column whose name includes any of the strings in columnNames
    if isinstance(columnSpecs, str):
        columnSpecs = [ columnSpecs ]

    if "Time(s)" not in columnSpecs:
        columnSpecs.append("Time(s)")

    # Get the matching column data
    data, names = getLoggedColumns(logPath, columnSpecs, columnsToExclude)

    # Grab x-axis
    for i in range(len(names)):
        if names[i] == "Time(s)":
            names.pop(i)
            x = data.pop(i)
            break

    # Plot data
    for i in range(len(names)):
        ax.plot(x, data[i], label=names[i])

    if len(names) > 0:
        ax.autoscale()
        ax.legend()

        if showPlot:
            plt.show()
    
    return names

def getLoggedColumns(logPath, columnSpecs, columnsToExclude=[], sep="\s+"):
    '''
        Inputs:
            logPath:            (string) path to a simulationLog or forceEvaluationLog file
            columnSpecs:        (list (string)) list of partial/full column names and/or regex expressions, to identify the desired columns
            columnsToExcluse:   (list (string)) list of full column names to exclude
            sep:                (string) controls the separator pandas.read_csv uses to load the file ('\s+' / whitespace) for rocketSimulator's log files. "," for .csv

        Outputs:
            Returns: matchingColumnData (list (list (float))), matchingColumnNames (list (string))
    '''
    # Make sure columnSpecs is a list of strings - otherwise we will iterate over every character in the string and match everything
    if isinstance(columnSpecs, str):
        columnSpecs = [ columnSpecs ]
    
    if logPath in logFileCache:
        df = logFileCache[logPath]
    else:
        with open(logPath, 'r') as file:
            lines = file.readlines()

        #### Figure out which lines are not part of the data table ####
        if "simulationLog" in logPath:
            skipRows, skipFooter = _getSkipRowsAndFooters_mainSimLog(lines)
        elif "forceEvaluationLog" in logPath:
            skipRows, skipFooter = _getSkipRowsAndFooters_forceEvalLog(lines)
        else:
            skipRows, skipFooter = (0, 0)

        #### Read file, grab desired columns ####
        # Read file, skipping any lines found in the previous section
        df = pd.read_csv(logPath, sep=sep, skiprows=skipRows, dtype=np.float64, skipfooter=skipFooter, engine='python')
            
        logFileCache[logPath] = df
        if len(logFileCache) > numberOfFilesToKeepCached:
            logFileCache.popitem(last=False)
    
    matchingColumnData = []
    matchingColumnNames = []

    for columnSpec in columnSpecs:
        for colName in df.columns:
            if (columnSpec in colName or re.match(columnSpec, colName)) and (colName not in columnsToExclude):
                y = list(df[colName])
                
                matchingColumnData.append(y)
                matchingColumnNames.append(colName)

    return matchingColumnData, matchingColumnNames

def _getSkipRowsAndFooters_mainSimLog(lines):
    '''
        Pass in an array of strings, containing all lines of a mainSimulationLog file.
        Returns the skiprows and skipfooter parameters used in the call to pd.readcsv
        skiprows and skipfooter are intended to skip everything in the log file that is not part of the main data table
    '''
    i = 0
    while len(lines[i]) < 4 or lines[i][0:4] != "Time":
        i += 1
    skipRows = list(range(i))

    # If in simulation log, find out how many footer lines to skip
    simEndLine = lines.index("Simulation Complete\n")
    skipFooter = len(lines) - simEndLine

    # Also find the parachute deployment lines and skip those
    stageDroppingSimStartPt = None
    for i in range(skipRows[-1]+2, simEndLine):
        if len(lines[i].split()) > 0:
            try:
                float(lines[i].split()[0])
            except (ValueError, IndexError):
                if lines[i].split()[0] == "Computing":
                    stageDroppingSimStartPt = i
                skipRows.append(i)

    if stageDroppingSimStartPt != None:
        # Skip values calculated for dropped stages
        skipFooter = len(lines) - stageDroppingSimStartPt

    return skipRows, skipFooter

def _getSkipRowsAndFooters_forceEvalLog(lines):
    '''
        Pass in an array of strings, containing all lines of a forceEvaluationLog file.
        Returns the skiprows and skipfooter parameters used in the call to pd.readcsv
        skiprows and skipfooter are intended to skip everything in the log file that is not part of the main data table
    '''
    skipRows = 0

    lastTime = float(lines[1].split()[0])
    firstDroppedStageRow = None
    for row in range(2, len(lines)):
        currTime = float(lines[row].split()[0])
        if lastTime - currTime > 5: # Assume time steps are generally < 5sec and Flight/Drop durations are generally > 5 sec
            firstDroppedStageRow = row
            break

        lastTime = currTime
    
    if firstDroppedStageRow != None:
        skipFooter = len(lines) + 1 - firstDroppedStageRow
    else:
        # Unknown file - try reading all rows
        skipFooter = 0

    return skipRows, skipFooter

### Plot Flight Paths ###
def plotFlightPaths_NoEarth(flightPaths, showPlot=True):
    '''
        Pass in an iterable of RocketFlight objects. Their flight paths will be plotted in a 3D graph.
    '''
    print("Plotting flight paths")

    plt.figure()
    ax = plt.axes(projection='3d')

    allX = []
    allY = []
    allZ = []

    for flight in flightPaths:
        x = []
        y = []
        z = []

        for state in flight.rigidBodyStates:
            x.append(state.position.X)
            y.append(state.position.Y)
            z.append(state.position.Z)

        allX += x
        allY += y
        allZ += z

        ax.plot3D(x, y, z)

    allCoords = allX + allY + allZ
    maxCoord = max(allCoords)
    minCoord = min(allCoords)
    halfLength = (maxCoord - minCoord)/2

    ax.set_xlim(mean(allX) - halfLength, mean(allX) + halfLength)
    ax.set_ylim(mean(allY) - halfLength, mean(allY) + halfLength)
    ax.set_zlim(min(allZ), min(allZ) + 2*halfLength)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')

    if showPlot:
        plt.show()

def plot_Earth_Mayavi(earthTexture='MAPLEAF/IO/blue_marble_spherical_splitFlipped.jpg'):
    from mayavi import mlab
    from tvtk.api import tvtk  # python wrappers for the C++ vtk ecosystem
    # create a figure window (and scene)
    fig = mlab.figure(size=(600, 600))

    # load and map the texture
    img = tvtk.JPEGReader()
    img.file_name = earthTexture
    texture = tvtk.Texture(input_connection=img.output_port, interpolate=1)
    # (interpolate for a less raster appearance when zoomed in)

    # use a TexturedSphereSource, a.k.a. getting our hands dirty
    R = 6371007.1809
    Nrad = 180

    # create the sphere source with a given radius and angular resolution
    sphere = tvtk.TexturedSphereSource(radius=R, theta_resolution=Nrad,
                                       phi_resolution=Nrad)

    # assemble rest of the pipeline, assign texture    
    sphere_mapper = tvtk.PolyDataMapper(input_connection=sphere.output_port)
    sphere_actor = tvtk.Actor(mapper=sphere_mapper, texture=texture)
    fig.scene.add_actor(sphere_actor)

def plotFlightPaths_FullEarth(flightPaths, showPlot=True):
    try:
        from mayavi import mlab
        from tvtk.api import tvtk  # python wrappers for the C++ vtk ecosystem
    
        plot_Earth_Mayavi()

        # Plot flight paths
        for flight in flightPaths:
            x = []
            y = []
            z = []

            for state in flight.rigidBodyStates:
                x.append(state.position.X)
                y.append(state.position.Y)
                z.append(state.position.Z)

            mlab.plot3d(x, y, z, tube_radius=20000, colormap='Spectral')

        if showPlot:
            mlab.show()

    except ModuleNotFoundError:
        print("Mayavi missing - falling back to plotting FlightPaths using matplotlib. Earth will not be shown.")
        plotFlightPaths_NoEarth(flightPaths)

### Flight Animation ###
def _keepNTimeSteps(flights, nFramesToKeep=600):
    '''
        Pass in list of RocketFlight objects and the number of frames/timesteps you'd like to keep
        Will return a matching list of Rocket Flight objects with linearly-interpolated, evenly spaced (in time) timesteps
    '''
    unpackResult = False
    if isinstance(flights, RocketFlight):
        unpackResult = True
        flights = [ flights ]

    maxLength = len(flights[0].times)
    
    # Find the max flight time of any stage
    maxTime = 0
    for flight in flights:
        if flight.times[-1] > maxTime:
            maxTime = flight.times[-1]

    newTimes = np.linspace(0, maxTime, num=nFramesToKeep)

    for flight in flights:
        # Interpolate flights to the times in newTimes
        interpStates = []

        interpCanardDefls = None
        if flight.actuatorDefls != None:
            nCanards = len(flight.actuatorDefls)
            interpCanardDefls = [ [] for i in range(nCanards) ]

        for time in newTimes:
            # SmallYIndex, SmallYWeight, largeYIndex, largeYWeight (for linear interpolation)
            smY, smYW, lgY, lgYW = linInterpWeights(flight.times, time)

            # Interpolate rigid body state
            if type(flight.rigidBodyStates[smY]) == type(flight.rigidBodyStates[lgY]):
                interpolatedState = interpolateRigidBodyStates(flight.rigidBodyStates[smY], flight.rigidBodyStates[lgY], smYW)
            else:
                # Handles the switch from 6DoF to 3DoF, where two adjacent states will be of different types
                interpolatedState = flight.rigidBodyStates[lgY] if lgYW > smYW else flight.rigidBodyStates[smY]

            interpStates.append(interpolatedState)

            # Interpolate canard deflections
            if flight.actuatorDefls != None:
                for i in range(nCanards):
                    # Interpolate the deflection of each canard
                    interpolatedDeflection = flight.actuatorDefls[i][smY]*smYW + flight.actuatorDefls[i][lgY]*lgYW
                    interpCanardDefls[i].append(interpolatedDeflection)
        
        flight.times = newTimes
        flight.rigidBodyStates = interpStates
        if flight.actuatorDefls != None:
            flight.actuatorDefls = interpCanardDefls

    if unpackResult:
        return flights[0]
    else:
        return flights

def _get3DPlotSize(Positions, sizeMultiple=1.1):
    '''
        Finds max X, Y, or Z distance from the origin reached during the a flight. Used to set the 3D plot size (which will be equal in all dimensions)
    '''
    centerOfPlot = Vector(mean(Positions[0]), mean(Positions[1]), mean(Positions[2]))

    xRange = max(Positions[0]) - min(Positions[0])
    yRange = max(Positions[1]) - min(Positions[1])
    zRange = max(Positions[2]) - min(Positions[2])

    if max(xRange, yRange, zRange) == 0:
        # For cases where the object does not move
        axisDimensions = 1.0
    else:
        axisDimensions = max([xRange, yRange, zRange]) * sizeMultiple
    
    return axisDimensions, centerOfPlot

def _findEventTimeStepNumber(flight, time):
    '''
        Given a time and a RocketFlight object, finds the time step that passes the given time
    '''
    if time == None:
        return None
    else:
        return bisect_right(flight.times, time) # Binary search for time value in 

def _createReferenceVectors(nCanards, maxAbsCoord, rocketLengthFactor=0.25, finLengthFactor=0.05):
    '''
        Creates a longitudinal vector and an array of nCanards perpendicular vectors. 
        These represent the rocket in the local frame and are rotated according to it's rigid body state at each time step.
        The size of the longitudinal and perpendicular lines are controlled by the rocketLength and finLength factor arguments and the maxAbsCoord.
    '''
    # Create vector reoresenting longitudinal axis in local frame
    refAxis = Vector( 0, 0, maxAbsCoord*rocketLengthFactor )
    
    # Create vectors going perpedicularly out, in the direction of each fin/canard in local frame
    radiansPerFin = radians(360 / nCanards)
    finToFinRotation = Quaternion(axisOfRotation=Vector(0,0,1), angle=radiansPerFin)
    perpVectors = [ Vector( maxAbsCoord*finLengthFactor, 0, 0 ) ]
    for i in range(nCanards-1):
        newVec = finToFinRotation.rotate(perpVectors[-1])
        perpVectors.append(newVec)

    return refAxis, perpVectors

def _createAnimationFigure(axisDimensions, centerOfPlot):
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    halfDim = axisDimensions/2
    
    ax.set_xlim3d([centerOfPlot[0] - halfDim, centerOfPlot[0] + halfDim])
    ax.set_ylim3d([centerOfPlot[1] - halfDim, centerOfPlot[1] + halfDim])
    ax.set_zlim3d([centerOfPlot[2] - halfDim, centerOfPlot[2] + halfDim])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    return fig, ax

def _createInitialFlightAnimationPlot_singleRocket(ax, nCanards, flight, refAxis, perpVectors):
    '''
        Called once for each rocket to set up the animation.
        Creates all the lines required for the rocket (longitudinal + perpendiculars + canards)
        These are then modified during each time step
    '''
    # Create flight path line
    Positions = flight.Positions
    flightPath = ax.plot(Positions[0][0:1], Positions[1][0:1], Positions[2][0:1])[0]
    if flight.engineOffTime != None:
        flightPath.set_color("red")
    else:
        flightPath.set_color("black")
    
    colors = ["blue", "red", "green", "purple", "brown", "grey", "peru", "skyblue", "pink", "darkblue", "darkgreen" ]

    # Create rocket/fin lines
    cg, rocketBase, tip, perpTips, canardTips, canardTails = _getRocketPoints(0, flight, refAxis, perpVectors)
    cgPoint = ax.scatter3D([ cg.X ], [ cg.Y ], [ cg.Z ])
    longitudinalLine = ax.plot([rocketBase.X, tip.X], [rocketBase.Y, tip.Y], [rocketBase.Z, tip.Z])[0]
    
    # Create fin and canard lines
    canardLines = []
    perpLines = []
    for i in range(nCanards):
        perpLines.append(ax.plot([rocketBase.X, perpTips[i].X], [rocketBase.Y, perpTips[i].Y], [rocketBase.Z, perpTips[i].Z])[0])
        perpLines[-1].set_color(colors[i % len(colors)])
        canardLines.append(ax.plot([canardTips[i].X, canardTails[i].X], [canardTips[i].Y, canardTails[i].Y], [canardTips[i].Z, canardTails[i].Z])[0])
        canardLines[-1].set_color(colors[i % len(colors)])

    # Plot target location if it exists
    target = flight.targetLocation
    if target != None:
        ax.scatter3D( [ target.X ], [ target.Y ], [ target.Z ])

    return cgPoint, flightPath, longitudinalLine, canardLines, perpLines

def _createInitialFlightAnimationPlot(ax, nCanards, flights, refAxis, perpVectors):
    '''
        Loops through all the rockets, calls the _singleRocket version of this function for each one.
    '''
    cgPoints = []
    flightPathLines = []
    longitudinalRocketLines = []
    allCanardLines = []
    allPerpLines = []
    for flight in flights:
        cgPoint, flightPath, longitudinalLine, canardLines, perpLines = _createInitialFlightAnimationPlot_singleRocket(ax, nCanards, flight, refAxis, perpVectors)
        cgPoints.append(cgPoint)
        flightPathLines.append(flightPath)
        longitudinalRocketLines.append(longitudinalLine)
        allCanardLines.append(canardLines)
        allPerpLines.append(perpLines)
    return cgPoints, flightPathLines, longitudinalRocketLines, allCanardLines, allPerpLines

def _getRocketPoints(timeStepNumber, flight, refAxis, perpVectors):
    '''
        For each rocket and time step, called to find the coordinates of the:
            cg, rocketBase, tip, perpTips, canardTips, canardTails
    '''
    nCanards = len(perpVectors)
    try:
        # Try plotting the actual rocket - using the orienation (only present in a 6DoF sim)
        cg = flight.rigidBodyStates[timeStepNumber].position
        currOrientation = flight.rigidBodyStates[timeStepNumber].orientation 

        axisVector = currOrientation.rotate(refAxis)
        perpVectors = [ currOrientation.rotate(x) for x in perpVectors ]
        
        # Assume rocket's CG is about halfway along it's axis
        rocketBase = flight.rigidBodyStates[timeStepNumber].position - axisVector*0.5        
        tip = rocketBase + axisVector
        perpTips = [ rocketBase + x for x in perpVectors ]

        if flight.actuatorDefls != None:
            # Get lengthwise center of each canard
            canardLongitudinalLocation = tip - axisVector*0.25
            canardCenters = [ canardLongitudinalLocation + x*0.5 for x in perpVectors ]

            # Get lengthwise vector of canard
            canardVecs = [ Quaternion(axisOfRotation=perpVectors[i], angle=radians(flight.actuatorDefls[i][timeStepNumber])).rotate(axisVector) for i in range(nCanards) ]

            canardRocketLengthFraction = 0.2
            canardTips = [ canardCenters[i] + canardVecs[i]*canardRocketLengthFraction*0.25 for i in range(nCanards) ]
            canardTails = [ canardCenters[i] - canardVecs[i]*canardRocketLengthFraction*0.75 for i in range(nCanards) ]
        else:
            # Initialize lines to zero co-ordinates - these lines can be used to plot the parachutes later, if necessary
            canardTips = []
            canardTails = []
            for i in range(nCanards):
                canardTips.append(Vector(0,0,0))
                canardTails.append(Vector(0,0,0))

    except AttributeError:
        # Otherwise plot parachute for 3DoF simulation (3DoF only used for descent)
        rocketBase = flight.rigidBodyStates[timeStepNumber].position   
        cg = flight.rigidBodyStates[timeStepNumber].position        
        axisVector = refAxis*0.75
        tip = rocketBase - refAxis*0.2

        if flight.mainChuteDeployTime == None or timeStepNumber < flight.mainChuteTimeStep:
            chuteSize = 0.35
        else:
            chuteSize = 1.0

        perpTips = [ rocketBase + x*chuteSize + axisVector for x in perpVectors ]
        canardTips = perpTips

        parachuteTip = rocketBase + axisVector*1.2
        canardTails = [parachuteTip]*nCanards

    return cg, rocketBase, tip, perpTips, canardTips, canardTails

def _update_plot(timeStepNumber, flights, refAxis, perpVectors, cgPoints, flightPathLines, longitudinalRocketLines, allCanardLines, allPerpLines):
    ''' 
        Plot Update function - This gets called every time step of the simulation, updates the data for each point and line in the plot
    '''

    rocketColors = [ 'black', 'blue', 'fuchsia', 'olive', 'maroon', 'purple', 'sienna' ]

    for i in range(len(flights)):
        # Get important rocket coordinates at current time step
        cg, rocketBase, tip, perpTips, canardTips, canardTails = _getRocketPoints(timeStepNumber, flights[i], refAxis, perpVectors)
        nCanards = len(perpVectors)
        
        # Extract data for current stage to simplify code below
        flightPath = flightPathLines[i]
        Positions = flights[i].Positions
        engineOffTimeStep = flights[i].engineOffTimeStep
        cgPoint = cgPoints[i]
        longitudinalLine = longitudinalRocketLines[i]
        perpLines = allPerpLines[i]
        canardLines = allCanardLines[i]

        def setRocketColor(clr):
            flightPath.set_color(clr)
            cgPoint.set_color(clr)
            longitudinalLine.set_color(clr)
            for line in perpLines:
                line.set_color(clr)
            for line in canardLines:
                line.set_color(clr)

        if timeStepNumber == 0:
            clr = rocketColors[i % len(rocketColors)]
            setRocketColor(clr)
            flightPath.set_color('black')    

        # Update Flight path
        flightPath.set_data(Positions[0][:timeStepNumber], Positions[1][:timeStepNumber])
        flightPath.set_3d_properties(Positions[2][:timeStepNumber])
        if i > 0:
            if timeStepNumber < engineOffTimeStep:
                flightPath.set_color("red")
                flightPath.set_alpha(1)
            else:
                flightPath.set_color("gray")
                flightPath.set_alpha(0.5)
        elif engineOffTimeStep == None or timeStepNumber >= engineOffTimeStep:
            flightPath.set_color("black")
        else:
            flightPath.set_color("red")
        
        # Update rocket CG and main line
        cgPoint._offsets3d = ([ cg.X ], [ cg.Y ], [ cg.Z ])
        longitudinalLine.set_data([rocketBase.X, tip.X], [rocketBase.Y, tip.Y])
        longitudinalLine.set_3d_properties([rocketBase.Z, tip.Z])

        # Update fins and canards
        for c in range(nCanards):
            # Update tail fins / orientation indicators / parachute
            perpLines[c].set_data([rocketBase.X, perpTips[c].X], [rocketBase.Y, perpTips[c].Y])
            perpLines[c].set_3d_properties([rocketBase.Z, perpTips[c].Z])
            # Update canards / parachute
            canardLines[c].set_data([canardTips[c].X, canardTails[c].X], [canardTips[c].Y, canardTails[c].Y])
            canardLines[c].set_3d_properties([canardTips[c].Z, canardTails[c].Z])

        # Turn lines gray if landed
        if cg == flights[i].rigidBodyStates[-1].position:
            setRocketColor('gray')

def flightAnimation(flights, showPlot=True, saveAnimFileName=None):
    '''
        Pass in a list of RocketFlight object(s). Intended to contain a single RocketFlight object, or multiple if it was a staged flight (one object per stage)
        showPlot controls where the animation is shown or not
        saveAnimFileName should be a string file name/path that the animation should be saved to ex: "SampleRocketFlightAnimation.mp4"
    '''
    #### Set up data for animation ####
    # Filter out extra frames - seems like there's a limit to the number of frames that will work well with matplotlib, otherwise the end of the animation is weirdly sped up
    flights = _keepNTimeSteps(flights, nFramesToKeep=900)

    # Transform position info into arrays of x, y, z coordinates
    for flight in flights:
        Positions = [ [], [], [] ]
        for state in flight.rigidBodyStates:
            for i in range(3):
                Positions[i].append(state.position[i])
        flight.Positions = Positions

    # Set xyz size of plot - equal in all dimensions
    axisDimensions, centerOfPlot = _get3DPlotSize(flights[0].Positions) # Assumes the top stage travels the furthest

    # Calculate frames at which engine turns off and main chute deploys
    for flight in flights:
        flight.engineOffTimeStep = _findEventTimeStepNumber(flight, flight.engineOffTime)
        flight.mainChuteTimeStep = _findEventTimeStepNumber(flight, flight.mainChuteDeployTime)

    if flights[0].actuatorDefls != None:
        # Assuming canards always on the top stage
        nCanards = len(flights[0].actuatorDefls)
    else:
        nCanards = 4 # Create canard lines to reuse them for the parachute

    refAxis, perpVectors = _createReferenceVectors(nCanards, axisDimensions)

    #### Create Initial Plot ####
    fig, ax = _createAnimationFigure(axisDimensions, centerOfPlot)
    cgPoints, flightPathLines, longitudinalRocketLines, allCanardLines, allPerpLines = _createInitialFlightAnimationPlot(ax, nCanards, flights, refAxis, perpVectors)

    # Play animation
    ani = animation.FuncAnimation(fig, _update_plot, frames=(len(Positions[0]) - 1), fargs=(flights, refAxis, perpVectors, cgPoints, flightPathLines, longitudinalRocketLines, allCanardLines, allPerpLines), interval=1, blit=False, save_count=0, repeat_delay=5000)

    if showPlot:
        plt.show()

    if saveAnimFileName != None:
        ani.save("SampleRocket.mp4", bitrate=2500, fps=60)

### Plot Monte Carlo Results ###
def plotAndSummarizeVectorResult(vectorList, name="Landing location", monteCarloLogger=None, showPlot=True):
    '''
        Plots 3D scatter plot of vector result, outputs data summary to console.
    '''
    x = []
    y = []
    z = []

    consoleOutput = []

    consoleOutput.append("{}s:   X       Y        Z".format(name))
    for i in range(len(vectorList)):
        loc = vectorList[i]
        consoleOutput.append("Simulation #{}:   {:>7.1f}".format(i+1, loc))
        x.append(loc.X)
        y.append(loc.Y)
        z.append(loc.Z)

    consoleOutput.append("")
    consoleOutput.append("Mean {}: ({:>8.1f} {:>8.1f} {:>8.1f})".format(name, mean(x), mean(y), mean(z)))
    consoleOutput.append("Standard deviation:    ({:>8.1f} {:>8.1f} {:>8.1f})".format(stdev(x), stdev(y), stdev(z)))
    consoleOutput.append("")

    for line in consoleOutput:
        if monteCarloLogger != None:
            monteCarloLogger.log(line)
        else:
            print(line)

    print("Plotting {}".format(name))     

    maxCoord = max(x + y + z)
    minCoord = min(x + y + z)
    halfLength = (maxCoord - minCoord)/2

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x,y,z)

    ax.set_xlim(mean(x) - halfLength, mean(x) + halfLength)
    ax.set_ylim(mean(y) - halfLength, mean(y) + halfLength)
    ax.set_zlim(min(z), min(z) + 2*halfLength)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')

    ax.set_title(name)

    if showPlot:
        plt.show()

def plotAndSummarizeScalarResult(scalarList, name="Apogee", monteCarloLogger=None, showPlot=True):
    '''
        Plots Histogram of scalar result, outputs data summary to console
    '''
    consoleOutput = []

    consoleOutput.append("{}s:".format(name))
    for i in range(len(scalarList)):
        consoleOutput.append("Simulation #{}: {}".format(i+1, scalarList[i]))

    consoleOutput.append("")
    consoleOutput.append("Mean {}:        {:>8.1f}".format(name, mean(scalarList)))
    consoleOutput.append("Standard Deviation: {:>8.1f}".format(stdev(scalarList)))
    consoleOutput.append("")

    for line in consoleOutput:
        if monteCarloLogger != None:
            monteCarloLogger.log(line)
        else:
            print(line)

    print("Plotting {}s".format(name))

    plt.hist(scalarList)
    plt.xlabel(name)
    plt.ylabel("Count")

    if showPlot:
        plt.show()
