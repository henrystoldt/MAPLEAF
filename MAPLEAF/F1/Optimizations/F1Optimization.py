from MAPLEAF.IO import Plotting
import math
import statistics

from pyparsing import Or



def trajectoryOptimization(logFilesList):

    # Choose the forces log file
    forcesLogPath = None
    for filePath in logFilesList:
        if "forceEvaluationLog" in filePath:
            forcesLogPath = filePath

    # If we didn't find it, crash
    if forcesLogPath == None:
        raise ValueError("One of the file paths provided to this function must be to a force evaluation log. Make sure SimControl.loggingLevel >= 2 to produce force evaluation log files.")

    columnSpecs = ["Position","Velocity", "Motor2Thrust", "Motor2PropellantRemaining", "Mass(kg)", "Motor2PropellantMassFlow","Time"]
    exclude = ['AngularVelocityX(rad/s)', 'AngularVelocityY(rad/s)', 'AngularVelocityZ(rad/s)', 'Motor1PropellantRemaining', 'Motor1Thrust(N)', 'Motor1PropellantMassFlow']

    columns, columnNames = Plotting.getLoggedColumns(filePath, columnSpecs, exclude, enableCache=False)

    xPos = columns[0]
    yPos = columns[1]
    zPos = columns[2]
    xVel = columns[3]
    yVel = columns[4]
    zVel = columns[5]
    motor2 = columns[6]
    propulsionMass = columns[7]
    rocketMass = columns[8]
    massFlowRate_Motor2 = columns[9]
    time = columns[10]
    
    

    # initialMass = rocketMass[0]
    desiredAltitude = 460000 #m
    earthRadius = 6371000 #m
    SECO = False
    g = 9.81 #m/s^2
    desiredOrbitVelocity = math.sqrt(g*earthRadius**2/(earthRadius + desiredAltitude))
    altitudeRange = 15000
    requiredDeltaV = 100 #m/s
    motor2Deployed = False
    stage2TotalMass = 0 
    stage2PropulsionMass = 0
    velocity = [0] * len(xVel)
    altitude = [0] * len(xPos)
    maxaltitude = None
    minaltitude = None

    # altitude and velocity calcualtion from log file
    for i in range(len(xPos)):
        altitude[i] = math.sqrt(xPos[i]**2 + yPos[i]**2 + zPos[i]**2) - earthRadius
        velocity[i] = math.sqrt(xVel[i]**2 + yVel[i]**2 + zVel[i]**2)
        SECO = True
        # mass_seco = rocketMass[i]
        massFlowRate = massFlowRate_Motor2[i-1]
        motor2Thrust = motor2[i-1]
        propulsionMassRemaining = propulsionMass[i-1]
        motor2Deployed = True

    # max altitude calculation
    for num in altitude:
        if (maxaltitude == None or num > maxaltitude):
            maxaltitude = num

    # min altitude calculation
    for num, v in zip(altitude, velocity):
        if (v < 7600):
            pass
                
        elif (minaltitude == None or num < minaltitude):
            minaltitude = num

    # average orbital velocity calculation
    res = []
    for idx, v in enumerate(velocity):
        if (v > 7500):
            res.append(velocity[idx])
   
    avg_orbitalveloctiy = sum(res)/len(res)  
    stdev_orbitalvelocity = statistics.stdev(res)

    # average altitude calculation
    rep = []  
    for idx, a in enumerate(altitude):
        if (a > minaltitude ):
            rep.append(altitude[idx])

    avg_altitude = sum(rep)/len(rep)
    stdev_altitude = statistics.stdev(rep)

    # Desired stdev for orbital velocity and altitude
    desiredstdev_orbitalvelocity = 2.5
    desiredstdev_altitude = 5000
    #Trajectory Cost
    print("Average Velocity: " + str(avg_orbitalveloctiy))
    print("Desired Velocity: " + str(desiredOrbitVelocity))
    print("Standard Deviation Orbital Velocity: " + str(stdev_orbitalvelocity))
    print("Max Altitude: " + str(maxaltitude))
    print("Min Altitude: " + str(minaltitude))
    print("Desired Altitude: " + str(desiredAltitude))
    print("Average Altitude: " + str(avg_altitude))
    print("Standard Deviation Altitude: " + str(stdev_altitude))

    c1 = abs(1 - maxaltitude/desiredAltitude) 
    c2 = abs(1 - minaltitude/desiredAltitude)
    c3 = abs(1 - stdev_altitude/desiredstdev_altitude)
    c4 = abs(1 - stdev_orbitalvelocity/desiredstdev_orbitalvelocity)
    cost = (c1 + c2)/2 + c3 * 0.25 + c4 * 0.25
    # COMMENT OUT BEFORE RUNNING OPTIMIZATION STUDY 
    # ---------------------------------------------------------
    return(cost)
    
    
#Computes cost function for a given file name. Does not run if called from another file
if __name__ == "__main__":
    #simDefName = "F9_trajectoryOptimizationTest"
    simDefName = 'F1'
    runNumber = 2
    fileName = "{}_forceEvaluationLog_run{}.txt".format(simDefName, runNumber)
    fileName2 = "{}_simulationLog_run{}.txt".format(simDefName, runNumber)
    logPath = "MAPLE_AF/F1/{}".format(fileName)
    logPath2 = "MAPLE_AF/F1/{}".format(fileName2)
    logFileList = ['random', logPath2, logPath]
    cost = trajectoryOptimization(logFileList)
    print("Cost Function output: " + str(cost))