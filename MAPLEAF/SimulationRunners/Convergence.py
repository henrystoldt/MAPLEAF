
from MAPLEAF.SimulationRunners import Simulation
import matplotlib.pyplot as plt
import csv

__all__ = [ "ConvergenceSimRunner" ]

class ConvergenceSimRunner(Simulation):
    '''
        Runs a simulation repeatedly, decreasing the time step or target error each time, monitoring for convergence
    '''
    def __init__(self, simDefinitionFilePath=None, simDefinition=None, silent=False):
        Simulation.__init__(self, simDefinitionFilePath=simDefinitionFilePath, simDefinition=simDefinition, silent=silent)

    def convergeSimEndPosition(self, refinementRatio=2, simLimit=10, plot=True, stopAtConvergence=False, showPlot=True, plotLineLabel="Simulations", ax1=None, ax2=None):
        '''
            Takes simulation and runs it repeatedly, cutting the time step in half each time.
            Once convergence is approximately asymptotic, exits and returns series of final positions, convergence order, and extrapolated final position
            Should use with simulations that have an EndCondition of type "Time"
                # Otherwise sim will be run using current settings, and its endtime will be taken as the new end time for future convergence sims
            This Fn called by compareIntegrationSchemes functions

            Parameters:
                simConfigFilePath       string, /path/to/simConfigFile
                fW                      SimDefinition, overrides simConfigFilePath
                refinementRatio         Number, Each time sim is run, time step or target error is divided by this number
                simLimit                Number, Max number of simulations to run (takes exponentially more time to run more simulations)
                plot                    True/False, whether to plot the results
                stopAtConvergence       True/False, if False, runs simLimit simulations even if asymptotic convergence is reached earlier
                showPlot                True/False, if True, calls plt.show()
                plotLineLabel           string, Label of line on plot
                ax1                     matplotlib Axes, Z-location (Y) axis
                ax2                     matplotlib Axes, Wall Time (Y) axis (2nd Y-axis)
        '''
        from MAPLEAF.IO.gridConvergenceFunctions import checkConvergence
        from statistics import mean
        import time

        self._setUpConfigFileForConvergenceRun()
        
        timeStepMethod = self.simDefinition.getValue("SimControl.timeDiscretization")
        adaptiveTimeStepping = "Adaptive" in timeStepMethod

        timeStepKey = "SimControl.timeStep"
        targetErrorKey = "SimControl.TimeStepAdaptation.targetError"

        #### Run Simulations ####
        print("Starting convergence simulations")
        if not adaptiveTimeStepping:
            timeStep = float(self.simDefinition.getValue(timeStepKey))*refinementRatio # Multiplied by 2 to give correct time step in first iteration
        else:
            targetError = float(self.simDefinition.getValue(targetErrorKey))*refinementRatio # Multiplied by 2 to give correct time step in first iteration

        simCount = 1
        finalPositionHistory = []
        convergenceHistory = []
        timeStepHistory = []
        simTimeHistory = []

        def printConvergenceHistory(ax1=ax1, ax2=ax2):
            print("")
            print("Convergence History:")
            print("Integration Method: {}".format(timeStepMethod))

            xPos = []
            yPos = []
            zPos = []

            for i in range(len(finalPositionHistory)):
                finalPos = finalPositionHistory[i]
                xPos.append(finalPos[0])
                yPos.append(finalPos[1])
                zPos.append(finalPos[2])
                printString = "FinalPosition(m): {:>7.3f} WallTime(s): {:>7.3f} ".format(finalPos, simTimeHistory[i])

                if i > 1: # TODO: Get convergence results into the .csv file
                    ordersOfConvergence, GCI12s, GCI23s, asymptoticChecks, richardsonExtrapVals, uncertainties = convergenceHistory[i-2]
                    printString += " Avg Order: {:>4.2f}, Avg Asymptotic Check: {:>6.3f}".format(mean(ordersOfConvergence), mean(asymptoticChecks))

                print(printString)

            if plot:
                if ax1 == None:
                    ax1 = plt.gca()
                if ax2 == None:
                    ax2 = ax1.twinx()

                ax1.plot(timeStepHistory, zPos, ":D", label=plotLineLabel)
                ax1.set_ylabel("Final Z Coordinate (m)")

                ax2.plot(timeStepHistory, simTimeHistory, "-*", label=plotLineLabel + " Wall Time")
                ax2.set_ylabel("Wall Time (s)")
                
                plt.xscale("log")
                plt.xlabel("Time Step (s)")
                plt.legend()
                plt.tight_layout()

                if showPlot:
                    plt.show()

        while simCount <= simLimit:
            if not adaptiveTimeStepping:
                timeStep /= refinementRatio
                self.simDefinition.setValue(timeStepKey, str(timeStep))
                timeStepHistory.append(timeStep)
                print("Simulation {}, Time step: {}".format(simCount, timeStep))
            else:
                targetError /= refinementRatio
                self.simDefinition.setValue(targetErrorKey, str(targetError))
                timeStepHistory.append(targetError)
                print("Simulation {}, Time step: {}".format(simCount, targetError))

            startTime = time.time()
            flights, _ = self.run()
            flight = flights[0]
            wallTime = time.time() - startTime
            simTimeHistory.append(wallTime)

            finalPositionHistory.append(flight.rigidBodyStates[-1].position)
            print("Final Position: {:1.3f}".format(finalPositionHistory[-1]))

            if len(finalPositionHistory) >= 3:
                # Check whether result is converging
                cV, mV, fV = finalPositionHistory[-3:]
                print("Checking convergence")
                convergResult = checkConvergence(cV, mV, fV, refinementRatio)
                ordersOfConvergence, GCI12s, GCI23s, asymptoticChecks, richardsonExtrapVals, uncertainties = convergResult
                convergenceHistory.append(convergResult)
                directions = ["X", "Y", "Z"]
                for d in range(len(directions)):
                    print("{}-Direction: Order: {:>4.3f}, Asymptotic Check: {:>6.3f}, RichardsonExtrap: {:>7.3f}, Estimated Uncertainty: {:>6.3f}".format(directions[d], ordersOfConvergence[d], asymptoticChecks[d], richardsonExtrapVals[d], uncertainties[d]))
                
                if stopAtConvergence and abs(sum(asymptoticChecks) / len(asymptoticChecks) - 1) < 0.1 and max(asymptoticChecks) - min(asymptoticChecks) < 0.2:
                    print("Simulation Converging Asymptotically")
                    printConvergenceHistory()
                    return timeStepHistory, finalPositionHistory, flight
            
            simCount += 1

        # Output whether convergence was achieved
        if simLimit >= 3:
            print("Asymptotic convergence not reached within {} simulations".format(simLimit))
        else:
            print("Asymptotic convergence impossible to reach with less than 3 iterations (performed {}). Adjust the parameter 'simLimit' to perform more iterations".format(simLimit))

        printConvergenceHistory(ax1, ax2)

        return timeStepHistory, finalPositionHistory, simTimeHistory

    def compareClassicalIntegrationSchemes(self, saveFigure=False, showPlot=True, simLimit=10, integrationSchemes = [ "Euler", "RK2Midpoint", "RK2Heun", "RK4" ], convergenceResultFilePath="convergenceResult.csv"):
        ''' Arguments:
                simConfigFilePath (string)
                saveFigure (Bool)
                convergenceFilePath (string or None) - will overwrite old files

            Outputs:
                Plot
                .csv file (Optional)

            Returns:
                Nothing
        '''

        plt.figure(figsize=(3.5,3))
        ax1 = plt.gca()
        ax2 = plt.twinx()
        
        initTimeStep = float(self.simDefinition.getValue("SimControl.timeStep"))

        # Lists to store results
        timeStepHistory = []
        finalPositionHistories = []
        wallTimeHistory = []

        # Run series of simulations for each integration scheme
        for scheme in integrationSchemes:
            self.simDefinition.setValue("SimControl.timeDiscretization", scheme)
            self.simDefinition.setValue("SimControl.timeStep", str(initTimeStep))
            timeSteps, finalPositions, wallTimes = self.convergeSimEndPosition(showPlot=False, simLimit=simLimit, plotLineLabel=scheme, ax1=ax1, ax2=ax2)
            
            timeStepHistory = timeSteps
            finalPositionHistories.append(finalPositions)
            wallTimeHistory.append(wallTimes)

        print("Simulations complete")

        if convergenceResultFilePath != None:
            print("Writing convergence results to: {}".format(convergenceResultFilePath))

            with open(convergenceResultFilePath, 'w', newline='') as file:
                writer = csv.writer(file)
                
                # Write Column Headers
                headerRow = [ "TimeStep(s)" ]
                for timeStep in range(len(integrationSchemes)):
                    intScheme = integrationSchemes[timeStep]
                    headerRow += [ "{}_FinalX(m)".format(intScheme), "{}_FinalY(m)".format(intScheme), "{}_FinalZ(m)".format(intScheme), "{}_WallTime(s)".format(intScheme) ]
                
                writer.writerow(headerRow)
                
                # Write convergence results, time step by time step
                for timeStep in range(len(timeStepHistory)):
                    row = [ timeStepHistory[timeStep] ]

                    for integrationScheme in range(len(integrationSchemes)):
                        row.append(finalPositionHistories[integrationScheme][timeStep].X)
                        row.append(finalPositionHistories[integrationScheme][timeStep].Y)
                        row.append(finalPositionHistories[integrationScheme][timeStep].Z)
                        row += [ wallTimeHistory[integrationScheme][timeStep] ]

                    writer.writerow(row)

        if saveFigure:
            try:
                plt.savefig("/home/hhstoldt/Documents/flightSimPaper/Figures/Images/AdaptTimeStepConvergence_ConstTimeStep.eps", bbox_inches="tight", pad_inches=0)
            except:
                plt.savefig("C:/Users/rando/Documents/flightSimPaper/Figures/Images/AdaptTimeStepConvergence_ConstTimeStep.eps", bbox_inches="tight", pad_inches=0)

        print("Showing plot")
        if showPlot:
            plt.show()

    def compareAdaptiveIntegrationSchemes(self, saveFigure=False, showPlot=True, integrationSchemes=["RK12Adaptive", "RK23Adaptive", "RK45Adaptive"], simLimit=10, convergenceResultFilePath="adaptiveConvergenceResult.csv"):
        ''' Arguments:
                simConfigFilePath (string)
                saveFigure (Bool)
                convergenceFilePath (string or None) - will overwrite old files

            Outputs:
                Plot
                .csv file (Optional)

            Returns:
                Nothing
        '''

        plt.figure(figsize=(3.5,3))
        ax1 = plt.gca()
        ax2 = plt.twinx()
        
        initErrorTarget = float(self.simDefinition.getValue("SimControl.TimeStepAdaptation.targetError"))
        
        # Lists to store results
        targetErrorHistory = []
        finalPositionHistories = []
        wallTimeHistory = []

        # Run simulations
        for scheme in integrationSchemes:
            self.simDefinition.setValue("SimControl.timeDiscretization", scheme)
            self.simDefinition.setValue("SimControl.TimeStepAdaptation.targetError", str(initErrorTarget))
            timeSteps, finalPositions, wallTimes = self.convergeSimEndPosition(showPlot=False, plotLineLabel=scheme, refinementRatio=2, simLimit=simLimit, ax1=ax1, ax2=ax2)
            
            targetErrorHistory = timeSteps
            finalPositionHistories.append(finalPositions)
            wallTimeHistory.append(wallTimes)

        # Write results to .csv file
        if convergenceResultFilePath != None:
            import csv
            print("Writing convergence results to: {}".format(convergenceResultFilePath))

            with open(convergenceResultFilePath, 'w', newline='') as file:
                writer = csv.writer(file)
                
                # Write Column Headers
                headerRow = [ "TargetError" ]
                for timeStep in range(len(integrationSchemes)):
                    intScheme = integrationSchemes[timeStep]
                    headerRow += [ "{}_FinalX(m)".format(intScheme), "{}_FinalY(m)".format(intScheme), "{}_FinalZ(m)".format(intScheme), "{}_WallTime(s)".format(intScheme) ]
                
                writer.writerow(headerRow)
                
                # Write convergence results, time step by time step
                for timeStep in range(len(targetErrorHistory)):
                    row = [ targetErrorHistory[timeStep] ]

                    for integrationScheme in range(len(integrationSchemes)):
                        row.append(finalPositionHistories[integrationScheme][timeStep].X)
                        row.append(finalPositionHistories[integrationScheme][timeStep].Y)
                        row.append(finalPositionHistories[integrationScheme][timeStep].Z)
                        row += [ wallTimeHistory[integrationScheme][timeStep] ]

                    writer.writerow(row)

        # Save results figure
        if saveFigure:
            try:
                plt.savefig("/home/hhstoldt/Documents/flightSimPaper/Figures/Images/TimeStepConvergence_ConstTimeStep.eps", bbox_inches="tight", pad_inches=0)
            except:
                plt.savefig("C:/Users/rando/Documents/flightSimPaper/Figures/Images/TimeStepConvergence_ConstTimeStep.eps", bbox_inches="tight", pad_inches=0)

        # Show Plot
        plt.xlabel("Target Error")

        if showPlot:
            plt.show()

    def _setUpConfigFileForConvergenceRun(self):
        print("Will attempt to converge final rocket position of simulation: {}".format(self.simDefinition.fileName))
        self.simDefinition.disableDistributionSampling = True # Don't sample from probability distributions while trying to converge a sim

        # Make sure no plots are created every time the sim runs
        self.simDefinition.setValue("SimControl.plot", "None")

        #### Make sure End Condition is a time ####
        endCondition = self.simDefinition.getValue("SimControl.EndCondition")
        if endCondition != "Time":
            print("Running simulation to determine end time")
            # Otherwise run the sim, get the end time and 
            flights, _ = self.run()
            endTime = flights[0].times[-1]
            # set that to the end condition
            print("Setting EndCondition = Time, EndConditionValue = {}".format(endTime))
            self.simDefinition.setValue("SimControl.EndCondition", "Time")
            self.simDefinition.setValue("SimControl.EndConditionValue", str(endTime))
