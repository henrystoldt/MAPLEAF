''' Functions to analyze grid convergence of simulations '''

import os
from math import log
from statistics import mean

from scipy.interpolate import interp1d

#TODO: Create a class to hold convergence results?

############# Functions used internally by checkConvergence ################
def orderOfConvergence(coarseVal, medVal, fineVal, gridRefinementRatio, minOrder=0.5, maxOrder=2):
    ''' Function includes limiter '''
    logBody = (coarseVal - medVal) / (medVal - fineVal)
    if logBody <= 0:
        return minOrder
    else:
        p = log(logBody) / log(gridRefinementRatio)
        
        #Apply Limits
        p = max(minOrder, p)
        p = min(maxOrder, p)
        
        return p

def relError(coarserVal, finerVal):
    return (coarserVal - finerVal) / finerVal

def GCI(coarserVal, finerVal, gridRefinementRatio, observedOrderOfConvergence, factorOfSafety=3, normalizationConstant=None):
    if normalizationConstant == None:
        normalizationConstant = finerVal
    numerator = factorOfSafety * abs((coarserVal - finerVal)/normalizationConstant)
    
    denominator = (gridRefinementRatio**observedOrderOfConvergence - 1)
    return abs(numerator / denominator) * 100

def asymptoticCheck(GCIFiner, GCICoarser, refinementRatio, observedOrderOfConvergence):
    return GCICoarser / (GCIFiner * refinementRatio**observedOrderOfConvergence)

def richardsonExtrap(finerVal, coarserVal, refinementRatio, orderOfConvergence=2):
    return finerVal + (finerVal - coarserVal) / (refinementRatio**orderOfConvergence - 1)

def errorEstimate(order, fineVal, medVal, meshRatio):
    return abs(fineVal - medVal) / (meshRatio**order - 1)

def interpolateDataToCoarseMesh(coarseX, medX, medY, fineX, fineY):
    ''' Uses cubic spline '''
    medVal = interp1d(medX, medY, kind="cubic", fill_value="extrapolate", bounds_error=False)
    fineVal = interp1d(fineX, fineY, kind="cubic", fill_value="extrapolate", bounds_error=False)

    interpMed = [ medVal(x) for x in coarseX ]
    interpFine = [ fineVal(x) for x in coarseX ]

    return interpMed, interpFine

################ Uncertainty Calculation - Pass one of these into checkConvergence as uncertaintyEstimator ###################
def uncertainty_FS(order, fineVal, medVal, meshRatio, formalOrder=2):
    ''' Factor of Safety Method, Xing and Stern '''
    P = order/formalOrder
    if P > 0 and P <= 1:
        FS = 1.6*P + 2.45*(1-P)
    elif P > 1:
        FS = 1.6*P + 14.8*(P-1)
    return FS * errorEstimate(order, fineVal, medVal, meshRatio)

def uncertainty_GCIOR(order, fineVal, medVal, meshRatio, formalOrder=2):
    if order > 1.8 and order < 2.2:
        return 1.25 * errorEstimate(order, fineVal, medVal, meshRatio)
    else:
        return 3 * errorEstimate(order, fineVal, medVal, meshRatio)

def uncertainty_GCI2g(order, fineVal, medVal, meshRatio, formalOrder=2):
    return 3 * errorEstimate(formalOrder, fineVal, medVal, meshRatio)

def uncertainty_GCIglb(order, fineVal, medVal, meshRatio, formalOrder=2):
    return 1.25 * errorEstimate(order, fineVal, medVal, meshRatio)

############# Main Convergence Check Function ################
def checkConvergence(coarseVals, medVals, fineVals, gridRefinementRatio, minConvergOrder=0.5, maxConvergOrder=2, orderTolerance=0.2, \
        asympTolerance=0.05, theoreticalOrderOfConvergence=2, GCINormalizationConstant=None, \
        uncertaintyEstimator=uncertainty_GCI2g, useAvgOrderOfConvergence=False, writeSummaryToConsole=False):
    '''
        #minConvergOrder: limiter on convergence order, recommend 0.5
        #maxConvergOrder: limiter on convergence order, recommend 2
        #orderTolerance: tolerance to set the safety factor in grid convergence 
        #   -> if orderTolerance=0.2, observed orders of 1.8-2.2 get a GCI safety factor of 1.25, others get 3
        #asympTolerance: tolerance for asymptotic convergence, if out 1 +/- asympTolerance range, GCI safety factor is 3
    '''
    def actuallyCheckConvergence(cV, mV, fV, p):
        ''' Pass in single coarse, medium and fine values, as well as the observed order of convergence '''

        # Calculate GCI12 & GCI23
        if GCINormalizationConstant == None:  
            GCI12 = GCI(mV, fV, gridRefinementRatio, p)
            GCI23 = GCI(cV, mV, gridRefinementRatio, p)
        else:
            GCI12 = GCI(mV, fV, gridRefinementRatio, p)
            GCI23 = GCI(cV, mV, gridRefinementRatio, p)

        # Check whether asymptotic
        asympCheck = asymptoticCheck(GCI12, GCI23, gridRefinementRatio, p)
        
        # If results match tolerances, recalculate GCI's with 1.25 factor of safety
        matchesExpectedOrderOfConvergence = (p < theoreticalOrderOfConvergence + orderTolerance and p > theoreticalOrderOfConvergence - orderTolerance)
        isAsymptotic = abs(asympCheck - 1) < asympTolerance
        if matchesExpectedOrderOfConvergence and isAsymptotic:
            GCI12 = GCI(mV, fV, gridRefinementRatio, p, factorOfSafety=1.25)
            GCI23 = GCI(cV, mV, gridRefinementRatio, p, factorOfSafety=1.25)
            
        # Calculate richardson value and uncertainty estimate
        richardsonVal = richardsonExtrap(fV, mV, gridRefinementRatio, orderOfConvergence=p)
        uncertaintyEstimate = uncertaintyEstimator(p, fV, mV, gridRefinementRatio, theoreticalOrderOfConvergence)

        # Return results
        return [p, GCI12, GCI23, asympCheck, richardsonVal, uncertaintyEstimate]

    # If single values are passed in for coarse/med/fine values, put them in 1-length lists
    try:
        t = iter(coarseVals)
        t = iter(medVals)
        t = iter(fineVals)
    except TypeError:
        # Items passed in are not lists, they are single values
        # So make them lists
        coarseVals = [ coarseVals ]
        medVals = [ medVals ]
        fineVals = [ fineVals ]

    # Calculate local and average orders of convergence
    ordersOfConvergence = [ orderOfConvergence(cV, mV, fV, gridRefinementRatio, minOrder=minConvergOrder, maxOrder=maxConvergOrder) for cV, mV, fV in zip(coarseVals, medVals, fineVals) ]
    avgP = mean(ordersOfConvergence)

    # Create empty lists to hold convergence data for each point
    GCI12s = []
    GCI23s = []
    asymptoticChecks = []
    richardsonExtrapVals = []
    uncertainties = []
    # Call actuallyCheckConvergence for each point
    for cV, mV, fV, aP in zip(coarseVals, medVals, fineVals, ordersOfConvergence):
        # Call actualluCheckConvergence
        if useAvgOrderOfConvergence:
            # Pass in average order of convergence
            p, G1, G2, asymp, rVal, u = actuallyCheckConvergence(cV, mV, fV, avgP)
        else:
            # Pass in local order of convergence
            p, G1, G2, asymp, rVal, u = actuallyCheckConvergence(cV, mV, fV, aP)
        
        # Store results
        GCI12s.append(G1)
        GCI23s.append(G2)
        asymptoticChecks.append(asymp)
        richardsonExtrapVals.append(rVal)
        uncertainties.append(u)

    ### Output/Return results ###
    if writeSummaryToConsole:
        print("Avg observed order of convergence: {}".format(mean(ordersOfConvergence)))
        print("Avg GCI(Fine): {}%".format(mean(GCI12s)))
        print("Avg GCI(Medium): {}%".format(mean(GCI23s)))
        print("Avg asymptotic check: {}".format(mean(asymptoticChecks)))
        print("Avg uncertainty: +/-{}".format(mean(uncertainties)))


    # If there was only a single data point passed in, return values instead of lists
    if len(ordersOfConvergence) == 1:
        ordersOfConvergence = ordersOfConvergence[0]
        GCI12s = GCI12s[0]
        GCI23s = GCI23s[0]
        asymptoticChecks = asymptoticChecks[0]
        richardsonExtrapVals = richardsonExtrapVals[0]
        uncertainties = uncertainties[0]
    return [ ordersOfConvergence, GCI12s, GCI23s, asymptoticChecks, richardsonExtrapVals, uncertainties ]

############# Calculate & Plot Results ################
def plotConvergence(coarseX, coarseY, medX, medY, fineX, fineY, \
    minConvergOrder=0.5, maxConvergOrder=2, writeSummaryToConsole=True, useAvgOrderOfConvergence=False, refinementRatio=1.5, \
    xLabel=r"Plate location (m)", yLabel=r"Wall Heat Flux (W)", xLim=None, yLim=None, showRichardson=True, showUncertainty=True, figSize=(6,4), \
    saveToDirectory=None, overwrite=False, showPlot=True, lineLabelPrefix="", lineLabels=["Coarse", "Medium", "Fine"], lineColor="k", \
    createZoomedInset=False, insetZoom=20, insetLoc=4, insetXLim=[1.16, 1.26], insetYLim=[10.25, 10.75], mark_insetLoc1=1, mark_insetLoc2=3, \
    resultsAxes=None, resultsAxins=None, resultsFig=None, convergenceAxes=None, convergenceFig=None, uncertaintyAxes=None, uncertaintyFig=None):    
    '''
        Saves .png/.eps/.pdf figures in saveToDirectory folder, if one is specified
        Show figures if showPlot is true

        Inputs:
            x/yLim:         (List or None) List should be lower, then upper limit for x or y Axis ex: [0, 1]
            x/yLabel:       (string or None)

        Inputs are organized by line:
            Data
            Convergence Settings
            Plotting Settings
            Plotting Settings
            Zoomed inset settings
            Fig/Axes inputs (to have lines plotted on existing graphs)
    '''
    
    import matplotlib.pyplot as plt
    LLP = lineLabelPrefix

    # Make sure we have data at the same x-locations
    interpMedY, interpFineY = interpolateDataToCoarseMesh(coarseX, medX, medY, fineX, fineY)

    # Calculate mesh convergence
    observedOrder, GCI12, GCI23, asymptCheck, richardsonVal, uncertainties = checkConvergence(coarseY, interpMedY, interpFineY, refinementRatio, \
        minConvergOrder=minConvergOrder, maxConvergOrder=maxConvergOrder, uncertaintyEstimator=uncertainty_FS, \
        writeSummaryToConsole=writeSummaryToConsole, useAvgOrderOfConvergence=useAvgOrderOfConvergence)

    print("\nPlotting Data\n")

    # Determine whether to save figures
    saveFigures = False
    if saveToDirectory != None:
        if os.path.isdir(saveToDirectory):
            saveFigures = True
        else:
            print("Error: {} is not a directory. Plots will not be saved.".format(saveToDirectory))

    ######## Figure 1 - Results ########
    if resultsAxes == None or (resultsFig == None and saveFigures):
        resultsFig, resultsAxes = plt.subplots(figsize=figSize)
    
    # Plot uncertainty range
    if showUncertainty:
        maxEst = [ f + e for f,e in zip(interpFineY, uncertainties)]
        minEst = [ f - e for f,e in zip(interpFineY, uncertainties)]
        resultsAxes.fill_between(coarseX, minEst, maxEst, facecolor=lineColor, alpha=0.15, antialiased=True, label=LLP+"Uncertainty")
    # Plot coarse/med/fine lines
    coarseLineStyle = "-."
    medLineStyle = "--"
    fineLineStyle = ":"
    resultsAxes.plot(coarseX, coarseY, coarseLineStyle, label=LLP+lineLabels[0], color=lineColor, alpha=0.5)
    resultsAxes.plot(medX, medY, medLineStyle, label=LLP+lineLabels[1], color=lineColor, alpha=0.5)
    resultsAxes.plot(fineX, fineY, fineLineStyle, label=LLP+lineLabels[2], color=lineColor, lw=3)
    if showRichardson:
        resultsAxes.plot(coarseX, richardsonVal, lineColor, label=LLP+"Richardson")

    if yLabel != None:
        resultsAxes.set_ylabel(yLabel)
    if yLim != None:
        resultsAxes.set_ylim(top=yLim[1], bottom=yLim[0])

    # Create zoomed inset
    if createZoomedInset:
        from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes, mark_inset

        if resultsAxins == None:
            resultsAxins = zoomed_inset_axes(resultsAxes, 20, loc=4) # zoom-factor: 2.5, location: upper-left

        if showUncertainty:
            resultsAxins.fill_between(coarseX, minEst, maxEst, facecolor=lineColor, alpha=0.15, antialiased=True)
        resultsAxins.plot(coarseX, coarseY, coarseLineStyle, alpha=0.5)
        resultsAxins.plot(medX, medY, medLineStyle, alpha=0.5)
        resultsAxins.plot(fineX, fineY, fineLineStyle, lw=3)
        if showRichardson:
            resultsAxins.plot(coarseX, richardsonVal, lineColor)

        resultsAxins.set_xlim(insetXLim[0], insetXLim[1]) # apply the x-limits
        resultsAxins.set_ylim(insetYLim[0], insetYLim[1]) # apply the y-limits

        # plt.xticks(visible=False)
        # plt.yticks(visible=False)

        plt.setp(resultsAxins.get_xticklabels(), visible=False)
        plt.setp(resultsAxins.get_yticklabels(), visible=False)
        mark_inset(resultsAxes, resultsAxins, loc1=mark_insetLoc1, loc2=mark_insetLoc2, fc="none", ec="0.5")

    #### Set up figure 2 - convergence properties ####
    if convergenceAxes == None or (convergenceFig == None and saveFigures):
        convergenceFig, convergenceAxes = plt.subplots(figsize=figSize)
    convergenceAxes.plot(coarseX, observedOrder, fineLineStyle, label=LLP+"Observed order of convergence")
    convergenceAxes.plot(coarseX, asymptCheck, color=lineColor, label=LLP+"Asymptotic check")
    convergenceAxes.set_ylim(top=5, bottom=0)

    #### Set up figure 3 - Uncertainty ####
    if uncertaintyAxes == None or (uncertaintyAxes == None and saveFigures):
        uncertaintyFig, uncertaintyAxes = plt.subplots(figsize=figSize)
    uncertaintyAxes.plot(coarseX, uncertainties, color=lineColor, label=LLP+"Uncertainty")

    # The embedded axes output warnings about tight_layout
    # for fig in [ resultsFig, convergenceFig, uncertaintyFig ]:
    #     fig.tight_layout()

    # Create legends
    for ax in [ resultsAxes, convergenceAxes, uncertaintyAxes ]:
        ax.legend()

    if xLim != None:
        resultsAxes.set_xlim(left=xLim[0], right=xLim[1])
        convergenceAxes.set_xlim(left=xLim[0], right=xLim[1])
        uncertaintyAxes.set_xlim(left=xLim[0], right=xLim[1])

    if xLabel != None:
        resultsAxes.set_xlabel(xLabel)
        convergenceAxes.set_xlabel(xLabel)
        uncertaintyAxes.set_xlabel(xLabel)

    #### Save / Show / Return Figures ####
    if saveFigures:
        saveFigureAndPrintNotification("Results", resultsFig, saveToDirectory, overwrite=overwrite)
        saveFigureAndPrintNotification("Convergence", convergenceFig, saveToDirectory, overwrite=overwrite)
        saveFigureAndPrintNotification("Uncertainty", uncertaintyFig, saveToDirectory, overwrite=overwrite)

    if showPlot:
        plt.show()

    return resultsAxes, resultsFig, resultsAxins, convergenceAxes, convergenceFig, uncertaintyAxes, uncertaintyFig

def saveFigureAndPrintNotification(fileName, figure, saveToDirectory, overwrite=False, pngVersion=True, epsVersion=True, pdfVersion=True, printStatementPrefix=""):
        def saveFigure(filePath):
            if overwrite or not os.path.exists(filePath):
                figure.savefig(filePath)
                print("{}Saved Image: {}".format(printStatementPrefix, filePath))
            elif not overwrite and os.path.exists(filePath):
                print("{}WARNING: Did not save image: {} - file already exists".format(printStatementPrefix, filePath))
        
        def getNoExtensionFilePath(filePath):
            noExtensionPath = filePath

            # Remove extension if it exists
            if '.' in filePath:
                dotIndex = filePath.rfind('.')
                noExtensionPath = filePath[:dotIndex]

            return noExtensionPath

        filePath = os.path.join(saveToDirectory, fileName)
        noExtensionPath = getNoExtensionFilePath(filePath)

        # Save each desired version of the figure
        if pngVersion:
            pngFilePath = noExtensionPath + ".png"
            saveFigure(pngFilePath)

        if epsVersion:
            epsFilePath = noExtensionPath + ".eps"
            saveFigure(epsFilePath)

        if pdfVersion:
            pdfFilePath = noExtensionPath + ".pdf"
            saveFigure(pdfFilePath)