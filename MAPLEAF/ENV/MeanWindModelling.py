''' Modeling of the mean / average component of wind velocity '''

import abc
import random
from math import cos, radians, sin

import numpy as np
import pandas as pd
from scipy.stats import rv_histogram

from MAPLEAF.IO import SubDictReader, defaultConfigValues, getAbsoluteFilePath
from MAPLEAF.Motion import Vector, linInterp

__all__ = [ "meanWindModelFactory", "ConstantWind", "Hellman", "InterpolatedWind" ]

# Mean Wind Model Abstract Base Class / Interface Definition
class MeanWindModel(abc.ABC):
    ''' Defines the interface for Mean Wind Models '''

    @abc.abstractmethod
    def getMeanWind(self, AGLAltitude):
        pass 

# Mean Wind Model Factory
def meanWindModelFactory(simDefinition=None, silent=False) -> MeanWindModel:
    ''' Instantiates a mean wind model '''

    if simDefinition == None:
        constWind = Vector(defaultConfigValues["Environment.ConstantMeanWind.velocity"])
        return ConstantWind(constWind)

    envReader = SubDictReader('Environment', simDefinition)
    meanWindModel = None
    meanWindModelType = envReader.getString("MeanWindModel")

    if meanWindModelType == "Constant":
        meanGroundWind = envReader.getVector("ConstantMeanWind.velocity")
        if not silent:
            print("Constant ground wind: {:1.2f} m/s".format(meanGroundWind))

        meanWindModel = ConstantWind(meanGroundWind)

    elif meanWindModelType in [ "SampledGroundWindData", "Hellman" ]:
        def getLocationSampledGroundWindVel():
            #### Get list of, and weights of, locations to sample wind rose from
            GroundWindLocationsSampled = envReader.getString("SampledGroundWindData.locationsToSample").split()
            locationsSampled = []
            locationWeights = []
            # Parse into locations (1st, 3rd, 5th... values) and weights (2nd, 4th, 6th... values)
            for i in range(0, len(GroundWindLocationsSampled), 2):
                locationsSampled.append(GroundWindLocationsSampled[i])
                locationWeights.append(float(GroundWindLocationsSampled[i+1]))
            launchMonth = envReader.getString("SampledGroundWindData.launchMonth")

            # Sample wind rose(s)
            sampler = WindRoseDataSampler(silent=silent)
            meanGroundWind = sampler.sampleWindRoses(locationsSampled, locationWeights, launchMonth)

            # Output choices parsed from input file and resulting wind
            if not silent:
                if simDefinition.monteCarloLogger != None:
                    simDefinition.monteCarloLogger.log("Sampling ground winds from: {}".format(locationsSampled))
                    simDefinition.monteCarloLogger.log("Sampling weights for each location: {}".format(locationWeights))
                    simDefinition.monteCarloLogger.log("Sampling wind distribution from month of: {}".format(launchMonth))
                    simDefinition.monteCarloLogger.log("Sampled mean ground wind: {:1.2f} m/s".format(meanGroundWind))
                else:
                    print("Sampling ground winds from: {}".format(locationsSampled))
                    print("Sampling weights for each location: {}".format(locationWeights))
                    print("Sampling wind distribution from month of: {}".format(launchMonth))
                    print("Sampled mean ground wind: {:1.2f} m/s".format(meanGroundWind))

            return meanGroundWind

        ### Create and return appropriate wind vs altitude model ###
        if meanWindModelType == "SampledGroundWindData":
            meanGroundWind = getLocationSampledGroundWindVel()
            if not silent:
                print("Wind is not a function of altitude")
            meanWindModel = ConstantWind(meanGroundWind)

        elif meanWindModelType == "Hellman":
            groundWindModel = envReader.getString("Hellman.groundWindModel")

            # Get ground wind
            if groundWindModel == "Constant":
                meanGroundWind = envReader.getVector("ConstantMeanWind.velocity")
            elif groundWindModel == "SampledGroundWindData":
                meanGroundWind = getLocationSampledGroundWindVel()

            HellmanAlphaCoeff = envReader.getFloat("Hellman.alphaCoeff")
            HellmanAltitudeLimit = envReader.getFloat("Hellman.altitudeLimit")

            if not silent:
                print("Constant ground wind: {:1.2f} m/s".format(meanGroundWind))
                print("Wind vs. altitude governed by Hellman law: v2 = v1*(z2/10)^a where a = {}".format(HellmanAlphaCoeff))
                print("Hellman law is taken to apply up to an altitude of {} m AGL".format(HellmanAltitudeLimit))

            meanWindModel = Hellman(meanGroundWind, HellmanAltitudeLimit, HellmanAlphaCoeff)
    
    elif meanWindModelType == "CustomWindProfile":
        meanWindProfileFilePath = envReader.getString("CustomWindProfile.filePath")
        meanWindModel = InterpolatedWind(windFilePath=meanWindProfileFilePath)

    elif meanWindModelType == "SampledRadioSondeData":
        # Get locations and location weights
        locationsAndWeights = envReader.getString("SampledRadioSondeData.locationsToSample").split()
        locations = []
        weights = []
        for i in range(0, len(locationsAndWeights), 2):
            locations.append(locationsAndWeights[i])
            weights.append(float(locationsAndWeights[i+1]))
            
        locationASLAltitudes = [ float(x) for x in envReader.getString("SampledRadioSondeData.locationASLAltitudes").split() ]

        # Get launch month
        launchMonth = envReader.getString("SampledRadioSondeData.launchMonth")

        # Get random seed (if provided)
        radioSondeRandomSeed = envReader.tryGetString("SampledRadioSondeData.randomSeed")

        # Select and parse radio sonde profile
        sampler = RadioSondeDataSampler(silent=silent)
        altitudes, windVectors = sampler.getRadioSondeWindProfile(locations, weights, locationASLAltitudes, launchMonth, radioSondeRandomSeed)
        
        meanWindModel = InterpolatedWind(windAltitudes=altitudes, winds=windVectors)

    else:
        raise ValueError("Unknown MeanWindModel: {}. Please see SimDefinitionTemplate.txt for available options.".format(meanWindModelType))

    return meanWindModel

#### Actual Mean Wind Model classes - these are instantiated by meanWindModelFactory ####
class ConstantWind(MeanWindModel):
    ''' Defines a constant wind speed at all altitudes  '''
    
    def __init__(self, wind):
        self.wind = wind

    def getMeanWind(self, AGLAltitude):
        return self.wind

class Hellman(MeanWindModel):
    '''
        Uses the Hellman law to scale a ground velocity using a power law as the atmospheric boundary layer is exited.
        Low-altitude only. https://en.wikipedia.org/wiki/Wind_gradient -> Engineering Section.

    '''
    def __init__(self, groundMeanWind=Vector(0,0,0), altitudeLimit=1500, hellmanAlphaCoeff=0.14):
        '''
            Arguments:
                groundMeanWind: Wind Vector at ground level (~10m above surface), in m/s
                altitudeLimit: Altitude above which the power law transitions into a constant profile (m, AGL)
                hellmanAlphaCoeff: Alpha in the Hellman model (1/7 is a common first guess, but can vary widely depending on terrain)
        '''
        self.groundMeanWind = groundMeanWind
        self.altitudeLimit = altitudeLimit
        self.hellmanAlphaCoeff = hellmanAlphaCoeff

    def getMeanWind(self, AGLAltitude):
        # Limit velocity scaling to up to the specified altitude limit
        HellmanAltitude = max(min(self.altitudeLimit, AGLAltitude), 10)
        # Assume initial winds come from a height of 10m
        return self.groundMeanWind * (HellmanAltitude/10)**self.hellmanAlphaCoeff

class InterpolatedWind(MeanWindModel):
    def __init__(self, windAltitudes=[], winds=[], windFilePath=None):
        '''
            Arguments:
                windAltitudes: list of altitudes at which wind vectors will be provided (m AGL)
                winds: list of wind vectors, matching the ordering of windAltitudes (m/s)

                windFilePath: string path to a text file containing a table, formatted similarly to the following:
                    AGlAltitude(m) WindX(m/s) WindY(m/s) WindZ(m/s)
                    a1              wx1         wy1     wz1
                    a2              wx2         wy2     wz2

            Notes:
                Provide either windAltitudes AND winds, OR windFilePath. If all are provided, windAltitudes/winds will be used
        '''
        if len(windAltitudes) + len(winds) > 0:
            self.windAltitudes = windAltitudes
            self.winds = winds

        elif windFilePath != None:
            self._readCustomWindProfile(windFilePath)

        else:
            raise ValueError("Incomplete initialization info provided. Provide either windAltitudes AND winds, OR windFilePath")

    def _readCustomWindProfile(self, filePath):
        windProfile = np.loadtxt(filePath, skiprows=1)
        self.windAltitudes = windProfile[:, 0]
        self.winds = windProfile[:, 1:]

    def getMeanWind(self, AGLAltitude):
        return Vector(*linInterp(self.windAltitudes, self.winds, AGLAltitude))

# Wind Data readers/samplers
class WindRoseDataSampler():
    ''' 
        Parses wind rose data files from Iowa State University's Environmental Mesonet.
    '''

    def __init__(self, silent=False):
        self.silent = silent

        # TODO: Generalize extreme winds outside Medecine Hat
        self.extremeWinds = [ 25, 31, 50 ] # mph = [ 40, 50, 80 ] km/h
        self.extremeWinds = [ x * 0.44704 for x in self.extremeWinds ] # convert to m/s
        self.extremeWindProbabilities = [ 0.01, 0.004, 0 ] # Decimal, chance of being at or higher than given wind speed, based on hourly peak gust data for Medecine Hat, AB

    def sampleWindRoses(self, locationsSampled, locationWeights, launchMonth):
        # Choose a wind rose based on the location weights
        averagedWindRose = self._getAveragedWindRose( Months=[launchMonth], locations=locationsSampled, locationWeights=locationWeights )
        speedrvHist = self._createWindSpeedCDF(averagedWindRose)
        
        # Choose a wind speed
        windSpeed = speedrvHist.ppf(random.random())
        
        # Construct heading probabilities based on that wind speed, then choose a wind heading
        heading_rvHist = self._createHeadingCDF(averagedWindRose, windSpeed=windSpeed)
        windHeading = heading_rvHist.ppf(random.random())
        windDirectionVector = _convertWindHeadingToXYPlaneWindDirection(windHeading)
        meanGroundWind = windDirectionVector * windSpeed

        return meanGroundWind

    def _readWindRose(self, filePath):
        '''
            Currently not general, will only work with Wind Rose data from Iowa State University - Iowa Environmental Mesonet
            http://mesonet.agron.iastate.edu/sites/windrose.phtml?network=WI_ASOS&station=CWA
        '''
        # Read data (Header = Direction,     Calm, 2.0  4.9, 5.0  6.9, 7.0  9.9,10.0 14.9,15.0 19.9,    20.0+)
        windRose = pd.read_csv(filePath, delimiter=",", comment="#")
        # Split direction column into min and max directions: ex. "000-010"
        directionMinMax = windRose["Direction"].str.split("-", n=1, expand=True)
        windRose.drop(columns=["Direction"], inplace=True)
        # Convert each of the split string data to floats
        directionMinMax = directionMinMax.astype(float)
        # Add new direction columns
        windRose["direction"] = (directionMinMax[0] + directionMinMax[1]) / 2
        # Set first direction to 0 degrees instead of 5 (assumed incorrect in windrose file)
        windRose.iloc[0,-1] = 0

        nameMap = {
            "     Calm": 1.0,
            " 2.0  4.9": 3.5,
            " 5.0  6.9": 6.0,
            " 7.0  9.9": 8.5,
            "10.0 14.9": 12.5,
            "15.0 19.9": 17.5,
            "    20.0+": 23.5
        }
        windRose.rename(mapper=nameMap, inplace=True, axis='columns')

        # Replace empty values in the first column with zeros
        windRose.replace(r'^\s*$', 0.0, regex=True, inplace=True)
        windRose[1.0] = windRose[1.0].astype(float)

        return windRose

    def _getAveragedWindRose(self, Months=["May"], MonthWeights=[1.0], locations=["MedecineHat"], locationWeights=[1.0]):
        windRose = self._readWindRose(getAbsoluteFilePath("MAPLEAF/Examples/Wind/WindroseAprMedecineHat.txt"))*0.0 # Initialize as zero
        for i in range(len(Months)):
            for a in range(len(locations)):
                windRoseFilePath = "MAPLEAF/Examples/Wind/Windrose{}{}.txt".format(Months[i], locations[a])
                windRoseFilePath = getAbsoluteFilePath(windRoseFilePath)
                windRose += self._readWindRose(windRoseFilePath) * MonthWeights[i] * locationWeights[a]

        return windRose.copy()

    def _createWindSpeedCDF(self, windRose):
        '''Calculates wind speed CDF from a windrose (wind rose can be generated using self.readWindRose())'''
        # Sum over headings to get probabilities for each speed
        likelyhoodBySpeed = windRose.sum()
        likelyhoodBySpeed.drop(index=["direction"], inplace=True)

        # Normalize to 0-1 probability range
        totalProb = likelyhoodBySpeed.sum()
        likelyhoodBySpeed = list(likelyhoodBySpeed / totalProb)

        # Bin boundaries
        cdfX = [ x * 0.44704 for x in [ 0, 2, 5, 7, 10, 15, 20, 23.5 ] ] # Convert to m/s from mph
        likelyhoodBySpeed[-1] -= sum(self.extremeWindProbabilities)
        cdfX += self.extremeWinds
        likelyhoodBySpeed += self.extremeWindProbabilities

        return rv_histogram((likelyhoodBySpeed, cdfX))

    def _createHeadingCDF(self, windRose, windSpeed=None):
        '''
            Calculates wind heading CDF from a windrose (wind rose can be generated using self.readWindRose())
            Pass in wind speed in m/s
        '''
        windRose = windRose.set_index("direction")
        windRose.loc[360] = windRose.loc[0]
        windRose.drop(index=[0], inplace=True)
        headings = list(windRose.index)
        headings.insert(0,0)
        
        if windSpeed == None:
            # Sum over all speeds to get overall probabilities for each heading
            likelyhoodByHeading = list(windRose.sum(axis=1))
        else:
            # Eliminate the empty first column, replace it with data from the 3.5 mph column
            windRose.iloc[:,0] = windRose.iloc[:,1]
            windRose.rename(columns={1.0:0.0}, inplace=True)

            # Convert from mph to m/s string
            windRose = windRose.rename(columns=lambda c: str(c * 0.44704))
            
            try:
                # If windSpeed exists already, use it
                likelyhoodByHeading = list(windRose[str(windSpeed)])
            except KeyError:
                #### Linearly interpolate to find wind speed probabilities at the given heading ####
                # Create new empty column at desired wind speed
                windRose[str(windSpeed)] = np.nan
                # Convert back to float names
                windRose = windRose.rename(columns=lambda c: float(c))
                # Sort the new column into position
                windRose = windRose.reindex(sorted(windRose.columns), axis=1)
                # Interpolate
                windRose = windRose.interpolate(method="index", axis=1, interpolate=True)            
                # Grab the new column as a list
                likelyhoodByHeading = list(windRose[windSpeed])

        #TODO: Need to de-rotate these headings by 5 degrees?
        return rv_histogram((likelyhoodByHeading, headings))

class RadioSondeDataSampler():
    ''' 
        Parses data from radio sonde data files from IGRA-2 after they've been post-processed by MAPLEAF/Examples/Wind/filterRadioSondeData.py
        IGRA-2 format info: https://www1.ncdc.noaa.gov/pub/data/igra/data/igra2-data-format.txt
    '''

    def __init__(self, silent=False, monteCarloDataLogger=None):
        self.silent = silent
        self.monteCarloDataLogger = monteCarloDataLogger

    def getRadioSondeWindProfile(self, locations=["Edmonton", "Glasgow"], locationWeights=[0.48, 0.52], locationASLAltitudes=[710, 638], launchMonth=None, randomSeed=None):
        '''
            Arguments:
                locations:              list of location names. Names must match those in radio sonde data file names (ex. "Edmonton" -> "RadioSondeEdmonton_filtered.txt")
                locationWeights:        list of numeric values, must add to one. Order should match list of locations.
                locationASLAltitudes:   list of ASL altitudes (m). Order should match list of locations.
                launchMonth:            3-letter string ("Jan", "Feb", "Mar", etc...) indicating month from which data is desired. Otherwise "Yearly" or None to select randomly from all months
                randomSeed:             Pass in the same random seed to pick the same radio sonde profile

            Returns:
                altitudes:              List of numeric altitudes (meters AGL) 
                windVectors:            Corresponding list of wind vectors (m/s)
        '''
        # Set random seed if provided
        random.seed(randomSeed)

        # Pick a location
        selectedLocation = random.choices(locations, locationWeights, k=1)[0]

        locationASLAltitude = locationASLAltitudes[locations.index(selectedLocation)]
        # Load radiosonde data for that location
        radioSondeFilePath = "MAPLEAF/Examples/Wind/RadioSonde{}_filtered.txt".format(selectedLocation)
        # Convert to absolute file path
        radioSondeFilePath = getAbsoluteFilePath(radioSondeFilePath)
        
        # Read datasets from fille
        if launchMonth != "Yearly":
            datasetStartLines, data = self._readRadioSondeDataFile(radioSondeFilePath, filterByMonth=launchMonth)
        else:
            datasetStartLines, data = self._readRadioSondeDataFile(radioSondeFilePath)

        # Pick one of it's datasets
        selectedDataset = random.randint(0, len(datasetStartLines)-1)
        
        # selectedDataset is an array of strings, header is a string
        # Example header: #CAM00071119 2000 01 01 12 1117   75 ncdc-gts ncdc-gts  535475 -1141083
        header, radioSondeData = self._extractRadioSondeDataSet(datasetStartLines, data, selectedDataset)
        stationID, year, month, day, hour = self._parseRadioSondeHeader(header)

        if not self.silent:
            if self.monteCarloDataLogger != None:
                self.monteCarloDataLogger.log("Wind data from radio sonde dataset: {}, stationID: {}, {}, {}, {}, {}".format(selectedLocation, stationID, year, month, day, hour))
            else:
                print("Wind data from radio sonde dataset: {}, stationID: {}, {}, {}, {}, {}".format(selectedLocation, stationID, year, month, day, hour))
        
        # Turn data from strings into a numpy array
        altitudes, windVectors = self._parseRadioSondeData(radioSondeData, locationASLAltitude)
        return altitudes, windVectors

    def _readRadioSondeDataFile(self, radioSondeFilePath, filterByMonth=None):
        '''
            Arguments:
                radioSondeFilePath: String File Path to radio sonde data file
                filterByMonth: 3-letter acronym of month from which radio sonde profiles are desired. Pass in "Yearly" or None to select all months

            Returns: datasetStartlines, data
                datasetStartlines is a list containing the numbers on which each data set starts - lines not from filterByMonth are ommitted
                data is a list of strings, each being a line of the original file
        '''
        
        with open(radioSondeFilePath, 'r') as file:
            data = file.readlines()

        # Yearly means no filter
        if filterByMonth == "Yearly":
            filterByMonth = None

        # Get Month number
        if filterByMonth != None:
            monthList = [ "noZerothMonth", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" ]
            monthNumber = monthList.index(filterByMonth)
        
        datasetStartLines = []
        i = 0
        while i < len(data):
            # Radio sonde data header lines start with #
            # Check that profile month matches the desired month
            if data[i][0] == "#" and (filterByMonth == None or int(data[i].split()[2]) == monthNumber):
                datasetStartLines.append(i)
            i += 1
        
        return datasetStartLines, data

    def _extractRadioSondeDataSet(self, datasetStartLines, data, desiredDataSetNumber):
        ''' Returns header string and list of strings representing data for chosen dataset '''
        startLine = datasetStartLines[desiredDataSetNumber]
        header = data[startLine]

        # Collect the data, starting with the line after the header
        i = startLine + 1
        dataset = []

        # Stop at next header or end of file
            # Don't go to next dataset start  line, because the start lines from other months are potentially excluded from that list, but still need to be respected here
        while i < len(data) and data[i][0] != "#":
            dataset.append(data[i])
            i += 1

        return header, dataset

    def _parseRadioSondeHeader(self, headerString):
        '''
            Returns stationID, Year, Month, Day, Hour
            Example header: #CAM00071119 2000 01 01 12 1117   75 ncdc-gts ncdc-gts  535475 -1141083
            Further data format info: https://www1.ncdc.noaa.gov/pub/data/igra/data/igra2-data-format.txt
        '''
        
        data = headerString.split()
        stationID = data[0][1:]
        year = int(data[1])
        month = int(data[2])
        day = int(data[3])
        hour = int(data[4])
        return stationID, year, month, day, hour

    def _parseRadioSondeData(self, data, locationASLAltitude):
        '''
            Returns:
                altitudes: List of AGL altitudes (m)
                windVectors: List of corresponding mean wind vectors

            Both returned lists are sorted from lowest to highest altitude

            Further info about radio sonde input data at: https://www1.ncdc.noaa.gov/pub/data/igra/data/igra2-data-format.txt
            Input data expects that radio sonde data has already been reduced to just the altitude, wind heading and wind speed columsn using MAPLEAF/Examples/Wind/filterRadioSondeData.py
        '''
        result = []
        for line in data:
            result.append([ float(x) for x in line.split() ])
        
        result = np.array(result)
        result[:,2] = result[:,2] / 10 # Convert to m/s from m/s *10

        # Sort rows by altitude
        result = result[result[:,0].argsort()]
        altitudes = result[:,0] - locationASLAltitude # Convert altitudes to AGL

        # Turn heading and speed into a velocity vector
        windVectors = []
        for i in range(len(result)):
            windDirection = _convertWindHeadingToXYPlaneWindDirection(result[i,1])
            windVec = windDirection*result[i,2]
            windVectors.append(np.array((windVec.X, windVec.Y, windVec.Z)))

        return altitudes, windVectors

def _convertWindHeadingToXYPlaneWindDirection(heading, AngleBetweenYAxisandNorth=0):
    '''
        Reminder: wind rose indicates where wind is blowing FROM
        This function outputs a vector indicating where wind is blowing TOWARDS
        Heading angle of 90 degrees is defined to point east
        AnglBetweenYAxis and North defined in the same way, if that's 90 degrees, the Y axis is pointing east
    '''
    totalAngleFromYAxis = heading + AngleBetweenYAxisandNorth
    y = cos(radians(totalAngleFromYAxis))
    x = sin(radians(totalAngleFromYAxis))
    headingVector = Vector(x, y, 0)
    windDirectionVector = -1 * headingVector # Wind is in opposite direction of heading
    return windDirectionVector
