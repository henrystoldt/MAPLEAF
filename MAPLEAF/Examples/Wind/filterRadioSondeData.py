''' 
    Takes raw radiosonde data file from NSGRA-2, removes data that's not required by the simulator, outputs _filtered version. 
    This _filtered file is the file MAPLEAF expects when using radiosonde mean wind models
'''

if __name__ == "__main__":
    # Adjust input (unfiltered) file location here
    filePath = "MAPLEAF/Examples/Wind/RadioSondeEdmonton.txt"

    with open(filePath, 'r') as file:
        lines = file.readlines()

    output = []
    for line in lines:
        line = line.strip()
        isHeader = line[0] == '#'
        hasWindData = line[-5:] != "-9999"
        hasAltitude = line[16:21] != "-9999"
        if isHeader:
            output.append(line + "\n")
        elif hasWindData and hasAltitude:
            # Only keep required columns
            altitude = line[16:21]
            windSpeed = line[-11:]
            output.append("{} {}\n".format(altitude, windSpeed))


    outputFilePath = filePath[:-4] + "_filtered.txt"
    with open(outputFilePath, 'w+') as file:
        file.writelines(output)
