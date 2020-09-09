from math import sqrt
from statistics import stdev

import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import welch

from TurbulenceModelling import PinkNoiseGenerator

nSamples = 100000

png = PinkNoiseGenerator(nPoles=1)
pinkNoise1 = [0]*nSamples
for i in range(nSamples):
    pinkNoise1[i] = png.getValue()
StdDev1Pole = stdev(pinkNoise1)
print("Pink Noise, 1   pole, Std Dev: {}".format(StdDev1Pole))

png = PinkNoiseGenerator()
pinkNoise = [0]*nSamples
for i in range(nSamples):
    pinkNoise[i] = png.getValue()
StdDev2Pole = stdev(pinkNoise)
print("Pink Noise, 2  poles, Std Dev: {}".format(StdDev2Pole))

png = PinkNoiseGenerator(nPoles=20)
pinkNoise2 = [0]*nSamples
for i in range(nSamples):
    pinkNoise2[i] = png.getValue()
StdDev20Pole = stdev(pinkNoise2)
print("Pink Noise, 20 poles, Std Dev: {}".format(StdDev20Pole))

whiteNoise = [0]*nSamples
for i in range(nSamples):
    whiteNoise[i] = np.random.normal()
print("White Noise, Std Dev: {}".format(stdev(whiteNoise)))

png = PinkNoiseGenerator()
png2 = PinkNoiseGenerator()
png3 = PinkNoiseGenerator()

pinkNoise3DLength = [0]*nSamples
for i in range(nSamples):
    pinkNoise3DLength[i] = sqrt(png.getValue()**2 + png2.getValue()**2 + png3.getValue()**2)

print("3 Components of Pink Noise, 2 poles each, Std Dev: {}".format(stdev(pinkNoise3DLength)))


print("")


# Plot noise
plt.subplot(3,1,1)
plt.plot(pinkNoise)
plt.title("Pink Noise, alpha=5/3, nPoles=2")

plt.subplot(3,1,2)
plt.plot(pinkNoise2)
plt.title("Pink Noise, alpha=5/3, nPoles=20")

plt.subplot(3,1,3)
plt.title("White Noise fit")
plt.plot(whiteNoise)

plt.tight_layout()

# Best fit line for PSD's
def bestFit(f, Pxx, label):
    # Plot line of best fit for the 20 pole data on the log-log plot
    f = f[1:-20]
    Pxx = Pxx[1:-20]
    logx = np.log(f)
    logy = np.log(Pxx)
    coeffs = np.polyfit(logx, logy, deg=1)
    poly = np.poly1d(coeffs)
    yfit = lambda x: np.exp(poly(np.log(x)))
    plt.plot(f, yfit(f), label=label)
    print("{} Equation:".format(label))
    print("{}x + {}".format(*coeffs))

# Plot PSD
plt.figure(figsize=(3.5,3))
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["font.size"] = "10"

f, Pxx = welch(pinkNoise1, fs=20)
peakPower = max(Pxx)
Pxx = [ x / peakPower for x in Pxx ]
plt.plot(f, Pxx, label="Kasdin, 1 term")
# bestFit(f, Pxx, "Pink Noise fit, nPoles=20")

f, Pxx = welch(pinkNoise, fs=20)
peakPower = max(Pxx)
Pxx = [ x / peakPower for x in Pxx ]
plt.plot(f, Pxx, label="Kasdin, 2 terms")

# bestFit(f, Pxx, "Pink Noise fit, nPoles=2")

f, Pxx = welch(pinkNoise2, fs=20)
peakPower = max(Pxx)
Pxx = [ x / peakPower for x in Pxx ]
plt.plot(f, Pxx, label="Kasdin, 20 terms")
# bestFit(f, Pxx, "Pink Noise fit, nPoles=20")


#### White Noise ####
# f, Pxx = welch(whiteNoise, fs=20)
# peakPower = max(Pxx)
# Pxx = [ x / peakPower for x in Pxx ]
# plt.plot(f, Pxx, label="White Noise")
# bestFit(f, Pxx, "White Noise")


#### Plot comparison spectrum: von Karman ####

Lu = 762 # Turbulence length scale (ft)
speeds = [ 100, 250, 500 ] # Aicraft speed (ft/s)
for speed in speeds:
    freq = []
    power = []
    currFreq = 0.01
    while currFreq < 10:
        freq.append(currFreq)
        pwr = 1 / ( 1 + (1.339*(Lu*currFreq/speed))**2) ** (5/6)
        power.append(pwr)
        currFreq *= 1.05

    plt.plot(freq, power, label="von Karman {} m/s".format(speed))

plt.xscale("log")
plt.yscale("log")
# plt.title("Noise PSDs, sampling at 20Hz")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Power Density")

plt.xlim([0.01, 10])
plt.ylim([0.0001, 1])

plt.legend()
plt.tight_layout()

plt.savefig("/home/hhstoldt/Documents/flightSimPaper/Figures/Images/TurbulenceSpectra.eps", bbox_inches="tight", pad_inches=0)

plt.show()
