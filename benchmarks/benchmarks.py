# Write the benchmarking functions here.
# See "Writing benchmarks" in the asv docs for more information.
import os
import sys
from pathlib import Path

from MAPLEAF.IO import SimDefinition
from MAPLEAF.SimulationRunners import Simulation
from MAPLEAF.Motion import Vector, Quaternion

def setup():
    os.chdir(Path(__file__).parent.parent)

class Time:
    def time_simulationDefinitionParsing(self):
        simDef = SimDefinition("test/test_IO/testSimDefinition.mapleaf")

    def time_derivedSimulationDefinitionParsing(self):
        simDef = SimDefinition("test/test_IO/testDerivedDicts.mapleaf")

    def time_initializeSimulationAndRocket(self):
        sim = Simulation("MAPLEAF/Examples/Simulations/benchmarkSim.mapleaf", silent=True)
        sim.createRocket()

    def time_initializeVector(self):
        v = Vector(0, 1, 2)

    def time_initializeQuaternion(self):
        q = Quaternion(0, 1, 2, 3)

class TimeVector:
    def setup(self):
        self.v1 = Vector(0, 1, 2)
        self.v2 = Vector(3, 4, 5)

    def time_add(self):
        v3 = self.v1 + self.v2
    
    def time_dotProduct(self):
        dp = self.v1*self.v2
    
    def time_crossProduct(self):
        cp = self.v1.crossProduct(self.v2)

class TimeQuaternion:
    def setup(self):
        self.q1 = Quaternion(1, 2, 3, 4)
        self.q2 = Quaternion(2, 3, 4, 5)
        self.v1 = Vector(1, 2, 3)

    def time_multiplyQuaternions(self):
        q3 = self.q1 * self.q2

    def time_rotateVector(self):
        v2 = self.q1.rotate(self.v1)

class TimeStep:
    def setup(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/benchmarkSim.mapleaf")
        sim = Simulation(simDefinition=simDef, silent=True)
        self.rocket = sim.createRocket()

    def time_timeStep(self):
        self.rocket.timeStep(0.001)

class TimeStepWithLogging:
    def setup(self):
        simDef = SimDefinition("MAPLEAF/Examples/Simulations/benchmarkSim.mapleaf")
        simDef.setValue("SimControl.loggingLevel", "2")
        sim = Simulation(simDefinition=simDef, silent=True)
        self.rocket = sim.createRocket()

    def time_timeStep(self):
        self.rocket.timeStep(0.001)
