# Created by: Henry Stoldt
# April 2020

import builtins
import sys
from contextlib import contextmanager

from MAPLEAF.Motion import Vector

#### Functions to assess near-equality for custom types ####

def assertQuaternionsAlmostEqual(TestCaseObject, Q1, Q2, n=7):
    norm1 = Q1.norm()
    norm2 = Q2.norm()
    TestCaseObject.assertAlmostEqual(norm1, norm2, n)
    rotAngle1 = Q1.rotationAngle()
    rotAngle2 = Q2.rotationAngle()
    TestCaseObject.assertAlmostEqual(rotAngle1, rotAngle2, n)
    rotAxis1 = Q1.rotationAxis()
    rotAxis2 = Q2.rotationAxis()
    assertVectorsAlmostEqual(TestCaseObject, rotAxis1, rotAxis2, n)

def assertVectorsAlmostEqual(TestCaseObject, Vector1, Vector2, n=7):
    TestCaseObject.assertAlmostEqual(Vector1.X, Vector2.X, n)
    TestCaseObject.assertAlmostEqual(Vector1.Y, Vector2.Y, n)
    TestCaseObject.assertAlmostEqual(Vector1.Z, Vector2.Z, n)

def assertIterablesAlmostEqual(TestCaseObject, x1, x2, n=7):
    # Could use this function to replace Vector, Quat, and AngVel almost equal functions
    if len(x1) != len(x2):
        raise ValueError("Iterables x1 (length: {}) and x2 (length: {}) must have the same length!".format(len(x1), len(x2)))
    for i in range(len(x1)):
        TestCaseObject.assertAlmostEqual(x1[i], x2[i], n)

def assertAngVelAlmostEqual(TestCaseObject, av1, av2, n=7):
    for i in range(3):
        TestCaseObject.assertAlmostEqual(av1[i], av2[i], n)

def assertRigidBodyStatesalmostEqual(TestCaseObject, state1, state2):
    assertVectorsAlmostEqual(TestCaseObject, state1.position, state2.position)
    assertVectorsAlmostEqual(TestCaseObject, state1.velocity, state2.velocity)
    assertQuaternionsAlmostEqual(TestCaseObject, state1.orientation, state2.orientation)
    assertAngVelAlmostEqual(TestCaseObject, state1.angularVelocity, state2.angularVelocity)

def assertInertiasAlmostEqual(TestCaseObject, i1, i2, n=7):
    assertVectorsAlmostEqual(TestCaseObject, i1.MOI, i2.MOI, n=n)
    assertVectorsAlmostEqual(TestCaseObject, i1.MOICentroidLocation, i2.MOICentroidLocation, n=n)
    assertVectorsAlmostEqual(TestCaseObject, i1.CG, i2.CG, n=n)
    TestCaseObject.assertAlmostEqual(i1.mass, i2.mass, n)

def assertForceMomentSystemsAlmostEqual(TestCaseObject, fms1, fms2, n=7):
    # Convert both systems to be defined about the origin
    fms1 = fms1.getAt(Vector(0,0,0))
    fms2 = fms2.getAt(Vector(0,0,0))
    # Then check that forces and moments are almost equal
    assertVectorsAlmostEqual(TestCaseObject, fms1.force, fms2.force, n)
    assertVectorsAlmostEqual(TestCaseObject, fms1.moment, fms2.moment, n)

#### Functions to manipulate sim definitions pre-simulation ####
def setUpSimDefForMinimalRunCheck(simDef):
    ''' Pass in a SimDefinition / simDefinition - it will be modified in place '''

    # Set the simulation to only take a single time step
    simDef.setValue("SimControl.EndCondition", "Time")
    simDef.setValue("SimControl.EndConditionValue", "0.01")
    simDef.setValue("SimControl.timeStep", "0.01")

    # Using Euler time stepping
    simDef.setValue("SimControl.timeDiscretization", "Euler")

    # And producing no plots/logs
    simDef.setValue("SimControl.plot", "None")
    simDef.setValue("SimControl.loggingLevel", "0")

def setUpSimDefForMinimalRunCheck_MonteCarlo(mCsimDef):
    ''' Pass in a SimDefinition / simDefinition - it will be modified in place '''
    setUpSimDefForMinimalRunCheck(mCsimDef)

    mCsimDef.setValue("MonteCarlo.output", "None")
    mCsimDef.setValue("MonteCarlo.numberRuns", "2")

#### Context Managers for testing functions that interact with the command line ####

@contextmanager
def mockInput(mock):
    ''' 
        Mocks the input() function
        
        Arguments:
            mock: (string or list of strings). 
                If list of strings, each element of the list is yielded one after another when input is called. 
                Otherwise the same string is returned each time input() is called.

        Returns: None

        Based on answer by ArtOfWarfare: https://stackoverflow.com/questions/21046717/python-mocking-raw-input-in-unittests
    '''
    # Store reference to normal input() function
    original_input = builtins.input

    if isinstance(mock, str):
        # Replace with a lambda function returning the string mock
        builtins.input = lambda _=None: mock
    else:
        # Assume mock is an iterable of strings
        # Replace input() with a function that returns the next item in mock
        stringGen = ( s for s in mock )
        
        def getNextMockInput(_=None):
            return next(stringGen)

        builtins.input = getNextMockInput

    yield # Context manager setup complete

    # On exit - restore normal function of input()
    builtins.input = original_input

@contextmanager
def captureOutput():
    '''
        Captures print statements

        Based on answer by Rob Kennedy: https://stackoverflow.com/questions/4219717/how-to-assert-output-with-nosetest-unittest-in-python
    '''
    from io import StringIO

    newOut, newErr = StringIO(), StringIO()
    oldOut, oldErr = sys.stdout, sys.stderr
    try:
        sys.stdout, sys.stderr = newOut, newErr
        yield sys.stdout, sys.stderr
    finally:
        sys.stdout, sys.stderr = oldOut, oldErr
