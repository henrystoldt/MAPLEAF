'''
In charge of initializing rocket components. Add new components to `stringNameToClassMap` to make them instantiate themselves when included in a Rocket
'''

from MAPLEAF.IO import SubDictReader
from MAPLEAF.Motion import Vector
from MAPLEAF.Rocket import (
    AeroDamping, AeroForce, BoatTail, BodyTube, FinSet, FixedForce, FixedMass,
    FractionalJetDamping, NoseCone, RecoverySystem, SampleStatefulComponent, TabulatedAeroForce,
    TabulatedInertia, TabulatedMotor, Transition)

__all__ = [ "stringNameToClassMap", "rocketComponentFactory", "initializeForceLogging" ]

stringNameToClassMap = {
    "AeroDamping":          AeroDamping,
    "AeroForce":            AeroForce,
    "BoatTail":             BoatTail,
    "Bodytube":             BodyTube,
    "FinSet":               FinSet,
    "Force":                FixedForce,
    "FractionalJetDamping": FractionalJetDamping,
    "Mass":                 FixedMass,
    "Motor":                TabulatedMotor,
    "Nosecone":             NoseCone,
    "RecoverySystem":       RecoverySystem,
    "StatefulSample":       SampleStatefulComponent,
    "TabulatedAeroForce":   TabulatedAeroForce,
    "TabulatedInertia":     TabulatedInertia,
    "Transition":           Transition,
}

def rocketComponentFactory(subDictPath, rocket, stage):
    """
        Initializes a rocket component based on the stringNameToClassMap
        Inputs:
            subDictPath:        (string) Path to subDict in simulation definition, like "Rocket.Stage1.Nosecone"
            rocket:             (Rocket) that the component is a part of
            stage:              (Stage) That the component is a part of
        Also uses the stringNameToClassMap dictionary
    """       
    # Create SubDictReader for the rocket component's dictionary
    componentDictReader = SubDictReader(subDictPath, rocket.simDefinition)

    # Figure out which class to initialize
    className = componentDictReader.getString("class")
    referencedClass = stringNameToClassMap[className]
    
    # Initialize the rocket component
    newComponent = referencedClass(componentDictReader, rocket, stage)

    # Initialize logging component forces (if desired)
    initializeForceLogging(newComponent, subDictPath, rocket)

    return newComponent

def initializeForceLogging(component, subDictPath, rocket):
    if rocket.derivativeEvaluationLog is not None:
        componentName = subDictPath[subDictPath.index(".")+1:]
        zeroVector = Vector(0,0,0)
        component.forcesLog = rocket.derivativeEvaluationLog.addColumn(componentName + "F(N)", zeroVector)
        component.momentsLog = rocket.derivativeEvaluationLog.addColumn(componentName + "M(Nm)", zeroVector)