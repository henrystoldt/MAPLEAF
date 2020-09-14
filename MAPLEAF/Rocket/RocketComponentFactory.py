'''
In charge of initializing rocket components. Add new components to `stringNameToClassMap` to make them instantiate themselves when included in a Rocket
'''

from MAPLEAF.IO import SubDictReader
from MAPLEAF.Rocket import (
    AeroDamping, AeroForce, BoatTail, BodyTube, FinSet, FixedForce, FixedMass,
    FractionalJetDamping, NoseCone, RecoverySystem, TabulatedAeroForce,
    TabulatedInertia, TabulatedMotor, Transition)

__all__ = [ "stringNameToClassMap", "rocketComponentFactory" ]

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
    
    # Initialize it
    return referencedClass(componentDictReader, rocket, stage)
