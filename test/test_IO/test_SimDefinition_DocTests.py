''' Allows unittest to discover the doc tests in MAPLEAF/IO/simDefinition.py '''

from MAPLEAF.IO import simDefinition # Importing the module (lower case simDefinition), not the class (SimDefinition)!


def load_tests(loader, tests, ignore):
    import doctest
    tests.addTests(doctest.DocTestSuite(simDefinition))
    return tests
