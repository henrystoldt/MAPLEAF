import argparse
import os
import sys
import unittest

from MAPLEAF.SimulationRunners.Batch import main as runBatchSim


def setupPath():
    # Make sure we're running from the main MAPLEAF directory
    cwd = os.getcwd()
    if os.path.basename(cwd) == "test":
        os.chdir("..")

    # Make sure main directory is in path
    cwd = os.getcwd()
    if cwd in sys.path:
        sys.path.remove(cwd)
    sys.path.insert(0, cwd)

def stringMatch(string, exclude=None, include=None) -> bool:
    if (exclude == None and include == None) or (exclude != None and include != None):
        ValueError("Exactly one of include: {} and exclude: {} must not be None".format(include, exclude))

    if exclude != None:
        # Check if name should be excluded
        for excludeStr in exclude:
            if excludeStr in string:
                return False
        
        return True

    if include != None:
        # Check if name should be included
        for includeStr in include:
            if includeStr in string:
                return True
    
        return False

def folderMatch(folder, exclude=None, include=None) -> bool:
    ''' Check whether itemName is a package that should be included in the test suite '''
    # Check if is a "test_" directory
    if not os.path.isdir(folder) or "test_" not in folder:
        return False
    
    return stringMatch(folder, exclude, include)

def fileMatch(file, exclude=None, include=None) -> bool:
    ''' Check whether itemName is a package that should be included in the test suite '''
    # Check if is a "test_" file
    if not os.path.isfile(file) or "test_" not in file:
        return False
    
    return stringMatch(file, exclude, include)

def runTests(exclude=None, include=None):
    setupPath()

    #### Find tests ####
    suite = unittest.TestSuite()

    if exclude == None and include == None:
        # Run all tests
        suite.addTests(unittest.defaultTestLoader.discover(start_dir="test"))
    
    else:
        testDirContents = [ ("test/" + x) for x in os.listdir("test") ]
        for item in testDirContents:
            moduleName = item.replace("test/", "")
            moduleName = moduleName.replace(".py", "")

            if folderMatch(item, exclude, include):
                suite.addTests(unittest.defaultTestLoader.discover(moduleName))
            
            elif fileMatch(item, exclude, include):
                suite.addTests(unittest.defaultTestLoader.loadTestsFromName(moduleName))

    #### Run Tests ####
    runner = unittest.TextTestRunner(verbosity=3)
    return runner.run(suite)
    
def runTests_byLevelPresets(level, excluding=None):
    '''
        Level 1 Runs just the quickest tests (Excludes tests in test_Rocket and test_SimulationRunners)
        Level 2 Excludes tests in test_SimulationRunners
        Level 3 Runs all unittest test cases
    '''
    if level == 0:
        return None
    elif level == 1:
        return runTests(exclude=["SimulationRunners", "Rocket", "IO"])
    elif level == 2:
        return runTests(exclude=["SimulationRunners", "Rocket"])
    elif level == 3:
        return runTests(exclude=["SimulationRunners"])
    else:
        return runTests()

def reOutputUnitTestResults(result):
    print("\n----------------------------------------------------------------------")
    print("UNIT TEST Results")
    passed = result.testsRun - len(result.failures) - len(result.errors)
    print("Passed: {}/{}".format(passed, result.testsRun))

    if len(result.skipped) > 0:
        print("Skipped: {}".format(len(result.skipped)))
    
    if len(result.expectedFailures) > 0:
        print("Expected Failures: {}".format(len(result.expectedFailures)))

    if len(result.unexpectedSuccesses) > 0:
        print("Unexpected Successes: {}".format(len(result.unexpectedSuccesses)))

    print("")

    if passed == result.testsRun:
        print("OK")
    else:
        print("FAIL")

def build_Parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description="""
    Convenience script for running subsets of the unittest test suite OR all tests including the regression test suite.  

    Examples:
        {startGray}# Key words:{endColor}
        python runTests.py all                      {startGray}# All unit and regression tests{endColor}
        python runTests.py regression               {startGray}# Only regression tests {endColor}

        {startGray}# Inclusive run:{endColor}
        python runTests.py Motion GNC IO            {startGray}# Run unit tests from test/test_Motion, test/test_GNC, and test/test_IO{endColor}
        
        {startGray}# Exclusive run:{endColor}
        python runTests.py --excl Motion GNC IO     {startGray}# Run all unit tests EXCEPT those from test/test_Motion, test/test_GNC, and test/test_IO{endColor}

        {startGray}# Run by level presets:{endColor}
        python runTests.py 1                        {startGray}# Exclude SimulationRunners, Rocket, and IO{endColor}
        python runTests.py 2                        {startGray}# Exclude SimulationRunners and Rocket{endColor}
        python runTests.py 3                        {startGray}# Exclude SimulationRunners{endColor}
        python runTests.py 4                        {startGray}# All unit tests{endColor}
        python runTests.py 5                        {startGray}# All unit and regression tests{endColor}
    """.format(startGray="\033[90m", endColor="\033[0m"))
    parser.add_argument(
        "--excluding",
        nargs='*',
        default=[],
        help="Exclude tests in packages with these parameter(s) in their names"
    )
    parser.add_argument(
        "Including",
        nargs='*',
        default=[],
        help="Only run tests from packages with these parameter(s) in their names"
    )
    
    return parser

def checkForMutuallyExclusiveArgs(args):
    if len(args.Including) > 0 and len(args.excluding) > 0:
        raise ValueError("Script can only be run in inclusive or exclusive mode")

def main(argv=None):
    parser = build_Parser()
    args = parser.parse_args(argv)
    checkForMutuallyExclusiveArgs(args)
    runRegressionTests = False

    ### Run unit tests ###
    unittestResult = None

    if len(args.Including) == 1:
        if args.Including[0].lower() == "all":
            args.Including = [ "4" ]
        
        elif args.Including[0].lower() == "regression":
            args.Including = [ "0" ] # Don't run any unit tests
            runRegressionTests = True # Run regression tests

        try:
            # Try to run tests by level preset
            level = int(args.Including[0])
            if level >= 4:
                runRegressionTests = True # For levels >= 5, run all unit tests and all regression tests
            unittestResult = runTests_byLevelPresets(level)
        except ValueError:
            pass

    if unittestResult == None and args.Including != [ "0" ]:
        if len(args.excluding) > 0:
            unittestResult = runTests(exclude=args.excluding)
        elif len(args.Including) > 0:
            unittestResult = runTests(include=args.Including)
        else:
            unittestResult = runTests()

    if runRegressionTests:
        sys.stdout = sys.__stdout__ # Remove any leftover loggers
        print("\n----------------------------------------------------------------------")
        print("RUNNING REGRESSION TESTS\n")

        runBatchSim(["./MAPLEAF/Examples/Simulations/regressionTests.mapleaf"])

        if unittestResult != None:
            reOutputUnitTestResults(unittestResult)            

if __name__ == "__main__":
    main()
