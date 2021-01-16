import argparse
import os
import sys
import unittest

from MAPLEAF.SimulationRunners.Batch import main as runBatchSim

#### Running Tests ####
def _setupPath():
    # Make sure we're running from the main MAPLEAF directory
    cwd = os.getcwd()
    if os.path.basename(cwd) == "test":
        os.chdir("..")

    # Make sure main directory is in path
    cwd = os.getcwd()
    if cwd in sys.path:
        sys.path.remove(cwd)
    sys.path.insert(0, cwd)

def _stringMatch(string, exclude=None, include=None) -> bool:
    if (exclude == None and include == None) or (exclude != None and include != None):
        ValueError("Exactly one of include: {} and exclude: {} must not be None".format(include, exclude))

    if "test_" not in string:
        return False

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

def runUnitTests(exclude=None, include=None):
    _setupPath()

    #### Find tests ####
    suite = unittest.TestSuite()

    if exclude == None and include == None:
        # Run all tests
        suite.addTests(unittest.defaultTestLoader.discover(start_dir="test"))
    
    else:
        testDirContents = [ ("test/" + x) for x in os.listdir("test") ]
        for item in testDirContents:
            # Obtain python name from file name: ex: "text/test_Interpolation.py" -> "test_Interpolation"
            pythonName = item.replace("test/", "")
            pythonName = pythonName.replace(".py", "")

            if _stringMatch(item, exclude, include):
                # If the item name matches current include/exclude rules, include it
                
                if os.path.isdir(item):
                    # Include from folder/package
                    suite.addTests(unittest.defaultTestLoader.discover(pythonName))
                
                elif os.path.isfile(item):
                    # Include from file/module
                    suite.addTests(unittest.defaultTestLoader.loadTestsFromName(pythonName))

    #### Run Tests ####
    runner = unittest.TextTestRunner(verbosity=3) # verbosity=3 same as running unittest -v
    return runner.run(suite)
    
def _runUnitTests_byLevelPresets(level, excluding=None):
    '''
        Level 1 Runs just the quickest tests (Excludes tests in test_Rocket and test_SimulationRunners)
        Level 2 Excludes tests in test_SimulationRunners
        Level 3 Runs all unittest test cases
    '''
    if level == 0:
        return None
    elif level == 1:
        return runUnitTests(exclude=["SimulationRunners", "Rocket", "IO"])
    elif level == 2:
        return runUnitTests(exclude=["SimulationRunners", "Rocket"])
    elif level == 3:
        return runUnitTests(exclude=["SimulationRunners"])
    else:
        return runUnitTests()

def _runRegressionTests():
    print("\n----------------------------------------------------------------------")
    print("RUNNING REGRESSION TESTS\n")

    return runBatchSim(["./MAPLEAF/Examples/BatchSims/regressionTests.mapleaf"])


#### Command line interface ####
def _reOutputUnitTestResults(result):
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

def _build_Parser() -> argparse.ArgumentParser:
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
    """.format(startGray="\033[90m", endColor="\033[0m")) # https://en.wikipedia.org/wiki/ANSI_escape_code#Escape_sequences

    mutexGroup = parser.add_mutually_exclusive_group()
    mutexGroup.add_argument(
        "--excluding",
        nargs='*',
        default=[],
        help="Exclude tests in packages with these parameter(s) in their names"
    )
    mutexGroup.add_argument(
        "Including",
        nargs='*',
        default=[],
        help="Only run tests from packages with these parameter(s) in their names"
    )
    
    return parser

def main(argv=None) -> int:
    parser = _build_Parser()
    args = parser.parse_args(argv)
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
            unittestResult = _runUnitTests_byLevelPresets(level)
        except ValueError:
            pass

    if unittestResult == None and args.Including != [ "0" ]:
        if len(args.excluding) > 0:
            unittestResult = runUnitTests(exclude=args.excluding)
        elif len(args.Including) > 0:
            unittestResult = runUnitTests(include=args.Including)
        else:
            unittestResult = runUnitTests()

    returnCode = 0
    if unittestResult != None:
        if len(unittestResult.failures) + len(unittestResult.errors) > 0:
            returnCode = 1

    if runRegressionTests:
        sys.stdout = sys.__stdout__ # Remove any leftover loggers
        regressionReturnCode = _runRegressionTests()

        if unittestResult != None:
            _reOutputUnitTestResults(unittestResult)            
        
        return max(returnCode, regressionReturnCode)

if __name__ == "__main__":
    main()
