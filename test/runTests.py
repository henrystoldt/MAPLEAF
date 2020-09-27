import argparse
import os
import sys
import unittest


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

def build_Parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="""
    Convenience script for running subsets of the unttest test suite.
    """)
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
    unittestResults = None

    if len(args.Including) == 1:
        # Check for keywords
        if args.Including[0].lower() == "all":
            args.Including[0] = "5"
        
        elif args.Including[0].lower() == "regression":
            args.Including[0] = "0"
            runRegressionTests = True

        # Check for running tests by level preset
        try:
            level = int(args.Including[0])
            if level >= 5:
                runRegressionTests = True
            unittestResults = runTests_byLevelPresets(level)
        except ValueError:
            pass

    if unittestResults == None and args.Including != [ "0" ]:
        if len(args.excluding) > 0:
            unittestResults = runTests(exclude=args.excluding)
        elif len(args.Including) > 0:
            unittestResults = runTests(include=args.Including)
        else:
            unittestResults = runTests()

    ### Run regression tests ###
    if runRegressionTests:
        sys.stdout = sys.__stdout__
        from MAPLEAF.SimulationRunners.Batch import main as batchMain
        print("\n----------------------------------------------------------------------")
        print("RUNNING REGRESSION TESTS\n")

        batchMain(["./MAPLEAF/Examples/Simulations/regressionTests.mapleaf"])

        if unittestResults != None:
            ### Re-output Unit Testing Results ###
            print("\n----------------------------------------------------------------------")
            print("UNIT TEST Results")
            passed = unittestResults.testsRun - len(unittestResults.failures) - len(unittestResults.errors)
            print("Passed: {}/{}".format(passed, unittestResults.testsRun))

            if len(unittestResults.skipped) > 0:
                print("Skipped: {}".format(len(unittestResults.skipped)))
            
            if len(unittestResults.expectedFailures) > 0:
                print("Expected Failures: {}".format(len(unittestResults.expectedFailures)))
        
            if len(unittestResults.unexpectedSuccesses) > 0:
                print("Unexpected Successes: {}".format(len(unittestResults.unexpectedSuccesses)))

            print("")

            if passed == unittestResults.testsRun:
                print("OK")
            else:
                print("FAIL")

if __name__ == "__main__":
    main()
