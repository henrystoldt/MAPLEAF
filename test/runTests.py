import unittest
import os, sys

def runTests(level=3):
    '''
        Level 1 Runs just the quickest tests (Excludes tests in test_Rocket and test_SimulationRunners)
        Level 2 Excludes tests in test_SimulationRunners
        Level 3 Runs all unittest test cases
    '''
    suite = unittest.TestSuite()

    # Make sure we're running from the main MAPLEAF directory
    cwd = os.getcwd()
    if os.path.basename(cwd) == "test":
        os.chdir("..")

    cwd = os.getcwd()
    if cwd not in sys.path:
        sys.path.insert(0, cwd)

    #### Find tests ####
    if level < 3:
        items = [ "test/" + x for x in os.listdir("test") ]

        for item in items:
            moduleName = item.replace("test/", "test.")
            moduleName = moduleName.replace(".py", "")
            
            # Exclude functional, long-running tests from the SimulationRunners folder/package
            if level == 1 and os.path.isdir(item) and "SimulationRunners" not in item and "Rocket" not in item and "test_" in item:
                print(moduleName)
                suite.addTests(unittest.defaultTestLoader.discover(moduleName))

            if level == 2 and os.path.isdir(item) and "SimulationRunners" not in item and "test_" in item:
                print(moduleName)
                suite.addTests(unittest.defaultTestLoader.discover(moduleName))
            
            # Add test modules from the test/ 
            if os.path.isfile(item) and "test_" in item:
                print(moduleName)
                suite.addTests(unittest.defaultTestLoader.loadTestsFromName(moduleName))
    elif level == 3:
        suite.addTests(unittest.defaultTestLoader.discover(start_dir="."))

    #### Run Tests ####
    runner = unittest.TextTestRunner(verbosity=3)
    runner.run(suite)

if __name__ == "__main__":
    runTests()