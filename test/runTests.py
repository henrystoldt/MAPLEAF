import unittest
import os, sys

def runTests(level=3):
    '''
        Level 1 Runs just the quickest tests (Excludes tests in test_Rocket and test_SimulationRunners)
        Level 2 Excludes tests in test_SimulationRunners
        Level 3 Runs all unittest test cases
    '''
    # Make sure we're running from the main MAPLEAF directory
    cwd = os.getcwd()
    if os.path.basename(cwd) == "test":
        os.chdir("..")

    # Make sure main directory is in path
    cwd = os.getcwd()
    if cwd in sys.path:
        sys.path.remove(cwd)
    sys.path.insert(0, cwd)

    #### Find tests ####
    suite = unittest.TestSuite()

    if level < 3:
        items = [ ("test/" + x) for x in os.listdir("test") ]

        for item in items:
            moduleName = item.replace("test/", "")
            moduleName = moduleName.replace(".py", "")
            
            # Exclude functional, long-running tests from the SimulationRunners and Rocket folders/packages
            if level == 1 and os.path.isdir(item) and "SimulationRunners" not in item and "Rocket" not in item and "test_" in item:
                print(moduleName)
                suite.addTests(unittest.defaultTestLoader.discover(moduleName))

            # Only exclude SimulationRunners
            elif level == 2 and os.path.isdir(item) and "SimulationRunners" not in item and "test_" in item:
                print(moduleName)
                suite.addTests(unittest.defaultTestLoader.discover(moduleName))
            
            # Always add test modules from test/ 
            elif os.path.isfile(item) and "test_" in item:
                print(moduleName)
                suite.addTests(unittest.defaultTestLoader.loadTestsFromName(moduleName))
    
    elif level == 3:
        # Run all tests
        suite.addTests(unittest.defaultTestLoader.discover(start_dir="test"))

    #### Run Tests ####
    runner = unittest.TextTestRunner(verbosity=3)
    result = runner.run(suite)

if __name__ == "__main__":
    runTests(1)