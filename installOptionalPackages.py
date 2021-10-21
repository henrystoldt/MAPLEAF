import subprocess
import sys


def tryInstallingPackage(packageName: str, errorMessage: str):
    ''' Function used to handle 'nice-to-have', but not required packages '''
    try:
        print("\nAttempting to install optional package: {}\n".format(packageName))
        subprocess.check_call([sys.executable, "-m", "pip", "install", packageName])
        print("\nInstalled {}\n".format(packageName))
        return True
    
    except subprocess.CalledProcessError:
        # Output error, but continue installation if ray install fails
            # Error message won't be visible unless running `python setup.py develop`
        print("\nWARNING: Unable to install {}. {}\n".format(packageName, errorMessage))

        return False

# ray and mayavi are optional dependencies that often cause issues, especially on Windows
tryInstallingPackage("ray", "MAPLEAF will only run single-threaded.")

# Three-step dance to try to install Mayavi
if sys.version_info[1] > 8:
    print("Detected python v{}.{}".format(sys.version_info[0], sys.version_info[1]))
    print("Mayavi won't work with Python version 3.9.X or above as of fall 2021. Try Python 3.8 for 3D renders. Otherwise, MAPLEAF will fall back to 2D Matplotlib outputs.\n")
else:
    successPyQt = tryInstallingPackage("PyQT5", "MAPLEAF will not produce 3D renders of the earth. Will fall back to Matplotlib instead.")
    
    if successPyQt:
        successVTK901 = tryInstallingPackage("vtk==9.0.1", "MAPLEAF will not produce 3D renders of the earth. Will fall back to Matplotlib instead.")
        
        if successVTK901:
            tryInstallingPackage("mayavi", "MAPLEAF will not produce 3D renders of the earth. Will fall back to Matplotlib instead.")
