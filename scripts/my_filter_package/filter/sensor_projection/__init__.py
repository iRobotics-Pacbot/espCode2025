import os
import sys
import ctypes

# Get the path to the current directory where the .so file is located
current_dir = os.path.dirname(__file__)

# Path to the shared object file
sensor_so_path = os.path.join(current_dir, 'sensor_projection.cpython-313-darwin.so')

# Check if the file exists
if not os.path.exists(sensor_so_path):
    raise FileNotFoundError(f"The shared object file {sensor_so_path} does not exist.")

# Use ctypes to load the .so file
sensor_projection = ctypes.CDLL(sensor_so_path)

# Now you should be able to use sensor_projection functions or attributes, if needed
