import sys
import os

# Add the root directory of the project to sys.path
package_path = os.path.join(os.path.dirname(__file__), 'my_filter_package')
if package_path not in sys.path:
    sys.path.append(package_path)

# Now you can import filter from my_filter_package
from filter import ParticleFilter

import UDPPeer


myFilter = ParticleFilter(.....)
myPeer = UDPPeer(....)