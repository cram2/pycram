from . import process_modules
from . import robot_descriptions
# from .specialized_designators import *

from .datastructures.world import World
import signal
__version__ = "1.0.0"


def signal_handler(sig, frame):
    if World.current_world:
        World.current_world.exit()
    print("Exiting...")
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

