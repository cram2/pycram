import pycram.process_modules
# from .specialized_designators import *

from .datastructures.world import World
import signal
__version__ = "0.0.2"


def signal_handler(sig, frame):
    logging.info('You pressed Ctrl+C!')
    World.current_world.exit()
    print("Exiting...")
    exit(0)

signal.signal(signal.SIGINT, signal_handler)



