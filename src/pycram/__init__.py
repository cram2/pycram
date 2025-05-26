
from .datastructures.world import World
import signal
__version__ = "1.0.1"


def signal_handler(sig, frame):
    if World.current_world:
        World.current_world.exit()
    print("Exiting...")
    exit(0)

signal.signal(signal.SIGINT, signal_handler)

