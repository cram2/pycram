import datetime
import os

from . import world_conf as world_conf
from ..worlds.multiverse_extras.helpers import find_multiverse_resources_path

# Multiverse Configuration
resources_path: str = find_multiverse_resources_path()
"""
The path to the Multiverse resources directory.
"""


# Multiverse Socket Configuration
HOST: str = "tcp://127.0.0.1"
SERVER_HOST: str = HOST
SERVER_PORT: str = "7000"
BASE_CLIENT_PORT: int = 9000

# Multiverse Client Configuration
READER_MAX_WAIT_TIME_FOR_DATA: datetime.timedelta = datetime.timedelta(milliseconds=1000)
"""
The maximum wait time for the data in seconds.
"""

# Multiverse Simulation Configuration
simulation_time_step: datetime.timedelta = datetime.timedelta(milliseconds=10)
simulation_frequency: int = int(1 / simulation_time_step.total_seconds())

simulation_wait_time_factor: float = 1.0
"""
The factor to multiply the simulation wait time with, this is used to adjust the simulation wait time to account for
the time taken by the simulation to process the request, this depends on the computational power of the machine
running the simulation.
"""

use_bullet_mode: bool = True
"""
If True, the simulation will always be in paused state unless the simulate() function is called, this behaves 
similar to bullet_world which uses the bullet physics engine.
"""

use_controller: bool = False
"""
Only used when use_bullet_mode is False. This turns on the controller for the robot joints.
"""

job_handling: world_conf.JobHandling = world_conf.JobHandling(let_pycram_move_attached_objects=False,
                                                              let_pycram_handle_spawning=False)

error_tolerance: world_conf.ErrorTolerance = world_conf.ErrorTolerance(acceptable_position_error=2e-2,
                                                                       acceptable_prismatic_joint_position_error=2e-2)
