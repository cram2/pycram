from . import world_conf as conf

HOST: str = "tcp://127.0.0.1"
SERVER_HOST: str = HOST
SERVER_PORT: str = "7000"

simulation_time_step: float = 1e-2
simulation_frequency: int = int(1 / simulation_time_step)

use_bullet_mode: bool = True
"""
If True, the simulation will always be in paused state unless the simulate() function is called, this behaves 
similar to bullet_world which uses the bullet physics engine.
"""

use_controller: bool = False
"""
Only used when use_bullet_mode is False. This turns on the controller for the robot joints.
"""

job_handling: conf.JobHandling = conf.JobHandling(let_pycram_move_attached_objects=False,
                                                  let_pycram_handle_spawning=False)

error_tolerance: conf.ErrorTolerance = conf.ErrorTolerance(acceptable_position_error=2e-2,
                                                           acceptable_prismatic_joint_position_error=2e-2)
