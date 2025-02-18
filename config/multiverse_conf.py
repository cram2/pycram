import datetime

from typing_extensions import Type

from .world_conf import WorldConfig
from pycram.description import ObjectDescription
from pycram.helper import find_multiverse_resources_path
from pycram.object_descriptors.mjcf import ObjectDescription as MJCF


class MultiverseConfig(WorldConfig):
    # Multiverse Configuration
    resources_path = find_multiverse_resources_path()
    """
    The path to the Multiverse resources directory.
    """

    # Multiverse Socket Configuration
    HOST: str = "tcp://127.0.0.1"
    SERVER_HOST: str = HOST
    SERVER_PORT: str = 7000
    BASE_CLIENT_PORT: int = 9000

    # Multiverse Client Configuration
    READER_MAX_WAIT_TIME_FOR_DATA: datetime.timedelta = datetime.timedelta(milliseconds=1000)
    """
    The maximum wait time for the data in seconds.
    """

    # Multiverse Simulation Configuration
    simulation_time_step: datetime.timedelta = datetime.timedelta(milliseconds=10)
    simulation_frequency: int = int(1 / simulation_time_step.total_seconds())
    """
    The time step of the simulation in seconds and the frequency of the simulation in Hz.
    """

    simulation_wait_time_factor: float = 1.0
    """
    The factor to multiply the simulation wait time with, this is used to adjust the simulation wait time to account for
    the time taken by the simulation to process the request, this depends on the computational power of the machine
    running the simulation.
    """

    use_static_mode: bool = True
    """
    If True, the simulation will always be in paused state unless the simulate() function is called, this behaves 
    similar to bullet_world which uses the bullet physics engine.
    """

    use_controller: bool = True
    """
    Only used when use_static_mode is False. This turns on the controller for the robot joints.
    """

    default_description_type: Type[ObjectDescription] = MJCF
    """
    The default description type for the objects.
    """

    use_physics_simulator_state: bool = False
    """
    Whether to use the physics simulator state when restoring or saving the world state.
    """

    validate_goals = True

    clear_cache_at_start = True

    let_pycram_move_attached_objects = False
    let_pycram_handle_spawning = False

    position_tolerance = 2e-2
    prismatic_joint_position_tolerance = 2e-2

    use_giskard_monitor = False
    allow_gripper_collision = True

    use_multiverse_process_modules = True

    depth_images_are_in_meter = True

    max_batch_size_for_rays = None
