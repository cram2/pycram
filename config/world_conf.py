import math
import os
from datetime import timedelta

import yaml
from typing_extensions import Tuple, Type, Dict, Optional
from ..description import ObjectDescription
from ..object_descriptors.urdf import ObjectDescription as URDF
from ..utils import classproperty


class WorldConfig:

    """
    A class to store the configuration of the world, this can be inherited to create a new configuration class for a
    specific world (e.g. multiverse has MultiverseConfig which inherits from this class).
    """
    resources_path = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', 'resources'))
    """
    Global reference for the resources path, this is used to search for the description files of the robot and
     the objects.
    """

    project_root: str = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
    """
    The root path of the project, used to find resources and other files.
    """

    cache_dir_name: str = 'cached'
    """
    The name of the cache directory.
    """

    cache_dir: str = os.path.join(resources_path, cache_dir_name)
    """
    Global reference for the cache directory, this is used to cache the description files of the robot and the objects.
    """

    clear_cache_at_start: bool = True
    """
    Whether to clear the cache directory at the start.
    """

    prospection_world_prefix: str = "prospection"
    """
    The prefix for the prospection world name.
    """

    simulation_frequency: int = 240
    """
    The simulation frequency (Hz), used for calculating the equivalent real time in the simulation.
    """

    update_poses_from_sim_on_get: bool = True
    """
    Whether to update the poses from the simulator when getting the object poses.
    """

    default_description_type: Type[ObjectDescription] = URDF
    """
    The default description type for the objects.
    """

    use_physics_simulator_state: bool = False
    """
    Whether to use the physics simulator state when restoring or saving the world state.
    Currently with PyBullet, this causes a bug where ray_test does not work correctly after restoring the state using the
    simulator, so it is recommended to set this to False in PyBullet.
    """

    let_pycram_move_attached_objects: bool = True
    let_pycram_handle_spawning: bool = True
    let_pycram_handle_world_sync: bool = True
    """
    Whether to let PyCRAM handle the movement of attached objects, the spawning of objects,
     and the world synchronization.
    """

    position_tolerance: float = 1e-3
    orientation_tolerance: float = 10 * math.pi / 180
    prismatic_joint_position_tolerance: float = 1e-2
    revolute_joint_position_tolerance: float = 2 * math.pi / 180
    """
    The acceptable error for the position and orientation of an object/link, and the joint positions.
    """

    use_percentage_of_goal: bool = False
    acceptable_percentage_of_goal: float = 0.5
    """
    Whether to use a percentage of the goal as the acceptable error.
    """

    validate_goals: bool = False
    """
    Whether to validate the goals when executing them.
    """

    raise_goal_validator_error: bool = False
    """
    Whether to raise an error if the goals are not achieved.
    """

    use_giskard_monitor: bool = True
    """
    Whether to use the Giskard goal monitor when executing the goals.
    """

    allow_gripper_collision: bool = True
    """
    Whether to allow the gripper to collide with the objects when planning for the goals.
    """

    depth_images_are_in_meter: bool = False
    """
    Whether the depth images produced by :meth:`datastructures.world.World.get_images_for_target` are in meters.
    """

    max_batch_size_for_rays: Optional[int] = 16380
    """
    The maximum batch size for the rays when using the ray test batch.
    """

    execution_delay: timedelta = timedelta(seconds=0)
    """
    The delay between the execution of actions/motions to imitate real world execution time.
    """

    @classproperty
    def default_camera_config(cls) -> Dict:
        """
        The default camera configuration for the world.

        :return: The configuration for the camera as pose, yaw, tilt
        """
        if os.path.exists(os.path.join(os.path.dirname(__file__), 'camera.yaml')):
            with open(os.path.join(os.path.dirname(__file__), 'camera.yaml'), 'r') as f:
                return yaml.load(f, Loader=yaml.SafeLoader)
        return {'target_position': [1, 2, 3], 'yaw': 50, 'pitch': -35, "dist": 1.5}

    @classmethod
    def get_pose_tolerance(cls) -> Tuple[float, float]:
        return cls.position_tolerance, cls.orientation_tolerance
