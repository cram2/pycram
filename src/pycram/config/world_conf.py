import math
import os
from dataclasses import dataclass

from typing_extensions import Tuple

resources_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'resources')
"""
Global reference for the resources path, this is used to search for the description files of the robot and
 the objects.
"""

cache_dir: str = os.path.join(resources_path, 'cached')
"""
Global reference for the cache directory, this is used to cache the description files of the robot and the objects.
"""

prospection_world_prefix: str = "prospection_"
"""
The prefix for the prospection world name.
"""

update_poses_from_sim_on_get: bool = True
"""
Whether to update the poses from the simulator when getting the object poses.
"""

DEBUG: bool = False
"""
Whether to use in debug mode. (This is used to print debug messages, plot images, etc.)
"""


@dataclass
class JobHandling:
    let_pycram_move_attached_objects: bool = True
    let_pycram_handle_spawning: bool = True
    let_pycram_handle_world_sync: bool = True
    """
    Whether to let PyCRAM handle the movement of attached objects, the spawning of objects,
     and the world synchronization.
    """

    def as_dict(self):
        return self.__dict__


@dataclass
class ErrorTolerance:
    acceptable_position_error: float = 1e-2
    acceptable_orientation_error: float = 10 * math.pi / 180
    acceptable_prismatic_joint_position_error: float = 1e-2
    acceptable_revolute_joint_position_error: float = 5 * math.pi / 180
    """
    The acceptable error for the position and orientation of an object/link, and the joint positions.
    """

    use_percentage_of_goal: bool = False
    acceptable_percentage_of_goal: float = 0.5
    """
    Whether to use a percentage of the goal as the acceptable error.
    """

    raise_goal_validator_error: bool = False
    """
    Whether to raise an error if the goals are not achieved.
    """

    @property
    def acceptable_pose_error(self) -> Tuple[float, float]:
        return self.acceptable_position_error, self.acceptable_orientation_error

    def as_dict(self):
        return self.__dict__
