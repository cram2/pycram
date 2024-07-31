import os

import numpy as np
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

acceptable_position_error: float = 5e-3
acceptable_orientation_error: float = 10 * np.pi / 180
acceptable_pose_error: Tuple[float, float] = (acceptable_position_error, acceptable_orientation_error)
use_percentage_of_goal: bool = True
acceptable_percentage_of_goal: float = 0.5
"""
The acceptable error for the position and orientation of an object/link.
"""

raise_goal_validator_error: bool = False
"""
Whether to raise an error if the goals are not achieved.
"""