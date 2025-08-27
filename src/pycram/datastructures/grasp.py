from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from itertools import product

import numpy as np
from semantic_world.robots import Manipulator
from semantic_world.world_entity import View, Body
from typing_extensions import Optional, Union, List

from .dataclasses import Rotations
from .enums import Grasp, AxisIdentifier, ApproachDirection, VerticalAlignment
from .pose import PoseStamped, Pose, Vector3, Quaternion
from ..has_parameters import HasParameters, has_parameters
from ..tf_transformations import quaternion_multiply
from ..utils import translate_pose_along_local_axis


# @has_parameters
@dataclass
class GraspDescription(HasParameters):
    """
    Represents a grasp description with a side grasp, top face, and orientation alignment.
    """

    approach_direction: ApproachDirection
    """
    The primary approach direction. 
    """

    vertical_alignment: VerticalAlignment = VerticalAlignment.NoAlignment
    """
    The vertical alignment when grasping the pose
    """

    rotate_gripper: bool = False
    """
    Indicates if the gripper should be rotated by 90°. Must be a boolean.
    """

    def __hash__(self):
        return hash((self.approach_direction, self.vertical_alignment, self.rotate_gripper))

    def as_list(self) -> List[Union[Grasp, Optional[Grasp], bool]]:
        """
        :return: A list representation of the grasp description.
        """
        return [self.approach_direction, self.vertical_alignment, self.rotate_gripper]


    def get_grasp_pose(self, end_effector: Manipulator, body: Body, translate_rim_offset: bool = False) -> PoseStamped:
        """
        Translates the grasp pose of the object using the desired grasp description and object knowledge.
        Leaves the orientation untouched.
        Returns the translated grasp pose.

        :param end_effector: The end effector that will be used to grasp the object.
        :param body: The body of the object to be grasped.
        :param translate_rim_offset: If True, the grasp pose will be translated along the rim offset.

        :return: The grasp pose of the object.
        """
        grasp_pose = PoseStamped().from_matrix(body.global_pose, body._world.root)

        approach_direction = self.approach_direction
        rim_direction_index = approach_direction.value[0].value.index(1)

        # TODO the 0 index of the bounding_boxes is temporarily and needs to be better handled
        rim_offset = body.bounding_box_collection.bounding_boxes[0].dimensions[rim_direction_index] / 2

        grasp_pose.rotate_by_quaternion(self.calculate_grasp_orientation(end_effector.front_facing_orientation.to_np()))
        if translate_rim_offset:
            grasp_pose = translate_pose_along_local_axis(grasp_pose, self.approach_direction.axis.value, -rim_offset)

        return grasp_pose

    def calculate_grasp_orientation(self, front_orientation: np.ndarray) -> List[float]:
        """
        Calculates the grasp orientation based on the approach axis and the grasp description.

        :param front_orientation: The front-facing orientation of the end effector as a numpy array.

        :return: The calculated orientation as a quaternion.
        """
        rotation = Rotations.SIDE_ROTATIONS[self.approach_direction]
        rotation = quaternion_multiply(rotation, Rotations.VERTICAL_ROTATIONS[self.vertical_alignment])
        rotation = quaternion_multiply(rotation, Rotations.HORIZONTAL_ROTATIONS[self.rotate_gripper])

        orientation = quaternion_multiply(rotation, front_orientation)

        norm = math.sqrt(sum(comp ** 2 for comp in orientation))
        orientation = [comp / norm for comp in orientation]

        return orientation

@dataclass
class PreferredGraspAlignment:
    """
    Description of the preferred grasp alignment for an object.
    """
    preferred_axis: Optional[AxisIdentifier]
    """
    The preferred axis, X, Y, or Z, for grasping the object, or None if not specified.
    """

    with_vertical_alignment: bool
    """
    Indicates if the object should be grasped with a vertical alignment.
    """

    with_rotated_gripper: bool
    """
    Indicates if the gripper should be rotated by 90° around X.
    """
