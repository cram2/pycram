from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from itertools import product
from typing import Tuple

import numpy as np
from semantic_world.robots import Manipulator, AbstractRobot
from semantic_world.world_entity import View, Body
from typing_extensions import Optional, Union, List
from scipy.spatial.transform import Rotation as R

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

    @staticmethod
    def calculate_grasp_descriptions(robot: AbstractRobot, pose: PoseStamped, grasp_alignment: Optional[PreferredGraspAlignment] = None) -> \
            List[GraspDescription]:
        """
        This method determines the possible grasp configurations (approach axis and vertical alignment) of the body,
        taking into account the bodies orientation, position, and whether the gripper should be rotated by 90°.

        :param robot: The robot for which the grasp configurations are being calculated.
        :param grasp_alignment: An optional PreferredGraspAlignment object that specifies preferred grasp axis,
        :param pose: The pose of the object to be grasped.

        :return: A sorted list of GraspDescription instances representing all grasp permutations.
        """
        objectTmap = pose

        robot_pose = PoseStamped.from_spatial_type(robot.root.global_pose)

        if grasp_alignment:
            side_axis = grasp_alignment.preferred_axis
            vertical = grasp_alignment.with_vertical_alignment
            rotated_gripper = grasp_alignment.with_rotated_gripper
        else:
            side_axis, vertical, rotated_gripper = AxisIdentifier.Undefined, False, False

        object_to_robot_vector_world = objectTmap.position.vector_to_position(robot_pose.position)
        orientation = objectTmap.orientation.to_list()

        mapRobject = R.from_quat(orientation).as_matrix()
        objectRmap = mapRobject.T

        object_to_robot_vector_local = objectRmap.dot(object_to_robot_vector_world.to_numpy())
        vector_x, vector_y, vector_z = object_to_robot_vector_local

        vector_side = Vector3(vector_x, vector_y, np.nan)
        side_faces = GraspDescription.calculate_closest_faces(vector_side, side_axis)

        vector_vertical = Vector3(np.nan, np.nan, vector_z)
        if vertical:
            vertical_faces = GraspDescription.calculate_closest_faces(vector_vertical)
        else:
            vertical_faces = [VerticalAlignment.NoAlignment]

        grasp_configs = [
            GraspDescription(approach_direction=side, vertical_alignment=top_face, rotate_gripper=rotated_gripper)
            for top_face in vertical_faces
            for side in side_faces
        ]

        return grasp_configs

    @staticmethod
    def calculate_closest_faces(pose_to_robot_vector: Vector3,
                                specified_grasp_axis: AxisIdentifier = AxisIdentifier.Undefined) \
            -> Union[Tuple[ApproachDirection, ApproachDirection], Tuple[VerticalAlignment, VerticalAlignment]]:
        """
        Determines the faces of the object based on the input vector.

        If `specified_grasp_axis` is None, it calculates the primary and secondary faces based on the vector's magnitude
        determining which sides of the object are most aligned with the robot. This will either be the x, y plane for side faces
        or the z axis for top/bottom faces.
        If `specified_grasp_axis` is provided, it only considers the specified axis and calculates the faces aligned
        with that axis.

        :param pose_to_robot_vector: A 3D vector representing one of the robot's axes in the pose's frame, with
                              irrelevant components set to np.nan.
        :param specified_grasp_axis: Specifies a specific axis (e.g., X, Y, Z) to focus on.

        :return: A tuple of two Grasp enums representing the primary and secondary faces.
        """
        all_axes = [AxisIdentifier.X, AxisIdentifier.Y, AxisIdentifier.Z]

        if not specified_grasp_axis == AxisIdentifier.Undefined:
            valid_axes = [specified_grasp_axis]
        else:
            valid_axes = [axis for axis in all_axes if
                          not np.isnan(pose_to_robot_vector.to_list()[axis.value.index(1)])]

        object_to_robot_vector = np.array(pose_to_robot_vector.to_list()) + 1e-9
        sorted_axes = sorted(valid_axes, key=lambda axis: abs(object_to_robot_vector[axis.value.index(1)]),
                             reverse=True)

        primary_axis: AxisIdentifier = sorted_axes[0]
        primary_sign = int(np.sign(object_to_robot_vector[primary_axis.value.index(1)]))

        primary_axis_class = VerticalAlignment if primary_axis == AxisIdentifier.Z else ApproachDirection
        primary_face = primary_axis_class.from_axis_direction(primary_axis, primary_sign)

        if len(sorted_axes) > 1:
            secondary_axis: AxisIdentifier = sorted_axes[1]
            secondary_sign = int(np.sign(object_to_robot_vector[secondary_axis.value.index(1)]))
        else:
            secondary_axis: AxisIdentifier = primary_axis
            secondary_sign = -primary_sign

        secondary_axis_class = VerticalAlignment if secondary_axis == AxisIdentifier.Z else ApproachDirection
        secondary_face = secondary_axis_class.from_axis_direction(secondary_axis, secondary_sign)

        return primary_face, secondary_face

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
