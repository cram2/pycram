from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Optional, Union, List

from .enums import Grasp, AxisIdentifier


@dataclass
class GraspDescription:
    """
    Represents a grasp description with a side grasp, top face, and orientation alignment.
    """

    approach_direction: Grasp
    """
    The primary approach direction. Must be one of {Grasp.FRONT, Grasp.BACK, Grasp.LEFT, Grasp.RIGHT}.
    """

    vertical_alignment: Optional[Grasp] = None
    """
    The vertical alignment when grasping the pose, or None if not applicable. Must be one of {Grasp.TOP, Grasp.BOTTOM, None}.
    """

    rotate_gripper: bool = False
    """
    Indicates if the gripper should be rotated by 90°. Must be a boolean.
    """

    def __post_init__(self):
        allowed_approach_direction = {Grasp.FRONT, Grasp.BACK, Grasp.LEFT, Grasp.RIGHT}
        if self.approach_direction not in allowed_approach_direction:
            raise ValueError(
                f"Invalid value for side_face: {self.approach_direction}. Allowed values are {allowed_approach_direction}")
        allowed_vertical_alignment = {Grasp.TOP, Grasp.BOTTOM, None}
        if self.vertical_alignment not in allowed_vertical_alignment:
            raise ValueError(
                f"Invalid value for top_face: {self.vertical_alignment}. Allowed values are {allowed_vertical_alignment}")

    def __hash__(self):
        return hash((self.approach_direction, self.vertical_alignment, self.rotate_gripper))

    def as_list(self) -> List[Union[Grasp, Optional[Grasp], bool]]:
        """
        :return: A list representation of the grasp description.
        """
        return [self.approach_direction, self.vertical_alignment, self.rotate_gripper]


@dataclass(frozen=True)
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
