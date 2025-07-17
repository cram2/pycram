from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Optional, Union, List

from .enums import Grasp, AxisIdentifier, ApproachDirection, VerticalAlignment
from ..has_parameters import HasParameters, has_parameters


@has_parameters
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
