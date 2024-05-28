from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point
from typing_extensions import Union, List, Optional, TYPE_CHECKING

from ..datastructures.enums import JointType
from ..datastructures.pose import Transform, Pose

if TYPE_CHECKING:
    from ..description import Link


class AbstractConstraint:
    """
    Represents an abstract constraint concept, this could be used to create joints for example or any kind of constraint
    between two links in the world.
    """

    def __init__(self,
                 parent_link: Link,
                 child_link: Link,
                 _type: JointType,
                 parent_to_constraint: Transform,
                 child_to_constraint: Transform):
        self.parent_link: Link = parent_link
        self.child_link: Link = child_link
        self.type: JointType = _type
        self.parent_to_constraint = parent_to_constraint
        self.child_to_constraint = child_to_constraint
        self._parent_to_child = None

    @property
    def parent_to_child_transform(self) -> Union[Transform, None]:
        if self._parent_to_child is None:
            if self.parent_to_constraint is not None and self.child_to_constraint is not None:
                self._parent_to_child = self.parent_to_constraint * self.child_to_constraint.invert()
        return self._parent_to_child

    @parent_to_child_transform.setter
    def parent_to_child_transform(self, transform: Transform) -> None:
        self._parent_to_child = transform

    @property
    def parent_object_id(self) -> int:
        """
        Returns the id of the parent object of the constraint.

        :return: The id of the parent object of the constraint
        """
        return self.parent_link.object_id

    @property
    def child_object_id(self) -> int:
        """
        Returns the id of the child object of the constraint.

        :return: The id of the child object of the constraint
        """
        return self.child_link.object_id

    @property
    def parent_link_id(self) -> int:
        """
        Returns the id of the parent link of the constraint.

        :return: The id of the parent link of the constraint
        """
        return self.parent_link.id

    @property
    def child_link_id(self) -> int:
        """
        Returns the id of the child link of the constraint.

        :return: The id of the child link of the constraint
        """
        return self.child_link.id

    @property
    def position_wrt_parent_as_list(self) -> List[float]:
        """
        Returns the constraint frame pose with respect to the parent origin as a list.

        :return: The constraint frame pose with respect to the parent origin as a list
        """
        return self.pose_wrt_parent.position_as_list()

    @property
    def orientation_wrt_parent_as_list(self) -> List[float]:
        """
        Returns the constraint frame orientation with respect to the parent origin as a list.

        :return: The constraint frame orientation with respect to the parent origin as a list
        """
        return self.pose_wrt_parent.orientation_as_list()

    @property
    def pose_wrt_parent(self) -> Pose:
        """
        Returns the joint frame pose with respect to the parent origin.

        :return: The joint frame pose with respect to the parent origin
        """
        return self.parent_to_constraint.to_pose()

    @property
    def position_wrt_child_as_list(self) -> List[float]:
        """
        Returns the constraint frame pose with respect to the child origin as a list.

        :return: The constraint frame pose with respect to the child origin as a list
        """
        return self.pose_wrt_child.position_as_list()

    @property
    def orientation_wrt_child_as_list(self) -> List[float]:
        """
        Returns the constraint frame orientation with respect to the child origin as a list.

        :return: The constraint frame orientation with respect to the child origin as a list
        """
        return self.pose_wrt_child.orientation_as_list()

    @property
    def pose_wrt_child(self) -> Pose:
        """
        Returns the joint frame pose with respect to the child origin.

        :return: The joint frame pose with respect to the child origin
        """
        return self.child_to_constraint.to_pose()


class Constraint(AbstractConstraint):
    """
    Represents a constraint between two links in the World.
    """

    def __init__(self,
                 parent_link: Link,
                 child_link: Link,
                 _type: JointType,
                 axis_in_child_frame: Point,
                 constraint_to_parent: Transform,
                 child_to_constraint: Transform):
        parent_to_constraint = constraint_to_parent.invert()
        AbstractConstraint.__init__(self, parent_link, child_link, _type, parent_to_constraint, child_to_constraint)
        self.axis: Point = axis_in_child_frame

    @property
    def axis_as_list(self) -> List[float]:
        """
        Returns the axis of this constraint as a list.

        :return: The axis of this constraint as a list of xyz
        """
        return [self.axis.x, self.axis.y, self.axis.z]


class Attachment(AbstractConstraint):
    def __init__(self,
                 parent_link: Link,
                 child_link: Link,
                 bidirectional: Optional[bool] = False,
                 parent_to_child_transform: Optional[Transform] = None,
                 constraint_id: Optional[int] = None):
        """
        Creates an attachment between the parent object link and the child object link.
        This could be a bidirectional attachment, meaning that both objects will move when one moves.

        :param parent_link: The parent object link.
        :param child_link: The child object link.
        :param bidirectional: If true, both objects will move when one moves.
        :param parent_to_child_transform: The transform from the parent link to the child object link.
        :param constraint_id: The id of the constraint in the simulator.
        """
        super().__init__(parent_link, child_link, JointType.FIXED, parent_to_child_transform,
                         Transform(frame=child_link.tf_frame))
        self.id = constraint_id
        self.bidirectional: bool = bidirectional
        self._loose: bool = False

        if self.parent_to_child_transform is None:
            self.update_transform()

        if self.id is None:
            self.add_fixed_constraint()

    def update_transform_and_constraint(self) -> None:
        """
        Updates the transform and constraint of this attachment.
        """
        self.update_transform()
        self.update_constraint()

    def update_transform(self) -> None:
        """
        Updates the transform of this attachment by calculating the transform from the parent link to the child link.
        """
        self.parent_to_child_transform = self.calculate_transform()

    def update_constraint(self) -> None:
        """
        Updates the constraint of this attachment by removing the old constraint if one exists and adding a new one.
        """
        self.remove_constraint_if_exists()
        self.add_fixed_constraint()

    def add_fixed_constraint(self) -> None:
        """
        Adds a fixed constraint between the parent link and the child link.
        """
        self.id = self.parent_link.add_fixed_constraint_with_link(self.child_link)

    def calculate_transform(self) -> Transform:
        """
        Calculates the transform from the parent link to the child link.
        """
        return self.parent_link.get_transform_to_link(self.child_link)

    def remove_constraint_if_exists(self) -> None:
        """
        Removes the constraint between the parent and the child links if one exists.
        """
        if self.child_link in self.parent_link.constraint_ids:
            self.parent_link.remove_constraint_with_link(self.child_link)

    def get_inverse(self) -> 'Attachment':
        """
        :return: A new Attachment object with the parent and child links swapped.
        """
        attachment = Attachment(self.child_link, self.parent_link, self.bidirectional,
                                constraint_id=self.id)
        attachment.loose = not self._loose
        return attachment

    @property
    def loose(self) -> bool:
        """
        If true, then the child object will not move when parent moves.
        """
        return self._loose

    @loose.setter
    def loose(self, loose: bool) -> None:
        """
        Sets the loose property of this attachment.

        :param loose: If true, then the child object will not move when parent moves.
        """
        self._loose = loose and not self.bidirectional

    @property
    def is_reversed(self) -> bool:
        """
        :return: True if the parent and child links are swapped.
        """
        return self.loose

    def __del__(self) -> None:
        """
        Removes the constraint between the parent and the child links if one exists when the attachment is deleted.
        """
        self.remove_constraint_if_exists()

    def __copy__(self):
        return Attachment(self.parent_link, self.child_link, self.bidirectional, self.parent_to_child_transform,
                          self.id)

    def __eq__(self, other):
        return (self.parent_link.name == other.parent_link.name
                and self.child_link.name == other.child_link.name
                and self.bidirectional == other.bidirectional
                and np.allclose(self.parent_to_child_transform.translation_as_list(),
                                other.parent_to_child_transform.translation_as_list(), rtol=0, atol=1e-4)
                and np.allclose(self.parent_to_child_transform.rotation_as_list(),
                                other.parent_to_child_transform.rotation_as_list(), rtol=0, atol=1e-4))

    def __hash__(self):
        return hash((self.parent_link.name, self.child_link.name, self.bidirectional, self.parent_to_child_transform))
