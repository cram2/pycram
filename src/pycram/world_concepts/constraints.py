from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point
from typing_extensions import Union, List, Optional, TYPE_CHECKING, Self

from ..datastructures.enums import JointType
from ..datastructures.pose import TransformStamped, PoseStamped

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
                 parent_to_constraint: TransformStamped,
                 child_to_constraint: TransformStamped):
        self.parent_link: Link = parent_link
        self.child_link: Link = child_link
        self.type: JointType = _type
        self.parent_to_constraint = parent_to_constraint
        self.child_to_constraint = child_to_constraint
        self._parent_to_child = None

    def get_child_object_pose(self) -> PoseStamped:
        """
        :return: The pose of the child object.
        """
        return self.child_link.object.pose

    def get_child_object_pose_given_parent(self, pose: PoseStamped) -> PoseStamped:
        """
        Get the pose of the child object given the parent pose.

        :param pose: The parent object pose.
        :return: The pose of the child object.
        """
        pose = self.parent_link.get_pose_given_object_pose(pose)
        child_link_pose = self.get_child_link_target_pose_given_parent(pose)
        return self.child_link.get_object_pose_given_link_pose(child_link_pose)

    def set_child_link_pose(self):
        """
        Set the target pose of the child object to the current pose of the child object in the parent object frame.
        """
        self.child_link.set_object_pose_given_link_pose(self.get_child_link_target_pose())

    def get_child_link_target_pose(self) -> PoseStamped:
        """
        :return: The target pose of the child object. (The pose of the child object in the parent object frame)
        """
        return self.parent_to_child_transform.to_pose_stamped()

    def get_child_link_target_pose_given_parent(self, parent_pose: PoseStamped) -> PoseStamped:
        """
        Get the target pose of the child object link given the parent link pose.

        :param parent_pose: The parent link pose.
        :return: The target pose of the child object link.
        """
        return (parent_pose.to_transform_stamped(self.parent_link.tf_frame) * self.parent_to_child_transform).to_pose_stamped()

    @property
    def parent_to_child_transform(self) -> TransformStamped:
        """
        Return the transform from the parent link to the child link of the constraint.

        :return: The transform from the parent link to the child link of the constraint.
        """
        if self._parent_to_child is None:
            if self.parent_to_constraint is not None and self.child_to_constraint is not None:
                self._parent_to_child = ~self.parent_to_constraint * self.child_to_constraint
        return self._parent_to_child

    @parent_to_child_transform.setter
    def parent_to_child_transform(self, transform: TransformStamped) -> None:
        self._parent_to_child = transform

    @property
    def parent_object_id(self) -> int:
        """
        :return: The id of the parent object of the constraint
        """
        return self.parent_link.object_id

    @property
    def child_object_id(self) -> int:
        """
        :return: The id of the child object of the constraint
        """
        return self.child_link.object_id

    @property
    def parent_link_id(self) -> int:
        """
        :return: The id of the parent link of the constraint
        """
        return self.parent_link.id

    @property
    def child_link_id(self) -> int:
        """
        :return: The id of the child link of the constraint
        """
        return self.child_link.id

    @property
    def position_wrt_parent_as_list(self) -> List[float]:
        """
        :return: The constraint frame pose with respect to the parent origin as a list
        """
        return self.pose_wrt_parent.position.to_list()

    @property
    def orientation_wrt_parent_as_list(self) -> List[float]:
        """
        :return: The constraint frame orientation with respect to the parent origin as a list
        """
        return self.pose_wrt_parent.orientation.to_list()

    @property
    def pose_wrt_parent(self) -> PoseStamped:
        """
        :return: The joint frame pose with respect to the parent origin
        """
        return self.parent_to_constraint.to_pose_stamped()

    @property
    def position_wrt_child_as_list(self) -> List[float]:
        """
        :return: The constraint frame pose with respect to the child origin as a list
        """
        return self.pose_wrt_child.position.to_list()

    @property
    def orientation_wrt_child_as_list(self) -> List[float]:
        """
        :return: The constraint frame orientation with respect to the child origin as a list
        """
        return self.pose_wrt_child.orientation.to_list()

    @property
    def pose_wrt_child(self) -> PoseStamped:
        """
        :return: The joint frame pose with respect to the child origin
        """
        return self.child_to_constraint.to_pose_stamped()


class Constraint(AbstractConstraint):
    """
    Represents a constraint between two links in the World.
    """

    def __init__(self,
                 parent_link: Link,
                 child_link: Link,
                 _type: JointType,
                 axis_in_child_frame: Point,
                 constraint_to_parent: TransformStamped,
                 child_to_constraint: TransformStamped):
        parent_to_constraint = ~constraint_to_parent
        AbstractConstraint.__init__(self, parent_link, child_link, _type, parent_to_constraint, child_to_constraint)
        self.axis: Point = axis_in_child_frame

    @property
    def axis_as_list(self) -> List[float]:
        """
        :return: The axis of this constraint as a list of xyz
        """
        return [self.axis.x, self.axis.y, self.axis.z]


class Attachment(AbstractConstraint):
    def __init__(self,
                 parent_link: Link,
                 child_link: Link,
                 bidirectional: bool = False,
                 parent_to_child_transform: Optional[TransformStamped] = None,
                 constraint_id: Optional[int] = None,
                 is_inverse: bool = False):
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
                         TransformStamped.from_list(frame=child_link.tf_frame))
        self.id = constraint_id
        self.bidirectional: bool = bidirectional
        self._loose: bool = False
        self.is_inverse: bool = is_inverse

        if parent_to_child_transform is not None:
            self.parent_to_child_transform = parent_to_child_transform

        elif self.parent_to_child_transform is None:
            self.update_transform()

        if self.id is None:
            self.add_fixed_constraint()

    @property
    def parent_object(self):
        return self.parent_link.object

    @property
    def child_object(self):
        return self.child_link.object

    def update_transform_and_constraint(self) -> None:
        """
        Update the transform and constraint of this attachment.
        """
        self.update_transform()
        self.update_constraint()

    def update_transform(self) -> None:
        """
        Update the transform of this attachment by calculating the transform from the parent link to the child link.
        """
        self.parent_to_child_transform = self.calculate_transform()

    def update_constraint(self) -> None:
        """
        Update the constraint of this attachment by removing the old constraint if one exists and adding a new one.
        """
        self.remove_constraint_if_exists()
        self.add_fixed_constraint()

    def add_fixed_constraint(self) -> None:
        """
        Add a fixed constraint between the parent link and the child link.
        """
        self.id = self.parent_link.add_fixed_constraint_with_link(self.child_link,
                                                                  ~self.parent_to_child_transform)

    def calculate_transform(self) -> TransformStamped:
        """
        Calculate the transform from the parent link to the child link.
        """
        return self.parent_link.get_transform_to_link(self.child_link)

    def remove_constraint_if_exists(self) -> None:
        """
        Remove the constraint between the parent and the child links if one exists.
        """
        if self.child_link in self.parent_link.constraint_ids.keys():
            self.parent_link.remove_constraint_with_link(self.child_link)

    def get_inverse(self) -> 'Attachment':
        """
        :return: A new Attachment object with the parent and child links swapped.
        """
        attachment = Attachment(self.child_link, self.parent_link, self.bidirectional,
                                constraint_id=self.id, is_inverse=not self.is_inverse)
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
        Set the loose property of this attachment.

        :param loose: If true, then the child object will not move when parent moves.
        """
        self._loose = loose and not self.bidirectional

    def __del__(self) -> None:
        """
        Remove the constraint between the parent and the child links if one exists when the attachment is deleted.
        """
        self.remove_constraint_if_exists()

    def __copy__(self):
        return Attachment(self.parent_link, self.child_link, self.bidirectional, self.parent_to_child_transform,
                          self.id)

    def __eq__(self, other):
        return (self.parent_link.name == other.parent_link.name
                and self.child_link.name == other.child_link.name
                and self.bidirectional == other.bidirectional
                and self.loose == other.loose
                and np.allclose(self.parent_to_child_transform.translation.to_list(),
                                other.parent_to_child_transform.translation.to_list(), rtol=0, atol=1e-3)
                and np.allclose(self.parent_to_child_transform.rotation.to_list(),
                                other.parent_to_child_transform.rotation.to_list(), rtol=0, atol=1e-3))

    def __hash__(self):
        return hash((self.parent_link.name, self.child_link.name, self.bidirectional, self.parent_to_child_transform))
