from typing import Tuple

from pycram.robot_plans.actions.base import ActionDescription
from __future__ import annotations

from dataclasses import dataclass

from pycram.plan import with_plan

from pycram.datastructures.partial_designator import PartialDesignator

from pycram import utils
from typing_extensions import Union, Optional, Iterable

from pycram.local_transformer import LocalTransformer

from pycram.datastructures.enums import Arms, AxisIdentifier

from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.world import World

from pycram.world_concepts.world_object import Object


@dataclass
class CuttingAction(ActionDescription):
    object_: Object
    tool: Object
    arm: Arms
    technique: Optional[str] = None
    slice_thickness: Optional[float] = 0.03

    def plan(self) -> None:
        if self.technique is None:
            self.technique = "Slicing"
        lt = LocalTransformer()
        obj = self.object_
        tool = self.tool
        pose = lt.transform_to_object_frame(obj.pose, obj)

        height = obj.size[2]
        length = max(obj.size[0], obj.size[1])
        length_tool = max(tool.size[0], tool.size[1])

        num_slices, start_offset = self.calculate_slices(length)
        slice_coordinates = [start_offset + i * self.slice_thickness for i in range(num_slices)]

        slice_poses = []
        for x in slice_coordinates:
            tmp_pose = pose.copy()
            tmp_pose.pose.position.x = x
            slice_poses.append(tmp_pose)

        for slice_pose in slice_poses:
            pose_a = obj.pose
            pose_b = World.robot.pose
            angle, angle_y = self.get_rotation_offset_from_axis_preference(pose_a, pose_b)
            direction = 1 if angle_y >= 0 else -1
            slice_pose.pose.position.y += direction * (length_tool / 2)

            new_pose = self.perpendicular_pose(slice_pose=slice_pose, angle=angle)
            final_pose = lt.transform_pose(new_pose, "map")

            print(final_pose)
            World.current_world.add_vis_axis(final_pose)

            lift_pose = new_pose.copy()
            lift_pose.pose.position.z += height

    @classmethod
    @with_plan
    def description(cls, object_: Union[Iterable[Object], Object], tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None, slice_thickness: Optional[float] = 0.03):
        return PartialDesignator(cls, object_=object_, tool=tool, arm=arm, technique=technique,
                                 slice_thickness=slice_thickness)

    def calculate_slices(self, obj_length):
        if self.technique == 'Halving':
            return 1, 0
        if self.technique in ['Cutting Action', 'Sawing', 'Paring', 'Cutting', 'Carving', 'Slicing']:
            num_slices = int(obj_length // self.slice_thickness)
            start_offset = (-obj_length / 2) + (self.slice_thickness / 2)
            return num_slices, start_offset
        return 0, 0

    @staticmethod
    def perpendicular_pose(slice_pose, angle) -> PoseStamped:
        pose_rotated = slice_pose
        rotation_quaternion = utils.axis_angle_to_quaternion([0, 0, 1], angle)
        pose_rotated.rotate_by_quaternion(rotation_quaternion)
        return pose_rotated

    @staticmethod
    def get_rotation_offset_from_axis_preference(pose_a, pose_b: PoseStamped) -> Tuple[int, float]:
        """
        Compute a discrete rotation offset (-90 or 90 degrees) to align this pose's local axes with the direction
        toward a target pose, based on which axis (X or Y) is more aligned.

        :param pose_a: The source pose.
        :param pose_b: The target pose to align with.
        :return: Tuple of (rotation offset in degrees, signed angle difference in radians for Y axis).
        """
        fx, ax = pose_a.is_facing_2d_axis(pose_b, axis=AxisIdentifier.X)
        fy, ay = pose_a.is_facing_2d_axis(pose_b, axis=AxisIdentifier.Y)

        return (-90 if abs(ax) > abs(ay) else 90), ay

CuttingActionDescription = CuttingAction.description