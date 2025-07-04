from pycram.robot_plans.actions.base import ActionDescription
from __future__ import annotations

from dataclasses import dataclass
from time import sleep

from pycram.plan import with_plan

from pycram.datastructures.partial_designator import PartialDesignator

from pycram import utils
from typing_extensions import Union, Optional, Iterable

from pycram.local_transformer import LocalTransformer
from pycram.ros import sleep

from pycram.datastructures.enums import Arms, Grasp

from pycram.datastructures.world import World

from pycram.robot_description import RobotDescription
from pycram.world_concepts.world_object import Object


@dataclass
class PouringAction(ActionDescription):
    object_: Object
    tool: Object
    arm: Arms
    technique: Optional[str] = None
    angle: Optional[float] = 90

    def plan(self) -> None:
        lt = LocalTransformer()
        gripper_frame = World.robot.get_link_tf_frame("base_link")
        grasp_rot = RobotDescription.current_robot_description.get_arm_chain(self.arm).end_effector.get_grasp(
            Grasp.FRONT, None, False)

        pose = lt.transform_pose(self.object_.pose, gripper_frame)
        pose.pose.position.x += 0.009
        pose.pose.position.y -= 0.125
        pose.pose.position.z += 0.17

        pose = lt.transform_pose(lt.transform_pose(pose, "map"), gripper_frame)
        pose.orientation = grasp_rot
        pose = lt.transform_pose(pose, "map")

        World.current_world.add_vis_axis(pose)
        # MoveTCPMotion(pose, self.arm, allow_gripper_collision=False, movement_type=MovementType.CARTESIAN).perform()

        pour_pose = pose.copy()
        pour_pose.rotate_by_quaternion(utils.axis_angle_to_quaternion([1, 0, 0], -self.angle))
        World.current_world.add_vis_axis(pour_pose)

        # MoveTCPMotion(pour_pose, self.arm, allow_gripper_collision=False,movement_type=MovementType.CARTESIAN).perform()
        sleep(3)
        # MoveTCPMotion(pose, self.arm, allow_gripper_collision=False, movement_type=MovementType.CARTESIAN).perform()

        World.current_world.remove_vis_axis()

    @classmethod
    @with_plan
    def description(cls, object_: Union[Iterable[Object], Object], tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None,
                    angle: Optional[Union[Iterable[float], float]] = 90):
        return PartialDesignator(cls, object_=object_, tool=tool, arm=arm, technique=technique, angle=angle)

PouringActionDescription = PouringAction.description