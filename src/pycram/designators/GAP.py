from __future__ import annotations
import abc
import inspect
import math
from dataclasses import dataclass, field
from datetime import timedelta
from functools import cached_property
from time import sleep
import numpy as np
from .object_designator import BelieveObject
from ..datastructures.world_entity import PhysicalBody
from ..has_parameters import has_parameters
from ..language import SequentialPlan, TryInOrderPlan
from ..plan import with_plan
from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.dataclasses import FrozenObject
from .. import utils
from ..tf_transformations import quaternion_from_euler
from typing_extensions import List, Union, Optional, Type, Dict, Any, Iterable
from pycrap.ontologies import Location, PhysicalObject
from .location_designator import CostmapLocation
from .motion_designator import MoveJointsMotion, MoveGripperMotion, MoveTCPMotion, MoveMotion, \
    LookingMotion, DetectingMotion, OpeningMotion, ClosingMotion
from ..datastructures.grasp import GraspDescription
from ..datastructures.world import World, UseProspectionWorld
from ..description import Joint, Link, ObjectDescription
from ..designator import ActionDescription, ObjectDesignatorDescription
from ..failure_handling import try_action
from ..failures import TorsoGoalNotReached, ConfigurationNotReached, ObjectNotInGraspingArea, \
    ObjectNotPlacedAtTargetLocation, ObjectStillInContact, LookAtGoalNotReached, \
    ContainerManipulationError
from ..local_transformer import LocalTransformer
from ..failures import ObjectUnfetchable, ReachabilityFailure, NavigationGoalNotReachedError, PerceptionObjectNotFound, \
    ObjectNotGraspedError
from ..robot_description import EndEffectorDescription
from ..ros import sleep
from ..config.action_conf import ActionConfig
from ..datastructures.enums import Arms, Grasp, GripperState, DetectionTechnique, DetectionState, MovementType, \
    TorsoState, StaticJointState, Frame, FindBodyInRegionMethod, ContainerManipulationType
from ..datastructures.pose import PoseStamped
from ..datastructures.world import World
from ..robot_description import RobotDescription, KinematicChainDescription
from ..ros import logwarn, loginfo
from ..validation.error_checkers import PoseErrorChecker
from ..validation.goal_validator import create_multiple_joint_goal_validator
from ..world_concepts.world_object import Object
from ..world_reasoning import move_away_all_objects_to_create_empty_space, generate_object_at_target, \
    cast_a_ray_from_camera, has_gripper_grasped_body, is_body_between_fingers
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Union, Iterable, Type
from datetime import timedelta



@dataclass
class GAP(ActionDescription):
    object_: ObjectDesignatorDescription
    tool: ObjectDesignatorDescription
    arm: Arms
    technique: Optional[str] = None

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        pass

    @abstractmethod
    def plan(self) -> None:
        ...


@has_parameters
@dataclass
class MixingAction(GAP):
    def plan(self) -> None:

        lt = LocalTransformer()
        obj = self.object_
        pose = lt.transform_to_object_frame(obj.pose, obj)
        height_offset = obj.size[2] + 0.05

        for t in range(20):  # 2 * steps, with steps=10
            p = pose.copy()
            r, a, h = 0.0035 * t, math.radians(30) * t, 0.001 * t
            p.pose.position.x += r * math.cos(a)
            p.pose.position.y += r * math.sin(a)
            p.pose.position.z += h

            spiral = lt.transform_pose(p, "map")
            spiral.pose.position.z += height_offset
            World.current_world.add_vis_axis(spiral)
            MoveTCPMotion(spiral, self.arm).perform()

        World.current_world.remove_vis_axis()

    @classmethod
    @with_plan
    def description(cls, object_: Union[Iterable[Object], Object],
                    tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None):
        return PartialDesignator(cls, object_=object_, tool=tool, arm=arm, technique=technique)


@dataclass
class CuttingAction(GAP):
    slice_thickness: Optional[float] = 0.03

    def plan(self) -> None:
        def calculate_slices(self, length, technique, thickness):
            if technique == 'Halving':
                return 1, 0
            return (1, -length / 2 + thickness / 2) if technique in ['Cutting Action', 'Sawing', 'Paring', 'Cutting',
                                                                     'Carving'] \
                else (int(length // thickness), -length / 2 + thickness / 2)

        def lift_pose(self, pose, height):
            p = pose.copy()
            p.position.z += 2 * height
            return p

        def rotate_pose(self, pose, angle_deg):
            p = pose.copy()
            q = utils.axis_angle_to_quaternion([0, 0, 1], angle_deg)
            p.multiply_quaternions(q)
            return p

        def plan(self) -> None:
            tf = LocalTransformer()
            obj = self.object_to_be_cut.world_object
            length, width, height = obj.get_object_dimensions()
            base_pose = tf.transform_to_object_frame(self.object_to_be_cut.pose, obj)

            slices, offset = self.calculate_slices(length, self.technique, self.slice_thickness)
            positions = [offset + i * self.slice_thickness for i in range(slices)]

            gripper = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()

            for x in positions:
                p = base_pose.copy()
                p.pose.position.x = x
                p.pose.position.y += width
                p = tf.transform_pose(p, "map")
                p = self.rotate_pose(p, 180)  # face robot
                p = self.rotate_pose(p, 90)  # perpendicular

                tcp_pose = tf.transform_pose(self.tool.pose, World.robot.get_link_tf_frame(gripper))
                diff = p.to_transform("target").inverse_times(tcp_pose.to_transform("object")).to_pose()
                up = self.lift_pose(diff, height)

                World.current_world.add_vis_axis(diff)
                MoveTCPMotion(up, self.arm).perform()
                MoveTCPMotion(diff, self.arm).perform()
                MoveTCPMotion(up, self.arm).perform()

    @classmethod
    @with_plan
    def description(cls, object_: Union[Iterable[Object], Object],
                    tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None):
        return PartialDesignator(cls, object_=object_, tool=tool, arm=arm, technique=technique)


@dataclass
class PouringAction(GAP):
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
    def description(cls, object_: Union[Iterable[Object], Object],
                    tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None,
                    angle: Optional[Union[Iterable[float], float]] = 90):
        return PartialDesignator(cls, object_=object_, tool=tool, arm=arm, technique=technique, angle=angle)

PouringActionDescription = PouringAction.description
MixingActionDescription = MixingAction.description
CuttingActionDescription = CuttingAction.description