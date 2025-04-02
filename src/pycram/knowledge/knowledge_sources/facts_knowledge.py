from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from typing_extensions import List

from ...datastructures.dataclasses import ReasoningResult
from ...datastructures.enums import Arms, ObjectType, Grasp
from ..knowledge_source import KnowledgeSource
from ...datastructures.property import ReachableProperty, GraspableProperty, GripperIsFreeProperty, VisibleProperty, \
    SpaceIsFreeProperty, EmptyProperty
from ...datastructures.pose import PoseStamped
from ...datastructures.world import World, UseProspectionWorld
from ...pose_generator_and_validator import PoseGenerator, reachability_validator
from ...robot_description import RobotDescription
from ...world_reasoning import visible
from ...costmaps import OccupancyCostmap, GaussianCostmap
from ...units import meter

if TYPE_CHECKING:
    from ...designators.object_designator import ObjectDesignatorDescription

class FactsKnowledge(KnowledgeSource, GripperIsFreeProperty, VisibleProperty, SpaceIsFreeProperty, GraspableProperty, ReachableProperty, EmptyProperty):
    """
    Knowledge source for hard coded facts, this knowledge source acts as a fallback if no other knowledge source is
    available.
    """

    def __init__(self):
        super().__init__(name="Facts", priority=99)

    @property
    def is_available(self) -> bool:
        return True

    @property
    def is_connected(self) -> bool:
        return True

    def connect(self):
        pass

    def clear_state(self) -> None:
        pass

    def reachable(self, pose: PoseStamped) -> ReasoningResult:
        """
        Check if a given pose is reachable by the robot. Simplified version of the CostmapLocation which can't be used
        here due to cyclic imports.

        :param pose: Pose which should be checked
        :return: A ReasoningResult with the result of the check and possible arms
        """
        # ground_pose = Pose(pose.position.to_list())
        # ground_pose.position.z = 0
        # occupancy = OccupancyCostmap(0.32, False, 200, 0.02, ground_pose)
        # gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
        # final_map = occupancy +  gaussian
        #
        # with UseProspectionWorld():
        #     test_robot = World.current_world.get_prospection_object_for_object(World.robot)
        #     for maybe_pose in PoseGenerator(final_map, number_of_samples=200):
        #         hand_links = []
        #         for description in RobotDescription.current_robot_description.get_manipulator_chains():
        #             hand_links += description.end_effector.links
        #         valid, arms = reachability_validator(maybe_pose, test_robot, pose,
        #                                              allowed_collision={test_robot: hand_links})
        #         if valid:
        #             return ReasoningResult(True, {"arm": arms})
        #
        # return ReasoningResult(False)
        return ReasoningResult(True)

    def graspable(self, object_designator: ObjectDesignatorDescription) -> ReasoningResult:
        """
         Check if the object is graspable by the robot.

        :param object_designator: Designator of the object which should be grasped
        :return: Reasoning Result with the result and a possible grasp
        """
        with UseProspectionWorld():
            object_desig = object_designator.resolve() if hasattr(object_designator, "resolve") else object_designator
            pro_obj = World.current_world.get_prospection_object_for_object(object_desig.world_object)
            bounding_box = pro_obj.get_axis_aligned_bounding_box(False)

            obj_x = bounding_box.max_x - bounding_box.min_x
            obj_y = bounding_box.max_y - bounding_box.min_y
            obj_z = bounding_box.max_z - bounding_box.min_z
            gripper_opening_dists = [ee.end_effector.opening_distance for ee in
                                     RobotDescription.current_robot_description.get_manipulator_chains()]

            for dist in gripper_opening_dists:
                if dist > obj_y * meter:
                    return ReasoningResult(True, {"grasp": Grasp.FRONT})
                elif dist > obj_x * meter:
                    return ReasoningResult(True, {"grasp": Grasp.LEFT})

            return ReasoningResult(False)

    def space_is_free(self, pose: PoseStamped) -> ReasoningResult:
        om = OccupancyCostmap(0.35, False, 200, 0.02, pose)
        origin_map = om.map[200 // 2 - 10: 200 // 2 + 10, 200 // 2 - 10: 200 // 2 + 10]
        return ReasoningResult(np.sum(origin_map) > 400 * 0.9)

    def gripper_is_free(self, grippers: List[Arms]) -> ReasoningResult:
        """
        Checks for a list of grippers if they are holding something.

        :param grippers: A list of gripper that should be checked
        :return: Result if a gripper is free and the resulting gripper
        """
        for gripper in grippers:
            tool_frame_link = RobotDescription.current_robot_description.get_arm_chain(gripper).get_tool_frame()
            for att in World.robot.attachments.values():
                if att.parent_link.name == tool_frame_link or att.child_link.name == tool_frame_link:
                    return ReasoningResult(False)
            return ReasoningResult(True, {"gripper": gripper})

    def is_visible(self, object_designator: ObjectDesignatorDescription) -> ReasoningResult:
        """
        Checks if an object is visible for the robot.

        :param object_designator: The object in question
        :return: Reasoning result with the visibility of the object
        """
        cam_pose = World.robot.get_link_pose(RobotDescription.current_robot_description.get_camera_link())
        return ReasoningResult(visible(object_designator.resolve().world_object, cam_pose))

    def empty(self) -> ReasoningResult:
        """
        Default property, which is always true.

        :return: Reasoning result with True
        """
        return ReasoningResult(True)
