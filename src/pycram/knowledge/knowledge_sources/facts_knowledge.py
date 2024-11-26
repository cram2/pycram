from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from ...datastructures.dataclasses import ReasoningResult
from ...datastructures.enums import Arms, ObjectType
from ..knowledge_source import KnowledgeSource
from ...datastructures.property import ReachableProperty, GraspableProperty, GripperIsFreeProperty, VisibleProperty, \
    SpaceIsFreeProperty, EmptyProperty
from ...datastructures.pose import Pose
from ...datastructures.world import World, UseProspectionWorld
# from ...designators.location_designator import CostmapLocation
# from ...designators.object_designator import BelieveObject
from ...robot_description import RobotDescription
from ...world_reasoning import visible
from ...costmaps import OccupancyCostmap

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

    # def reachable(self, pose: Pose) -> bool:
    #     robot_desig = BelieveObject(types=[ObjectType.ROBOT])
    #     c = CostmapLocation(pose, reachable_for=robot_desig.resolve()).resolve()
    #     if c.pose:
    #         return True

    def reachable(self, pose: Pose) -> ReasoningResult:
        return ReasoningResult(True)

    def graspable(self, object_designator: ObjectDesignatorDescription) -> ReasoningResult:
        with UseProspectionWorld():
            pro_obj = World.current_world.get_prospection_object_for_object(object_designator.resolve().world_object)
            pro_obj.set_pose(Pose([0, 0, 0], [0, 0, 0, 1]))
            bounding_box = pro_obj.get_axis_aligned_bounding_box()

            obj_x = bounding_box.max_x - bounding_box.min_x
            obj_y = bounding_box.max_y - bounding_box.min_y
            obj_z = bounding_box.max_z - bounding_box.min_z
            gripper_opening_dists = [ee.end_effector.opening_distance for ee in
                                     RobotDescription.current_robot_description.get_manipulator_chains()]
            for dist in gripper_opening_dists:
                if dist > obj_x or dist > obj_y or dist > obj_z:
                    return ReasoningResult(True)
            return ReasoningResult(False)

    def space_is_free(self, pose: Pose) -> ReasoningResult:
        om = OccupancyCostmap(0.35, False, 200, 0.02, pose)
        origin_map = om.map[200 // 2 - 10: 200 // 2 + 10, 200 // 2 - 10: 200 // 2 + 10]
        return ReasoningResult(np.sum(origin_map) > 400 * 0.9)

    def gripper_is_free(self, gripper: Arms) -> ReasoningResult:
        tool_frame_link = RobotDescription.current_robot_description.get_arm_chain(gripper).get_tool_frame()
        for att in World.robot.attachments.values():
            if att.parent_link == tool_frame_link or att.child_link == tool_frame_link:
                return ReasoningResult(False)
        return ReasoningResult(True)

    def is_visible(self, object_designator: ObjectDesignatorDescription) -> ReasoningResult:
        cam_pose = World.robot.get_link_pose(RobotDescription.current_robot_description.get_camera_frame())
        return ReasoningResult(visible(object_designator.resolve().world_object, cam_pose))

    def empty(self) -> ReasoningResult:
        return ReasoningResult(True)
