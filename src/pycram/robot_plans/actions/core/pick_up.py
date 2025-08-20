from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta
from functools import cached_property

from semantic_world.world_entity import Body
from typing_extensions import Union, Optional, Type, Any, Iterable

from ...motions.gripper import MoveGripperMotion, MoveTCPMotion
from ....config.action_conf import ActionConfig
from ....datastructures.dataclasses import FrozenObject
from ....datastructures.enums import Arms, Grasp, GripperState, MovementType, \
    Frame, FindBodyInRegionMethod
from ....datastructures.grasp import GraspDescription
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....failures import ObjectNotGraspedError
from ....failures import ObjectNotInGraspingArea
from ....has_parameters import has_parameters
from ....language import SequentialPlan
from ....robot_description import EndEffectorDescription, ViewManager
from ....robot_description import RobotDescription, KinematicChainDescription
from ....robot_plans.actions.base import ActionDescription, record_object_pre_perform
from ....ros import logwarn
from ....world_reasoning import has_gripper_grasped_body, is_body_between_fingers


@has_parameters
@dataclass
class ReachToPickUpAction(ActionDescription):
    """
    Let the robot reach a specific pose.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be picked up
    """

    arm: Arms
    """
    The arm that should be used for pick up
    """

    grasp_description: GraspDescription
    """
    The grasp description that should be used for picking up the object
    """

    object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the world object is changed.
    """
    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

        # Store the object's data copy at execution
        self.pre_perform(record_object_pre_perform)

    def plan(self) -> None:

        # target_pose = self.object_designator.get_grasp_pose(self.end_effector, self.grasp_description)
        robot_view = ViewManager().find_robot_view_for_world(self.world)
        end_effector = ViewManager.get_arm_view(self.arm, robot_view)

        target_pose = self.grasp_description.get_grasp_pose(end_effector, self.object_designator)
        # target_pose.rotate_by_quaternion(self.end_effector.grasps[self.grasp_description])

        target_pre_pose = LocalTransformer().translate_pose_along_local_axis(target_pose,
                                                                             self.end_effector.get_approach_axis(),
                                                                             -self.object_designator.get_approach_offset())

        SequentialPlan(self.context, MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm)).perform()

        self.move_gripper_to_pose(target_pre_pose)

        self.move_gripper_to_pose(target_pose, MovementType.STRAIGHT_CARTESIAN)

        # Remove the vis axis from the world if it was added
        World.current_world.remove_vis_axis()

    def move_gripper_to_pose(self, pose: PoseStamped, movement_type: MovementType = MovementType.CARTESIAN,
                             add_vis_axis: bool = True):
        """
        Move the gripper to a specific pose.

        :param pose: The pose to go to.
        :param movement_type: The type of movement that should be performed.
        :param add_vis_axis: If a visual axis should be added to the world.
        """
        pose = self.local_transformer.transform_pose(pose, Frame.Map.value)
        if add_vis_axis:
            World.current_world.add_vis_axis(pose)
        SequentialPlan(self.context,
                       MoveTCPMotion(pose, self.arm, allow_gripper_collision=False,
                                     movement_type=movement_type)).perform()

    @cached_property
    def local_transformer(self) -> LocalTransformer:
        return LocalTransformer()

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

    @cached_property
    def end_effector(self) -> EndEffectorDescription:
        return self.arm_chain.end_effector

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if object is contained in the gripper such that it can be grasped and picked up.
        """
        fingers_link_names = self.arm_chain.end_effector.fingers_link_names
        if fingers_link_names:
            if not is_body_between_fingers(self.object_designator, fingers_link_names,
                                           method=FindBodyInRegionMethod.MultiRay):
                raise ObjectNotInGraspingArea(self.object_designator, World.robot, self.arm, self.grasp_description)
        else:
            logwarn(f"Cannot validate reaching to pick up action for arm {self.arm} as no finger links are defined.")

    @classmethod
    def description(cls, object_designator: Union[Iterable[Body], Body],
                    arm: Union[Iterable[Arms], Arms] = None,
                    grasp_description: Union[Iterable[GraspDescription], GraspDescription] = None) -> PartialDesignator[
        Type[ReachToPickUpAction]]:
        return PartialDesignator(ReachToPickUpAction, object_designator=object_designator,
                                 arm=arm,
                                 grasp_description=grasp_description)


@has_parameters
@dataclass
class PickUpAction(ActionDescription):
    """
    Let the robot pick up an object.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be picked up
    """

    arm: Arms
    """
    The arm that should be used for pick up
    """

    grasp_description: GraspDescription
    """
    The GraspDescription that should be used for picking up the object
    """

    object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

        # Store the object's data copy at execution
        self.pre_perform(record_object_pre_perform)

    def plan(self) -> None:
        SequentialPlan(self.context,
                       ReachToPickUpActionDescription(self.object_designator, self.arm, self.grasp_description),

                       MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm)).perform()

        tool_frame = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
        World.robot.attach(self.object_designator, tool_frame)

        self.lift_object(distance=0.1)

        # Remove the vis axis from the world
        World.current_world.remove_vis_axis()

    def lift_object(self, distance: float = 0.1):
        lift_to_pose = self.gripper_pose()
        lift_to_pose.pose.position.z += distance
        SequentialPlan(self.context, MoveTCPMotion(lift_to_pose, self.arm, allow_gripper_collision=True)).perform()

    def gripper_pose(self) -> PoseStamped:
        """
        Get the pose of the gripper.

        :return: The pose of the gripper.
        """
        gripper_link = self.arm_chain.get_tool_frame()
        return World.robot.links[gripper_link].pose

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if picked up object is in contact with the gripper.
        """
        if not has_gripper_grasped_body(self.arm, self.object_designator):
            raise ObjectNotGraspedError(self.object_designator, World.robot, self.arm, self.grasp_description)

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

    @classmethod
    def description(cls, object_designator: Union[Iterable[Body], Body],
                    arm: Union[Iterable[Arms], Arms] = None,
                    grasp_description: Union[Iterable[GraspDescription], GraspDescription] = None) -> \
            PartialDesignator[Type[PickUpAction]]:
        return PartialDesignator(PickUpAction, object_designator=object_designator, arm=arm,
                                 grasp_description=grasp_description)


@has_parameters
@dataclass
class GraspingAction(ActionDescription):
    """
    Grasps an object described by the given Object Designator description
    """
    object_designator: Body  # Union[Object, ObjectDescription.Link]
    """
    Object Designator for the object that should be grasped
    """
    arm: Arms
    """
    The arm that should be used to grasp
    """
    prepose_distance: float = ActionConfig.grasping_prepose_distance
    """
    The distance in meters the gripper should be at before grasping the object
    """

    def plan(self) -> None:
        object_pose = self.object_designator.pose
        lt = LocalTransformer()
        gripper_name = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()

        object_pose_in_gripper = lt.transform_pose(object_pose,
                                                   World.robot.get_link_tf_frame(gripper_name))

        pre_grasp = object_pose_in_gripper.copy()
        pre_grasp.pose.position.x -= self.prepose_distance

        SequentialPlan(self.context, MoveTCPMotion(pre_grasp, self.arm),
                       MoveGripperMotion(GripperState.OPEN, self.arm),

                       MoveTCPMotion(object_pose, self.arm, allow_gripper_collision=True),
                       MoveGripperMotion(GripperState.CLOSE, self.arm, allow_gripper_collision=True)).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        body = self.object_designator
        contact_links = body.get_contact_points_with_body(World.robot).get_all_bodies()
        arm_chain = RobotDescription.current_robot_description.get_arm_chain(self.arm)
        gripper_links = arm_chain.end_effector.links
        if not any([link.name in gripper_links for link in contact_links]):
            raise ObjectNotGraspedError(self.object_designator, World.robot, self.arm, None)

    @classmethod
    def description(cls, object_designator: Union[Iterable[Body], Body],
                    arm: Union[Iterable[Arms], Arms] = None,
                    prepose_distance: Union[Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> \
            PartialDesignator[Type[GraspingAction]]:
        return PartialDesignator(GraspingAction, object_designator=object_designator, arm=arm,
                                 prepose_distance=prepose_distance)


ReachToPickUpActionDescription = ReachToPickUpAction.description
PickUpActionDescription = PickUpAction.description
GraspingActionDescription = GraspingAction.description
