from dataclasses import dataclass
from typing import Optional, List

from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
from semantic_digital_twin.world_description.world_entity import Body

from .base import BaseMotion
from ...datastructures.enums import Arms, GripperState, MovementType, WaypointsMovementType
from ...datastructures.grasp import GraspDescription
from ...datastructures.pose import PoseStamped
from ...failure_handling import try_motion
from ...failures import ToolPoseNotReachedError
from ...joint_state import JointStateManager
from ...process_module import ProcessModuleManager
from ...robot_description import ViewManager
from ...utils import translate_pose_along_local_axis


@dataclass
class ReachMotion(BaseMotion):
    """
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
    movement_type: MovementType = MovementType.CARTESIAN
    """
    The type of movement that should be performed.
    """

    def _calculate_pose_sequence(self) -> List[PoseStamped]:
        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot_view)

        target_pose = GraspDescription.get_grasp_pose(self.grasp_description, end_effector, self.object_designator)
        target_pose.rotate_by_quaternion(GraspDescription.calculate_grasp_orientation(self.grasp_description,
                                                                                      end_effector.front_facing_orientation.to_np()[
                                                                                          :3]))
        target_pre_pose = translate_pose_along_local_axis(target_pose,
                                                          end_effector.front_facing_axis.to_np(),
                                                          -self.object_designator.get_approach_offset())

        pose = PoseStamped.from_spatial_type(self.world.transform(target_pre_pose.to_spatial_type(), self.world.root))

        return [target_pre_pose, pose]

    def perform(self):
        pose_sequence = self._calculate_pose_sequence()

        MoveTCPMotion(pose_sequence[1], self.arm, allow_gripper_collision=False,
                      movement_type=self.movement_type).perform()

    @property
    def _motion_chart(self):
        pass


# @dataclass
# class MoveArmJointsMotion(BaseMotion):
#     """
#     Moves the joints of each arm into the given position
#     """
#
#     left_arm_poses: Optional[Dict[str, float]] = None
#     """
#     Target positions for the left arm joints
#     """
#     right_arm_poses: Optional[Dict[str, float]] = None
#     """
#     Target positions for the right arm joints
#     """
#
#     def perform(self):
#         pm_manager = ProcessModuleManager().get_manager(self.robot_view)
#         return pm_manager.move_arm_joints().execute(self)
#
#     @property
#     def _motion_chart(self):
#         left_connections = [self.world.get_connection_by_name(name) for name in self.left_arm_poses.keys()]
#         right_connections = [self.world.get_connection_by_name(name) for name in self.right_arm_poses.keys()]
#         return JointPositionList()


@dataclass
class MoveGripperMotion(BaseMotion):
    """
    Opens or closes the gripper
    """

    motion: GripperState
    """
    Motion that should be performed, either 'open' or 'close'
    """
    gripper: Arms
    """
    Name of the gripper that should be moved
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper is allowed to collide with something
    """

    def perform(self):
        return
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        return pm_manager.move_gripper().execute(self)

    @property
    def _motion_chart(self):
        gripper_state = JointStateManager().get_gripper_state(self.gripper, self.motion, self.robot_view)
        return JointPositionList(goal_state=JointState(
            mapping={self.world.get_connection_by_name(joint_name): joint_position for joint_name, joint_position in
                     zip(gripper_state.joint_names, gripper_state.joint_positions)}))


@dataclass
class MoveTCPMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: PoseStamped
    """
    Target pose to which the TCP should be moved
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: Optional[MovementType] = MovementType.CARTESIAN
    """
    The type of movement that should be performed.
    """

    def perform(self):
        return
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        try_motion(pm_manager.move_tcp(), self, ToolPoseNotReachedError)

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot_view).tool_frame
        return CartesianPose(root_link=self.robot_view.root, tip_link=tip, goal_pose=self.target.to_spatial_type())


@dataclass
class MoveTCPWaypointsMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    waypoints: List[PoseStamped]
    """
    Waypoints the TCP should move along 
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: WaypointsMovementType = WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT
    """
    The type of movement that should be performed.
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        pm_manager.move_tcp_waypoints().execute(self)

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot_view).tool_frame
        pass
