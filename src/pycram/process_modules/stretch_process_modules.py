from .default_process_modules import *
from ..external_interfaces.ik import request_giskard_ik
from typing_extensions import Any


class StretchOpen(DefaultOpen):
    """
    Process module for the simulated Stretch that opens an already grasped container
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[1] - 0.05)


def _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    # inv = request_ik(target, robot, joints, gripper)
    pose, joint_states = request_giskard_ik(target, robot, gripper)
    robot.set_pose(pose)
    robot.set_joint_positions(joint_states)


###########################################################
########## Process Modules for the Real Stretch ###########
###########################################################


class StretchMoveGripperReal(DefaultMoveGripperReal):
    """
    Opens or closes the gripper of the real Stretch, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        chain = RobotDescription.current_robot_description.get_arm_chain(designator.gripper).get_static_gripper_state(
            designator.motion)
        giskard.achieve_joint_goal(chain)


class StretchManager(DefaultManager):
    def __init__(self):
        super().__init__()
        self.robot_name = "stretch"

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return StretchMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type ==  ExecutionType.SIMULATED:
            return StretchOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultOpenReal(self._open_lock)
