from threading import Lock
from typing import Any

from ..helper import _apply_ik
from .default_process_modules import *


class StretchNavigate(DefaultNavigation):
    pass


class StretchMoveHead(ProcessModule):
    def _execute(self, designator) -> Any:
        target = designator.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_pan"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_tilt"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(-pose_in_tilt.position.y, pose_in_tilt.position.z ** 2 + pose_in_tilt.position.x ** 2) * -1

        current_pan = robot.get_joint_state("joint_head_pan")
        current_tilt = robot.get_joint_state("joint_head_tilt")

        robot.set_joint_state("joint_head_pan", new_pan + current_pan)
        robot.set_joint_state("joint_head_tilt", new_tilt + current_tilt)


class StretchMoveGripper(DefaultMoveGripper):
    pass


class StretchDetecting(DefaultDetecting):
    pass


class StretchMoveTCP(DefaultMoveTCP):
    pass


class StretchMoveArmJoints(DefaultMoveArmJoints):
    pass


class StretchMoveJoints(DefaultMoveJoints):
    pass


class StretchWorldStateDetecting(DefaultWorldStateDetecting):
    pass


class StretchOpen(ProcessModule):
    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[1] - 0.05)


class StretchClose(ProcessModule):
    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: str) -> None:
    gripper = robot_description.get_tool_frame(arm)

    joints = robot_description.chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv, joints)


class StretchManager(ProcessModuleManager):
    def __init__(self):
        super().__init__("stretch")
        self._navigate_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchNavigate(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveTCP(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveArmJoints(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return StretchWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveJoints(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveGripper(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchOpen(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchClose(self._close_lock)

