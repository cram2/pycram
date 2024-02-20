from threading import Lock

import numpy as np

from ..process_module import ProcessModule, ProcessModuleManager
from ..process_modules.pr2_process_modules import Pr2Navigation as DefaultNavigation, Pr2PickUp as DefaultPickUp, \
    Pr2Place as DefaultPlace, Pr2WorldStateDetecting as DefaultWorldStateDetecting, Pr2Open as DefaultOpen, \
    Pr2Close as DefaultClose, Pr2MoveGripper as DefaultMoveGripper, Pr2Detecting as DefaultDetecting, \
    Pr2MoveTCP as DefaultMoveTCP
from ..robot_descriptions import robot_description
from ..world import World
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import LookingMotion, MoveArmJointsMotion, MoveJointsMotion


class DefaultMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()

        pan_link = robot_description.chains["neck"].links[0]
        tilt_link = robot_description.chains["neck"].links[1]

        pan_joint = robot_description.chains["neck"].joints[0]
        tilt_joint = robot_description.chains["neck"].joints[1]
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame(pan_link))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame(tilt_link))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_position(pan_joint)
        current_tilt = robot.get_joint_position(tilt_joint)

        robot.set_joint_position(pan_joint, new_pan + current_pan)
        robot.set_joint_position(tilt_joint, new_tilt + current_tilt)


class DefaultMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion.Motion):

        robot = World.robot
        if desig.right_arm_poses:
            for joint, pose in desig.right_arm_poses.items():
                robot.set_joint_position(joint, pose)
        if desig.left_arm_poses:
            for joint, pose in desig.left_arm_poses.items():
                robot.set_joint_position(joint, pose)


class DefaultMoveJoints(ProcessModule):
    def _execute(self, desig: MoveJointsMotion.Motion):
        robot = World.robot
        for joint, pose in zip(desig.names, desig.positions):
            robot.set_joint_position(joint, pose)


class DefaultManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("default")
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
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
            return DefaultNavigation(self._navigate_lock)

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultPickUp(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultPlace(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultMoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultMoveTCP(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultMoveArmJoints(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultMoveJoints(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultMoveGripper(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultOpen(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return DefaultClose(self._close_lock)
