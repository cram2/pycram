from abc import ABC
from threading import Lock
from typing import Any

import pycram.bullet_world_reasoning as btr
import numpy as np
import time
import rospy
import pybullet as p

from ..plan_failures import EnvironmentManipulationImpossible
from ..robot_descriptions import robot_description
from ..process_module import ProcessModule, ProcessModuleManager
from ..bullet_world import BulletWorld, Object
from ..helper import transform
from ..external_interfaces.ik import request_ik, IKError
from ..helper import _transform_to_torso, _apply_ik, calculate_wrist_tool_offset, inverseTimes
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..enums import JointType
from ..external_interfaces import giskard


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of PR2 and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = BulletWorld.robot
    if arm == "right":
        for joint, pose in robot_description.get_static_joint_chain("right", "park").items():
            robot.set_joint_state(joint, pose)
    if arm == "left":
        for joint, pose in robot_description.get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class Pr2Navigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion.Motion):
        robot = BulletWorld.robot
        robot.set_pose(desig.target)


class Pr2PickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig: PickUpMotion.Motion):
        object = desig.object_desig.bullet_world_object
        robot = BulletWorld.robot
        grasp = robot_description.grasps.get_orientation_for_grasp(desig.grasp)
        target = object.get_pose()
        target.orientation.x = grasp[0]
        target.orientation.y = grasp[1]
        target.orientation.z = grasp[2]
        target.orientation.w = grasp[3]

        arm = desig.arm
        arm_short = "r" if arm == "right" else "l"

        _move_arm_tcp(target, robot, arm)
        tool_frame = robot_description.get_tool_frame(arm)
        robot.attach(object, tool_frame)


class Pr2Place(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig: PlaceMotion.Motion):
        """

        :param desig: A PlaceMotion
        :return:
        """
        object = desig.object.bullet_world_object
        robot = BulletWorld.robot
        arm = desig.arm

        _move_arm_tcp(desig.target, robot, arm)
        robot.detach(object)


class Pr2MoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_state("head_pan_joint")
        current_tilt = robot.get_joint_state("head_tilt_joint")

        robot.set_joint_state("head_pan_joint", new_pan + current_pan)
        robot.set_joint_state("head_tilt_joint", new_tilt + current_tilt)


class Pr2MoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion.Motion):
        robot = BulletWorld.robot
        gripper = desig.gripper
        motion = desig.motion
        for joint, state in robot_description.get_static_gripper_chain(gripper, motion).items():
            robot.set_joint_state(joint, state)


class Pr2Detecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion.Motion):
        robot = BulletWorld.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = robot_description.get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = robot_description.front_facing_axis

        objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)
        for obj in objects:
            if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                return obj


class Pr2MoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        _move_arm_tcp(target, robot, desig.arm)


class Pr2MoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion.Motion):

        robot = BulletWorld.robot
        if desig.right_arm_poses:
            for joint, pose in desig.right_arm_poses.items():
                robot.set_joint_state(joint, pose)
        if desig.left_arm_poses:
            for joint, pose in desig.left_arm_poses.items():
                robot.set_joint_state(joint, pose)


class PR2MoveJoints(ProcessModule):
    def _execute(self, desig: MoveJointsMotion.Motion):
        robot = BulletWorld.robot
        for joint, pose in zip(desig.names, desig.positions):
            robot.set_joint_state(joint, pose)


class Pr2WorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion.Motion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class Pr2Open(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[1])


class Pr2Close(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion.Motion):
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


###########################################################
########## Process Modules for the Real PR2 ###############
###########################################################


class Pr2NavigationReal(ProcessModule):
    """
    Process module for the real PR2 that sends a cartesian goal to giskard to move the robot base
    """
    def _execute(self, designator: MoveMotion.Motion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")


class Pr2PickUpReal(ProcessModule):

    def _execute(self, designator: PickUpMotion.Motion) -> Any:
        pass


class Pr2PlaceReal(ProcessModule):

    def _execute(self, designator: MotionDesignatorDescription.Motion) -> Any:
        pass


class Pr2MoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """
    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_state("head_pan_joint")
        current_tilt = robot.get_joint_state("head_tilt_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_pan_joint": new_pan + current_pan,
                                    "head_tilt_joint": new_tilt + current_tilt})


class Pr2DetectingReal(ProcessModule):
    """
    Process Module for the real Pr2 that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """
    def _execute(self, designator: DetectingMotion.Motion) -> Any:
        pass


class Pr2MoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real PR2 while avoiding all collisions
    """
    def _execute(self, designator: MoveTCPMotion.Motion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")

        giskard.avoid_all_collisions()
        giskard.achieve_cartesian_goal(pose_in_map, robot_description.get_tool_frame(designator.arm),
                                       robot_description.base_link)


class Pr2MoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real PR2 to the given configuration while avoiding all collisions
    """
    def _execute(self, designator: MoveArmJointsMotion.Motion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class Pr2MoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """
    def _execute(self, designator: MoveJointsMotion.Motion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class Pr2MoveGripperReal(ProcessModule):

    def _execute(self, designator: MoveGripperMotion.Motion) -> Any:
        gripper = designator.gripper
        motion = designator.motion
        joint_goals = robot_description.get_static_gripper_chain(gripper, motion).items()

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class Pr2OpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """
    def _execute(self, designator: OpeningMotion.Motion) -> Any:
        giskard.achieve_open_container_goal(robot_description.get_tool_frame(designator.arm),
                                            designator.object_part.name)


class Pr2CloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """
    def _execute(self, designator: ClosingMotion.Motion) -> Any:
        giskard.achieve_close_container_goal(robot_description.get_tool_frame(designator.arm),
                                             designator.object_part.name)


class Pr2Manager(ProcessModuleManager):

    def __init__(self):
        super().__init__("pr2")
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
            return Pr2Navigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return Pr2NavigationReal(self._navigate_lock)

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2PickUp(self._pick_up_lock)
        elif ProcessModuleManager.execution_type == "real":
            return Pr2PickUpReal(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2Place(self._place_lock)
        elif ProcessModuleManager.execution_type == "real":
            return Pr2PlaceReal(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2MoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return Pr2MoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2Detecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2MoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return Pr2MoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2MoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return Pr2MoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return Pr2WorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return PR2MoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return Pr2MoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2MoveGripper(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2Open(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return Pr2OpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2Close(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return Pr2CloseReal(self._close_lock)
