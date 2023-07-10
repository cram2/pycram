from abc import ABC
from threading import Lock

import pycram.bullet_world_reasoning as btr
import numpy as np
import time
import pybullet as p

from ..plan_failures import EnvironmentManipulationImpossible
from ..robot_descriptions import robot_description
from ..process_module import ProcessModule, ProcessModuleManager
from ..bullet_world import BulletWorld
from ..helper import transform
from ..external_interfaces.ik import request_ik, IKError
from ..helper import _transform_to_torso, _apply_ik, calculate_wrist_tool_offset, inverseTimes
from ..local_transformer import local_transformer
from ..designators.motion_designator import *
from ..enums import JointType


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
        robot.set_position_and_orientation(desig.target[0], desig.target[1])
        local_transformer.update_from_btr()


class Pr2PickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig: PickUpMotion.Motion):
        object = desig.object_desig.bullet_world_object
        robot = BulletWorld.robot
        grasp = robot_description.grasps.get_orientation_for_grasp(desig.grasp)
        target = [object.get_position(), grasp]
        target = _transform_to_torso(target, robot)
        arm = desig.arm
        arm_short = "r" if arm == "right" else "l"
        diff = calculate_wrist_tool_offset(arm_short + "_wrist_roll_link", arm_short + "_gripper_tool_frame", robot)
        target = inverseTimes(target, diff)

        joints = robot_description._safely_access_chains(arm).joints

        # Get Link before first joint in chain
        base_link = robot_description.get_parent(joints[0])
        # Get link after last joint in chain
        end_effector = robot_description.get_child(joints[-1])
        inv = request_ik(base_link, end_effector, target, robot, joints)
        _apply_ik(robot, inv, joints)
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
        target = desig.target
        target = _transform_to_torso(target, robot)
        arm = desig.arm
        arm_short = "r" if arm == "right" else "l"
        diff = calculate_wrist_tool_offset(arm_short + "_wrist_roll_link", arm_short + "_gripper_tool_frame", robot)
        target = inverseTimes(target, diff)
        joints = robot_description._safely_access_chains(arm).joints

        # Get Link before first joint in chain
        base_link = robot_description.get_parent(joints[0])
        # Get link after last joint in chain
        end_effector = robot_description.get_child(joints[-1])

        inv = request_ik(base_link, end_effector, target, robot, joints)
        _apply_ik(robot, inv, joints)
        robot.detach(object)


class Pr2MoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot
        if type(target) is str:
            target_frame = local_transformer.projection_namespace + '/' + target \
                if local_transformer.projection_namespace \
                else target
            target = local_transformer.tf_transform(local_transformer.map_frame, target_frame)[0]

        pan_transform = p.invertTransform(robot.get_link_position("head_pan_link"),
                                          robot.get_link_orientation("head_pan_link"))
        # Flattens everything in one list because of 'transform'
        pan_transform = [i for sublist in pan_transform for i in sublist]

        tilt_transform = p.invertTransform(robot.get_link_position("head_tilt_link"),
                                           robot.get_link_orientation("head_tilt_link"))
        # Flattens everything in one list because of 'transform'
        tilt_transform = [i for sublist in tilt_transform for i in sublist]

        target = [i for sublist in target for i in sublist]
        pose_in_pan = transform(target, pan_transform)[:3]
        pose_in_tilt = transform(target, tilt_transform)[:3]

        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])
        new_tilt = np.arctan2(pose_in_tilt[2], pose_in_tilt[0] ** 2 + pose_in_tilt[1] ** 2) * -1

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
            if btr.visible(obj, robot.get_link_position_and_orientation(cam_frame_name), front_facing_axis):
                return obj


class Pr2MoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot
        target = _transform_to_torso(target, robot)
        arm_short = "r" if desig.arm == "right" else "l"
        diff = calculate_wrist_tool_offset(arm_short + "_wrist_roll_link", arm_short + "_gripper_tool_frame", robot)
        target = inverseTimes(target, diff)
        robot = BulletWorld.robot

        joints = robot_description._safely_access_chains(desig.arm).joints

        # Get Link before first joint in chain
        base_link = robot_description.get_parent(joints[0])
        # Get link after last joint in chain
        end_effector = robot_description.get_child(joints[-1])

        inv = request_ik(base_link, end_effector, target, robot, joints)

        _apply_ik(robot, inv, joints)


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
    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)


        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[0])


def _move_arm_tcp(target, robot, arm):
    target = _transform_to_torso(target, robot)
    arm_short = "r" if arm == "right" else "l"
    diff = calculate_wrist_tool_offset(arm_short + "_wrist_roll_link", arm_short + "_gripper_tool_frame", robot)
    target = inverseTimes(target, diff)
    robot = BulletWorld.robot

    joints = robot_description._safely_access_chains(arm).joints

    # Get Link before first joint in chain
    base_link = robot_description.get_parent(joints[0])
    # Get link after last joint in chain
    end_effector = robot_description.get_child(joints[-1])

    inv = request_ik(base_link, end_effector, target, robot, joints)
    _apply_ik(robot, inv, joints)


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

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2PickUp(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2Place(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2MoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2Detecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2MoveTCP(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2MoveArmJoints(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2WorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return PR2MoveJoints(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2MoveGripper(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2Open(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return Pr2Close(self._close_lock)
