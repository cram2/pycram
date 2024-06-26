from threading import Lock

import numpy as np

from ..datastructures.enums import JointType
from ..external_interfaces.ik import request_ik
from ..utils import _apply_ik
from ..process_module import ProcessModule
from ..robot_description import RobotDescription
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..world_reasoning import visible, link_pose_for_joint_config
from ..world_concepts.world_object import Object
from ..datastructures.world import World

class DefaultNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = World.robot
        robot.set_pose(desig.target)

class DefaultMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()

        pan_link = RobotDescription.current_robot_description.kinematic_chains["neck"].links[0]
        tilt_link = RobotDescription.current_robot_description.kinematic_chains["neck"].links[1]

        pan_joint = RobotDescription.current_robot_description.kinematic_chains["neck"].joints[0]
        tilt_joint = RobotDescription.current_robot_description.kinematic_chains["neck"].joints[1]
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame(pan_link))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame(tilt_link))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_position(pan_joint)
        current_tilt = robot.get_joint_position(tilt_joint)

        robot.set_joint_position(pan_joint, new_pan + current_pan)
        robot.set_joint_position(tilt_joint, new_tilt + current_tilt)


class DefaultMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion):
        robot = World.robot
        gripper = desig.gripper
        motion = desig.motion
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(gripper).get_static_gripper_state(motion).items():
            robot.set_joint_position(joint, state)


class DefaultDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion):
        robot = World.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = RobotDescription.current_robot_description.get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = RobotDescription.current_robot_description.get_default_camera().front_facing_axis

        objects = World.current_world.get_object_by_type(object_type)
        for obj in objects:
            if visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                return obj


class DefaultMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion):
        target = desig.target
        robot = World.robot

        _move_arm_tcp(target, robot, desig.arm)


class DefaultMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion):

        robot = World.robot
        if desig.right_arm_poses:
            for joint, pose in desig.right_arm_poses.items():
                robot.set_joint_position(joint, pose)
        if desig.left_arm_poses:
            for joint, pose in desig.left_arm_poses.items():
                robot.set_joint_position(joint, pose)


class DefaultMoveJoints(ProcessModule):
    def _execute(self, desig: MoveJointsMotion):
        robot = World.robot
        for joint, pose in zip(desig.names, desig.positions):
            robot.set_joint_position(joint, pose)


class DefaultWorldStateDetecting(ProcessModule):
    """
    This process moduledetectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, World.current_world.objects))[0]


class DefaultOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[1])


class DefaultClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """
    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv)


class DefaultManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("default")
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
            return DefaultNavigation(self._navigate_lock)

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
