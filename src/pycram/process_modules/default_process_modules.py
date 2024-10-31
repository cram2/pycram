import numpy as np
import rospy
from typing_extensions import Any
from scipy.spatial.transform import Rotation as R

from ..datastructures.enums import JointType, Grasp
from ..external_interfaces.ik import request_ik
from ..external_interfaces.robokudo import query_object
from ..helper import adjust_grasp_for_object_rotation
from ..utils import _apply_ik
from ..process_module import ProcessModule
from ..external_interfaces import giskard
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
    Moves the robot's head to look at a specified target point in the world coordinate frame.
    The target can be either a position or an object.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot
        local_transformer = LocalTransformer()

        neck = RobotDescription.current_robot_description.get_neck()
        pan_link = neck["yaw"][0]
        tilt_link = neck["pitch"][0]

        pan_joint = neck["yaw"][1]
        tilt_joint = neck["pitch"][1]

        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame(pan_link)).position_as_list()
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame(tilt_link)).position_as_list()

        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])

        tilt_offset = RobotDescription.current_robot_description.get_offset(tilt_joint)
        tilt_offset_rotation = tilt_offset.rotation if tilt_offset else [0, 0, 0]
        rotation_tilt_offset = R.from_euler('xyz', tilt_offset_rotation).apply(pose_in_tilt)

        new_tilt = -np.arctan2(rotation_tilt_offset[2],
                               np.sqrt(rotation_tilt_offset[0] ** 2 + rotation_tilt_offset[1] ** 2))

        # @TODO: find a better way to handle this
        if RobotDescription.current_robot_description.name in {"iCub", "tiago_dual"}:
            new_tilt = -new_tilt

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
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(gripper).get_static_gripper_state(
                motion).items():
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
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def _execute(self, desig: MoveJointsMotion):
        robot = World.robot
        torso_joint = RobotDescription.current_robot_description.get_torso_joint()

        if [torso_joint] == desig.names:
            joint_poses: dict = RobotDescription.current_robot_description.get_static_joint_chain("torso",
                                                                                                  desig.positions[
                                                                                                      0])
        else:
            joint_poses: dict = dict(zip(desig.names, desig.positions))

        robot.set_joint_positions(joint_poses)
        return ",", ","


class DefaultWorldStateDetecting(ProcessModule):
    """
    This process moduledetectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, World.current_world.objects))[0]


class DefaultOpen(ProcessModule):
    """
    Low-level implementation for opening a container in the simulation. Assumes that
    the handle has already been grasped by the robot.
    """

    def _execute(self, desig: OpeningMotion):
        # TODO: Should probably be a motion, but DefaultOpenReal does not do any of these calculations, so maybe its fine? Need to think about this
        part_of_object = desig.object_part.world_object

        if desig.object_part.name == "handle_cab3_door_top":
            container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.REVOLUTE)
        else:
            container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        grasp = Grasp.FRONT
        adjusted_grasp = adjust_grasp_for_object_rotation(goal_pose, grasp, desig.arm)
        goal_pose = goal_pose.copy()
        goal_pose.set_orientation(adjusted_grasp)

        if desig.goal_location:
            _navigate_to_pose(desig.goal_location.pose, World.robot)
        else:
            _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[1])


class DefaultClose(ProcessModule):
    """
    Low-level implementation for closing a grasped container in simulation.
    Assumes the robot is already holding the container's handle.
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        if desig.object_part.name == "handle_cab3_door_top":
            container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.REVOLUTE)
        else:
            container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        grasp = Grasp.FRONT
        adjusted_grasp = adjust_grasp_for_object_rotation(goal_pose, grasp, desig.arm)
        goal_pose = goal_pose.copy()
        goal_pose.set_orientation(adjusted_grasp)

        if desig.goal_location:
            _navigate_to_pose(desig.goal_location.pose, World.robot)
        else:
            _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv)


def _navigate_to_pose(target: Pose, robot: Object) -> None:
    robot.set_pose(target)


###########################################################
######### Process Modules for the Real Robot ##############
###########################################################

class DefaultNavigationReal(ProcessModule):
    """
    Process module for the real robot that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, RobotDescription.current_robot_description.base_link, "map")


class DefaultMoveHeadReal(ProcessModule):
    """
    Process module for controlling the real robot's head to look at a specified position.
    Uses the same calculations as the simulated version to orient the head.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()

        neck = RobotDescription.current_robot_description.get_neck()
        pan_link = neck["yaw"][0]
        tilt_link = neck["pitch"][0]

        pan_joint = neck["yaw"][1]
        tilt_joint = neck["pitch"][1]

        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame(pan_link)).position_as_list()
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame(tilt_link)).position_as_list()

        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])

        tilt_offset = RobotDescription.current_robot_description.get_offset(tilt_joint)
        tilt_offset_rotation = tilt_offset.rotation if tilt_offset else [0, 0, 0]
        rotation_tilt_offset = R.from_euler('xyz', tilt_offset_rotation).apply(pose_in_tilt)

        new_tilt = -np.arctan2(rotation_tilt_offset[2],
                               np.sqrt(rotation_tilt_offset[0] ** 2 + rotation_tilt_offset[1] ** 2))

        # @TODO: find a better way to handle this
        if RobotDescription.current_robot_description.name in {"iCub", "tiago_dual"}:
            new_tilt = -new_tilt

        current_pan = robot.get_joint_position(pan_joint)
        current_tilt = robot.get_joint_position(tilt_joint)

        # TODO: Not sure that this is enough to move head in real world
        robot.set_joint_position(pan_joint, new_pan + current_pan)
        robot.set_joint_position(tilt_joint, new_tilt + current_tilt)


class DefaultDetectingReal(ProcessModule):
    """
    Process Module for the real robot that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion) -> Any:
        query_result = query_object(ObjectDesignatorDescription(types=[designator.object_type]))
        # print(query_result)
        obj_pose = query_result["ClusterPoseBBAnnotator"]

        lt = LocalTransformer()
        obj_pose = lt.transform_pose(obj_pose, World.robot.get_link_tf_frame("torso_lift_link"))
        obj_pose.orientation = [0, 0, 0, 1]
        obj_pose.position.x += 0.05

        bullet_obj = World.current_world.get_objects_by_type(designator.object_type)
        if bullet_obj:
            bullet_obj[0].set_pose(obj_pose)
            return bullet_obj[0]
        elif designator.object_type == ObjectType.JEROEN_CUP:
            cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=obj_pose)
            return cup
        elif designator.object_type == ObjectType.BOWL:
            bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=obj_pose)
            return bowl

        return bullet_obj[0]


class DefaultMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real robot while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")

        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm.name)
        giskard.achieve_cartesian_goal(pose_in_map,
                                       RobotDescription.current_robot_description.get_arm_chain(
                                           designator.arm).get_tool_frame(),
                                       RobotDescription.current_robot_description.base_link)


class DefaultMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real robot to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class DefaultMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class DefaultMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real robot, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        raise NotImplementedError(f"There is DefaultMoveGripperReal process module")


class DefaultOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class DefaultCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class DefaultManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("default")

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return DefaultCloseReal(self._close_lock)
