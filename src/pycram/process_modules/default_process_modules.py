import inspect
import time

import numpy as np
from typing_extensions import List, TYPE_CHECKING
from scipy.spatial.transform import Rotation as R

from pycrap.urdf_parser import parse_furniture
from ..datastructures.dataclasses import Colors
from ..datastructures.world import World
from ..designators.motion_designator import *
from ..external_interfaces import giskard
from ..external_interfaces.ik import request_ik
from ..external_interfaces.move_base import query_pose_nav
from ..external_interfaces.robokudo import query_all_objects, query_object, query_human, query_specific_region, \
    query_human_attributes, query_waving_human
from ..failures import NavigationGoalNotReachedError
from ..local_transformer import LocalTransformer
from ..object_descriptors.generic import ObjectDescription as GenericObjectDescription
from ..process_module import ProcessModule
from ..robot_description import RobotDescription
from ..ros import get_time
from ..ros import logdebug, loginfo
from ..tf_transformations import euler_from_quaternion
from ..utils import _apply_ik
from ..world_concepts.world_object import Object
from ..world_reasoning import visible, link_pose_for_joint_config

if TYPE_CHECKING:
    from ..designators.object_designator import ObjectDesignatorDescription


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

        pose_in_map = local_transformer.transform_pose(target, "map")

        pose_in_pan = local_transformer.transform_pose(pose_in_map,
                                                       robot.get_link_tf_frame(pan_link)).position.to_list()
        pose_in_tilt = local_transformer.transform_pose(pose_in_map,
                                                        robot.get_link_tf_frame(tilt_link)).position.to_list()

        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])

        tilt_offset = RobotDescription.current_robot_description.get_offset(tilt_joint)
        if tilt_offset:
            tilt_offset_rotation = tilt_offset.orientation
            quaternion_list = [tilt_offset_rotation.x, tilt_offset_rotation.y, tilt_offset_rotation.z,
                               tilt_offset_rotation.w]
        else:
            quaternion_list = [0, 0, 0, 1]

        tilt_offset_rotation = euler_from_quaternion(quaternion_list, axes='sxyz')
        adjusted_pose_in_tilt = R.from_euler('xyz', tilt_offset_rotation).apply(pose_in_tilt)

        new_tilt = -np.arctan2(adjusted_pose_in_tilt[2],
                               np.sqrt(adjusted_pose_in_tilt[0] ** 2 + adjusted_pose_in_tilt[1] ** 2))

        if RobotDescription.current_robot_description.name in {"iCub", "tiago_dual"}:
            new_tilt = -new_tilt

        current_pan = robot.get_joint_position(pan_joint)
        current_tilt = robot.get_joint_position(tilt_joint)
        robot.set_joint_position(pan_joint, new_pan + current_pan)
        robot.set_joint_position(tilt_joint, new_tilt + current_tilt)


class DefaultMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only move one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion):
        robot_description = RobotDescription.current_robot_description
        gripper = desig.gripper
        arm_chain = robot_description.get_arm_chain(gripper)
        if arm_chain.end_effector.gripper_object_name is not None:
            robot = World.current_world.get_object_by_name(arm_chain.end_effector.gripper_object_name)
        else:
            robot = World.robot
        motion = desig.motion
        robot.set_multiple_joint_positions(arm_chain.get_static_gripper_state(motion))


class DefaultDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    :return: A list of perceived objects.
    """

    def _execute(self, designator: DetectingMotion):
        robot = World.robot
        cam_link_name = RobotDescription.current_robot_description.get_camera_link()
        camera_description = RobotDescription.current_robot_description.cameras[
            list(RobotDescription.current_robot_description.cameras.keys())[0]]
        front_facing_axis = camera_description.front_facing_axis
        query_result = []
        world_objects = []

        if designator.technique == DetectionTechnique.TYPES:
            try:
                object_types = designator.object_designator_description.obj_type
            except AttributeError:
                raise AttributeError("The object designator does not contain a type attribute")

            list1 = World.current_world.get_object_by_type(object_types)
            world_objects = world_objects + list1
        elif designator.technique == DetectionTechnique.ALL:
            world_objects = World.current_world.get_scene_objects()
        elif designator.technique == DetectionTechnique.HUMAN:
            raise NotImplementedError("Detection by human is not yet implemented in simulation")
        elif designator.technique == DetectionTechnique.REGION:
            raise NotImplementedError("Detection by region is not yet implemented in simulation")
        elif designator.technique == DetectionTechnique.HUMAN_ATTRIBUTES:
            raise NotImplementedError("Detection by human attributes is not yet implemented in simulation")
        elif designator.technique == DetectionTechnique.HUMAN_WAVING:
            raise NotImplementedError("Detection by waving human is not yet implemented in simulation")
        for obj in world_objects:
            if visible(obj, robot.get_link_pose(cam_link_name), front_facing_axis):
                query_result.append(obj)
        if query_result is None:
            raise PerceptionObjectNotFound(
                f"Could not find an object with the type {object_types} in the FOV of the robot")
        else:
            object_dict = []

            for obj in query_result:
                object_dict.append(obj)

            return object_dict


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
        return list(filter(lambda obj: obj.obj_type == obj_type, World.current_world.objects))[0]


class DefaultOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.parent_entity

        container_joint_name = part_of_object.find_joint_above_link(desig.object_part.name)
        lower_limit, upper_limit = part_of_object.get_joint_limits(container_joint_name)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint_name: max(lower_limit, upper_limit - 0.05)}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        part_of_object.set_joint_position(container_joint_name, upper_limit)


class DefaultClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.parent_entity

        container_joint_name = part_of_object.find_joint_above_link(desig.object_part.name)
        lower_joint_limit = part_of_object.get_joint_limits(container_joint_name)[0]

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint_name: lower_joint_limit}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        part_of_object.set_joint_position(container_joint_name, lower_joint_limit)


class DefaultMoveTCPWaypoints(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm along a list of waypoints.
    """

    def _execute(self, desig: MoveTCPWaypointsMotion):
        waypoints = desig.waypoints
        robot = World.robot
        for waypoint in waypoints:
            _move_arm_tcp(waypoint, robot, desig.arm)


def _move_arm_tcp(target: PoseStamped, robot: Object, arm: Arms, tip_link: str = None) -> None:
    """
    Calls the ik solver to calculate the inverse kinematics of the arm and then sets the joint states accordingly.

    :param target: Target pose to which the end-effector should move.
    :param robot: Robot object representing the robot.
    :param arm: Which arm to move
    """
    if tip_link is None:
        tip_link = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

    inv = request_ik(target, robot, joints, tip_link)
    _apply_ik(robot, inv)


###########################################################
########## Process Modules for the Real     ###############
###########################################################

class DefaultDetectingReal(ProcessModule):
    def _execute(self, designator: DetectingMotion) -> List[Object]:
        """
            Perform a query based on the detection technique and state defined in the designator.

            :return: A list of perceived objects.
            """
        print(designator.technique)
        object_designator_description = designator.object_designator_description
        query_methods = {
            DetectionTechnique.TYPES: lambda: query_object(object_designator_description),
            DetectionTechnique.HUMAN: lambda: query_human(),
            DetectionTechnique.HUMAN_ATTRIBUTES: query_human_attributes,
            DetectionTechnique.HUMAN_WAVING: query_waving_human,
            DetectionTechnique.REGION: lambda: query_specific_region(designator.region)
        }  # Fetch the appropriate query function
        query_func = query_methods.get(designator.technique, query_all_objects)
        query_result = query_func() if callable(query_func) else query_func
        # Handle the case where no result is found
        if query_result is None:
            raise PerceptionObjectNotFound(
                f"Could not find an object in the FOV of the robot")
        else:
            perceived_objects = []
            for i in range(0, len(query_result.res)):
                try:
                    obj_pose = PoseStamped.from_ros_message(query_result.res[i].pose[0])
                except IndexError:
                    obj_pose = PoseStamped.from_ros_message(query_result.res[i].pose)
                    pass
                lt = LocalTransformer()
                obj_pose.frame_id = World.robot.get_link_tf_frame(obj_pose.frame_id)
                obj_pose_T_m = lt.transform_pose(obj_pose, "map")

                obj_type = query_result.res[i].type
                obj_size = None
                try:
                    obj_size = query_result.res[i].shape_size[0].dimensions
                except IndexError:
                    pass

                obj_color = None
                try:
                    obj_color = query_result.res[i].color[0]
                except IndexError:
                    pass

                if obj_size is None:
                    hsize = [0.2, 0.2, 0.2]

                else:
                    hsize = [obj_size.x / 2, obj_size.y / 2, obj_size.z / 2]

                # Check if the object type is a subclass of the classes in the objects module (pycrap)
                type_concept = parse_furniture(obj_type)
                if type_concept is None:
                    loginfo(f"No class name contains the string '{obj_type}'")
                    type_concept = PhysicalObject

                obj_name = obj_type + "" + str(get_time())

                gen_obj_desc = GenericObjectDescription(obj_name, [0, 0, 0], hsize)

                if obj_color is not None: color = Colors.from_string(obj_color)
                else: color = Colors.PINK

                generic_obj = Object(name=obj_name, concept=type_concept, path=None, description=gen_obj_desc,
                                     color=color)


                generic_obj.set_pose(obj_pose)


                perceived_objects.append(generic_obj)

            object_dict = []

            for obj in perceived_objects:
                object_dict.append(ObjectDesignatorDescription.Object(obj.name, obj.obj_type,
                                                                      obj))

            return object_dict


class DefaultNavigationReal(ProcessModule):
    """
    Process module for the real robot that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion):
        # logdebug(f"Sending goal to movebase to Move the robot")
        # query_pose_nav(designator.target)
        logdebug(f"Sending goal to giskard to Move the robot")
        giskard.avoid_all_collisions()
        giskard.achieve_cartesian_goal(designator.target,
                                       RobotDescription.current_robot_description.base_link,
                                       "map")

        if not World.current_world.robot.pose.almost_equal(designator.target, 0.05, 3):
            raise NavigationGoalNotReachedError(World.current_world.robot.pose, designator.target)


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

        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame(pan_link)).position.to_list()
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame(tilt_link)).position.to_list()

        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])

        tilt_offset = RobotDescription.current_robot_description.get_offset(tilt_joint)
        if tilt_offset:
            tilt_offset_rotation = tilt_offset.pose.orientation
            quaternion_list = [tilt_offset_rotation.x, tilt_offset_rotation.y, tilt_offset_rotation.z,
                               tilt_offset_rotation.w]
        else:
            quaternion_list = [0, 0, 0, 1]

        tilt_offset_rotation = euler_from_quaternion(quaternion_list, axes='sxyz')
        adjusted_pose_in_tilt = R.from_euler('xyz', tilt_offset_rotation).apply(pose_in_tilt)

        new_tilt = -np.arctan2(adjusted_pose_in_tilt[2],
                               np.sqrt(adjusted_pose_in_tilt[0] ** 2 + adjusted_pose_in_tilt[1] ** 2))

        if RobotDescription.current_robot_description.name in {"iCub", "tiago_dual"}:
            new_tilt = -new_tilt

        current_pan = robot.get_joint_position(pan_joint)
        current_tilt = robot.get_joint_position(tilt_joint)

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({pan_joint: new_pan + current_pan,
                                    tilt_joint: new_tilt + current_tilt})


class DefaultMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real robot while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion):
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
        tip_link = RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame()
        root_link = "base_link"

        gripper_that_can_collide = designator.arm if designator.allow_gripper_collision else None
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)

        if designator.movement_type == MovementType.STRAIGHT_TRANSLATION:
            giskard.achieve_straight_translation_goal(pose_in_map.position.to_list(), tip_link, root_link)
        elif designator.movement_type == MovementType.STRAIGHT_CARTESIAN:
            giskard.achieve_straight_cartesian_goal(pose_in_map, tip_link, root_link)
        elif designator.movement_type == MovementType.TRANSLATION:
            giskard.achieve_translation_goal(pose_in_map.position.to_list(), tip_link, root_link)
        elif designator.movement_type == MovementType.CARTESIAN:
            giskard.achieve_cartesian_goal(pose_in_map, tip_link, root_link,
                                           grippers_that_can_collide=gripper_that_can_collide)
        time.sleep(2)
        if not World.current_world.robot.get_link_pose(tip_link).almost_equal(designator.target, 0.3, 3):
            raise ToolPoseNotReachedError(World.current_world.robot.get_link_pose(tip_link), designator.target)


class DefaultMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real robot to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion):
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

    def _execute(self, designator: MoveJointsMotion):
        name_to_position = dict(zip(designator.names, designator.positions))
        align = designator.align
        tip_link = designator.tip_link
        tip_normal = designator.tip_normal
        root_normal = designator.root_normal
        root_link = designator.root_link
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position, align=align, tip_link=tip_link, tip_normal=tip_normal,
                                   root_link=root_link, root_normal=root_normal)


class DefaultMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real robot, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion):
        raise NotImplementedError(f"There is DefaultMoveGripperReal process module")


class DefaultOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion):
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class DefaultCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion):
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class DefaultMoveTCPWaypointsReal(ProcessModule):
    """
    Moves the tool center point of the real robot along a list of waypoints while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPWaypointsMotion):
        lt = LocalTransformer()
        waypoints = [lt.transform_pose(x, "map") for x in designator.waypoints]
        tip_link = RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame()
        root_link = "map"

        giskard.avoid_all_collisions()
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)

        giskard.achieve_cartesian_waypoints_goal(waypoints=waypoints,
                                                 tip_link=tip_link, root_link=root_link,
                                                 enforce_final_orientation=True if designator.movement_type == WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT else False)


class DefaultManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("default")

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if (ProcessModuleManager.execution_type == ExecutionType.SIMULATED or
                ProcessModuleManager.execution_type == ExecutionType.REAL):
            return DefaultWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultClose(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultCloseReal(self._close_lock)

    def move_tcp_waypoints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCPWaypoints(self._move_tcp_waypoints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultMoveTCPWaypointsReal(self._move_tcp_waypoints_lock)

# Initialize the default manager and register it with the ProcessModuleManager
DefaultManager()