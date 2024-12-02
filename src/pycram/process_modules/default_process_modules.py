from threading import Lock
from typing import List

import numpy as np
import rospy

from ..datastructures.dataclasses import Color
from ..datastructures.enums import JointType
from ..external_interfaces.ik import request_ik
from ..external_interfaces.robokudo import query_all_objects, query_object, query_human, query_specific_region, \
    query_human_attributes, query_waving_human, stop_query
from ..utils import _apply_ik
from ..process_module import ProcessModule
from ..robot_description import RobotDescription
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..world_reasoning import visible, link_pose_for_joint_config
from ..world_concepts.world_object import Object
from ..datastructures.world import World
from ..object_descriptors.generic import ObjectDescription as GenericObjectDescription
from pycrap import *
import inspect


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
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(gripper).get_static_gripper_state(
                motion).items():
            robot.set_joint_position(joint, state)


class DefaultDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    :return: A list of perceived objects.
    """

    def _execute(self, designator: DetectingMotion):
        robot = World.robot
        cam_frame_name = RobotDescription.current_robot_description.get_camera_link()
        camera_description = RobotDescription.current_robot_description.cameras[
            list(RobotDescription.current_robot_description.cameras.keys())[0]]
        front_facing_axis = camera_description.front_facing_axis
        query_result = []
        world_objects = []
        object_types = designator.object_designator_description.types
        if designator.technique == DetectionTechnique.TYPES:
            for tip in object_types:
                list1 = World.current_world.get_object_by_type(tip)
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
            if visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                query_result.append(obj)
        if query_result is None:
            raise PerceptionObjectNotFound(
                f"Could not find an object with the type {object_types} in the FOV of the robot")
        else:
            object_dict = {}

            for i, obj in enumerate(query_result):
                object_dict[obj.name] = obj
            return query_result


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


###########################################################
########## Process Modules for the Real     ###############
###########################################################

class DefaultDetectingReal(ProcessModule):
    def _execute(self, designator: DetectingMotion) -> List[Object]:
        """
            Perform a query based on the detection technique and state defined in the designator.

            :return: A list of perceived objects.
            """
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
                    obj_pose = Pose.from_pose_stamped(query_result.res[i].pose[0])
                except IndexError:
                    obj_pose = Pose.from_pose_stamped(query_result.res[i].pose)
                    pass
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

                # Map color names to RGBA values
                color_switch = {
                    "red": [1, 0, 0, 1],
                    "yellow": [1, 1, 0, 1],
                    "green": [0, 1, 0, 1],
                    "cyan": [0, 1, 1, 1],
                    "blue": [0, 0, 1, 1],
                    "magenta": [1, 0, 1, 1],
                    "white": [1, 1, 1, 1],
                    "black": [0, 0, 0, 1],
                    "grey": [0.5, 0.5, 0.5, 1],
                    # add more colors if needed
                }

                color = color_switch.get(obj_color)
                if color is None:
                    color = Color(0, 0, 0, 1)

                hsize = [obj_size.x / 2, obj_size.y / 2, obj_size.z / 2]

                # Check if the object type is a subclass of the classes in the objects module (pycrap)
                class_names = [name for name, obj in inspect.getmembers(objects, inspect.isclass)]

                matching_classes = [class_name for class_name in class_names if obj_type in class_name]

                obj_name = obj_type + "" + str(rospy.get_time())
                # Check if there are any matches
                if matching_classes:
                    rospy.loginfo(f"Matching class names: {matching_classes}")
                    obj_type = matching_classes[0]
                else:
                    rospy.loginfo(f"No class name contains the string '{obj_type}'")
                    obj_type = GenObj
                gen_obj_desc = GenericObjectDescription(obj_name, [0, 0, 0], hsize)
                generic_obj = Object(name=obj_name, concept=obj_type, path=None, description=gen_obj_desc, color=color)

                generic_obj.set_pose(obj_pose)

                perceived_objects.append(generic_obj)

            return perceived_objects


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
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultNavigation(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return DefaultDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveTCP(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveArmJoints(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveJoints(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultOpen(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultClose(self._close_lock)
