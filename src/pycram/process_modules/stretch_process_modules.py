from typing import Any

import rospy

from ..external_interfaces.robokudo import query
from ..utils import _apply_ik
from ..external_interfaces import giskard
from .default_process_modules import *
from ..datastructures.world import World
from ..designators.motion_designator import *
from ..external_interfaces.ik import request_giskard_ik
from ..robot_description import RobotDescription


class StretchNavigate(DefaultNavigation):
    """
    Process module for the simulated Stretch that sends a cartesian goal to the robot to move the robot base
    """
    pass


class StretchMoveHead(ProcessModule):
    """
    Process module for the simulated Stretch that moves the head such that it looks at the given position
    """

    def _execute(self, designator: MoveMotion) -> Any:
        target = designator.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_pan"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        current_pan = robot.get_joint_position("joint_head_pan")

        robot.set_joint_position("joint_head_pan", new_pan + current_pan)

        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_tilt"))
        new_tilt = np.arctan2(-pose_in_tilt.position.y,
                              np.sqrt(pose_in_tilt.position.z ** 2 + pose_in_tilt.position.x ** 2)) * -1
        current_tilt = robot.get_joint_position("joint_head_tilt")

        robot.set_joint_position("joint_head_tilt", new_tilt + current_tilt)


class StretchMoveGripper(DefaultMoveGripper):
    """
    Process module for the simulated Stretch that opens or closes the gripper
    """
    pass


class StretchDetecting(DefaultDetecting):
    """
    Process Module for the simulated Stretch that tries to detect an object fitting the given object description
    """
    pass


class StretchMoveTCP(DefaultMoveTCP):
    """
    Process module for the simulated Stretch that moves the tool center point of the robot
    """
    pass


class StretchMoveArmJoints(DefaultMoveArmJoints):
    """
    Process module for the simulated Stretch that moves the arm joints of the robot
    """
    pass


class StretchMoveJoints(DefaultMoveJoints):
    """
    Process module for the simulated Stretch that moves any joint of the robot
    """
    pass


class StretchWorldStateDetecting(DefaultWorldStateDetecting):
    """
    Process Module for the simulated Stretch that tries to detect an object using the world state
    """
    pass


class StretchOpen(ProcessModule):
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


class StretchClose(ProcessModule):
    """
    Process module for the simulated Stretch that closes an already grasped container
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, World.robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                       part_of_object.get_joint_limits(
                                                           container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    # inv = request_ik(target, robot, joints, gripper)
    pose, joint_states = request_giskard_ik(target, robot, gripper)
    robot.set_pose(pose)
    robot.set_joint_positions(joint_states)


###########################################################
########## Process Modules for the Real Stretch ###########
###########################################################


class StretchNavigationReal(ProcessModule):
    """
    Process module for the real Stretch that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, RobotDescription.current_robot_description.base_link, "map")


class StretchMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)

        current_pan = robot.get_joint_position("joint_head_pan")
        current_tilt = robot.get_joint_position("joint_head_tilt")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_pan_joint": new_pan + current_pan})

        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))
        new_tilt = np.arctan2(-pose_in_tilt.position.y,
                              np.sqrt(pose_in_tilt.position.z ** 2 + pose_in_tilt.position.x ** 2)) * -1
        current_tilt = robot.get_joint_position("joint_head_tilt")
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_tilt_joint": new_tilt + current_tilt})


class StretchDetectingReal(ProcessModule):
    """
    Process Module for the real Stretch that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion) -> Any:
        query_result = query(ObjectDesignatorDescription(types=[designator.object_type]))
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


class StretchMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real Stretch while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")

        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
                                       RobotDescription.current_robot_description.base_link)


class StretchMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real Stretch to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class StretchMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class StretchMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real Stretch, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        chain = RobotDescription.current_robot_description.get_arm_chain(designator.gripper).get_static_gripper_state(designator.motion)
        giskard.achieve_joint_goal(chain)


class StretchOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
                                            designator.object_part.name)


class StretchCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
            giskard.achieve_close_container_goal(RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
                                             designator.object_part.name)


class StretchManager(ProcessModuleManager):
    def __init__(self):
        super().__init__("stretch_description")
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
        elif ProcessModuleManager.execution_type == "real":
            return StretchNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return StretchWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchCloseReal(self._close_lock)
