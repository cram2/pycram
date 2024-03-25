from threading import Lock

import numpy as np

from pycram.world import World
from ..designators.motion_designator import PlaceMotion
from pycram.datastructures.local_transformer import LocalTransformer
from ..process_module import ProcessModule, ProcessModuleManager
from ..robot_descriptions import robot_description
from ..process_modules.pr2_process_modules import (_park_arms, Pr2Navigation as BoxyNavigation,
                                                   Pr2PickUp as BoxyPickUp, Pr2Detecting as BoxyDetecting,
                                                   Pr2MoveTCP as BoxyMoveTCP, Pr2MoveArmJoints as BoxyMoveArmJoints,
                                                   Pr2WorldStateDetecting as BoxyWorldStateDetecting, _move_arm_tcp)


class BoxyPlace(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig: PlaceMotion.Motion):
        """

        :param desig: A PlaceMotion
        :return:
        """
        obj = desig.object.world_object
        robot = World.robot
        arm = desig.arm

        # Transformations such that the target position is the position of the object and not the tcp
        object_pose = obj.get_pose()
        local_tf = LocalTransformer()
        tcp_to_object = local_tf.transform_pose(object_pose,
                                                robot.get_link_tf_frame(robot_description.get_tool_frame(arm)))
        target_diff = desig.target.to_transform("target").inverse_times(tcp_to_object.to_transform("object")).to_pose()

        _move_arm_tcp(target_diff, robot, arm)
        robot.detach(obj)


class BoxyParkArms(ProcessModule):
    """
    This process module is for moving the arms in a parking position.
    It is currently not used.
    """

    def _execute(self, desig):
        solutions = desig.reference()
        if solutions['cmd'] == 'park':
            _park_arms()


class BoxyMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig):
        target = desig.target
        robot = World.robot

        local_transformer = LocalTransformer()

        pose_in_shoulder = local_transformer.transform_pose(target, robot.get_link_tf_frame("neck_shoulder_link"))

        if pose_in_shoulder.position.x >= 0 and pose_in_shoulder.position.x >= abs(pose_in_shoulder.position.y):
            robot.set_joint_positions(robot_description.get_static_joint_chain("neck", "front"))
        if pose_in_shoulder.position.y >= 0 and pose_in_shoulder.position.y >= abs(pose_in_shoulder.position.x):
            robot.set_joint_positions(robot_description.get_static_joint_chain("neck", "neck_right"))
        if pose_in_shoulder.position.x <= 0 and abs(pose_in_shoulder.position.x) > abs(pose_in_shoulder.position.y):
            robot.set_joint_positions(robot_description.get_static_joint_chain("neck", "back"))
        if pose_in_shoulder.position.y <= 0 and abs(pose_in_shoulder.position.y) > abs(pose_in_shoulder.position.x):
            robot.set_joint_positions(robot_description.get_static_joint_chain("neck", "neck_left"))

        pose_in_shoulder = local_transformer.transform_pose(target, robot.get_link_tf_frame("neck_shoulder_link"))

        new_pan = np.arctan2(pose_in_shoulder.position.y, pose_in_shoulder.position.x)

        robot.set_joint_position("neck_shoulder_pan_joint",
                                 new_pan + robot.get_joint_position("neck_shoulder_pan_joint"))


class BoxyMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only move one gripper at a time.
    """

    def _execute(self, desig):
        robot = World.robot
        gripper = desig.gripper
        motion = desig.motion
        robot.set_joint_positions(robot_description.get_static_gripper_chain(gripper, motion))


class BoxyManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("boxy")
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
            return BoxyNavigation(self._navigate_lock)

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return BoxyPickUp(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return BoxyPlace(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return BoxyMoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return BoxyDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return BoxyMoveTCP(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return BoxyMoveArmJoints(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return BoxyWorldStateDetecting(self._world_state_detecting_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return BoxyMoveGripper(self._move_gripper_lock)
