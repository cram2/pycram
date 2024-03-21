from threading import Lock
from typing import Any

from ..external_interfaces.ik import request_ik
from ..helper import _apply_ik
from ..pose import Pose
from ..process_module import ProcessModule, ProcessModuleManager
from ..robot_descriptions import robot_description
from ..bullet_world import BulletWorld, Object
from ..bullet_world_reasoning import visible


class StretchNavigate(ProcessModule):
    def _execute(self, designator) -> Any:
        BulletWorld.robot.set_pose(designator.target)


class StretchMoveHead(ProcessModule):
    def _execute(self, designator) -> Any:
        pass


class StretchMoveGripper(ProcessModule):
    def _execute(self, designator) -> Any:
        robot = BulletWorld.robot
        gripper = designator.gripper
        motion = designator.motion
        for joint, state in robot_description.get_static_gripper_chain(gripper, motion).items():
            robot.set_joint_state(joint, state)


class StretchDetect(ProcessModule):
    def _execute(self, designator) -> Any:
        robot = BulletWorld.robot
        object_type = designator.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = robot_description.get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = robot_description.front_facing_axis

        objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)
        for obj in objects:
            if visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                return obj


class StretchMoveTCP(ProcessModule):
    def _execute(self, designator) -> Any:
        target = designator.target
        robot = BulletWorld.robot

        _move_arm_tcp(target, robot, designator.arm)


class StretchMoveArmJoints(ProcessModule):
    def _execute(self, designator) -> Any:
        robot = BulletWorld.robot
        if designator.right_arm_poses:
            robot.set_joint_states(designator.right_arm_poses)
        if designator.left_arm_poses:
            robot.set_joint_states(designator.left_arm_poses)


class StretchMoveJoints(ProcessModule):
    def _execute(self, designator) -> Any:
        robot = BulletWorld.robot
        robot.set_joint_states(dict(zip(designator.names, designator.positions)))


class StretchWorldStateDetecting(ProcessModule):
    def _execute(self, designator) -> Any:
        obj_type = designator.object_type
        return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class StretchOpen(ProcessModule):
    def _execute(self, designator) -> Any:
        pass


class StretchClose(ProcessModule):
    def _execute(self, designator) -> Any:
        pass


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
            return StretchNavigate()
