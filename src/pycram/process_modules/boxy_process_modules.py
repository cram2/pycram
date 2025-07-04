from .default_process_modules import *
from .. import world_reasoning as btr
from pycram.robot_plans.motions.motion_designator import *
from ..datastructures.enums import StaticJointState

from ..datastructures.world import World
from ..local_transformer import LocalTransformer
from ..process_module import ProcessModule, ProcessModuleManager
from ..robot_description import RobotDescription


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arm of Donbot and applies them to the
    in the World defined robot.
    :return: None
    """

    robot = World.robot
    if arm == "left":
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("left",
                                                                                             StaticJointState.Park).items():
            robot.set_joint_position(joint, pose)

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
            robot.set_multiple_joint_positions(
                RobotDescription.current_robot_description.get_static_joint_chain("neck", "front"))
        if pose_in_shoulder.position.y >= 0 and pose_in_shoulder.position.y >= abs(pose_in_shoulder.position.x):
            robot.set_multiple_joint_positions(
                RobotDescription.current_robot_description.get_static_joint_chain("neck", "neck_right"))
        if pose_in_shoulder.position.x <= 0 and abs(pose_in_shoulder.position.x) > abs(pose_in_shoulder.position.y):
            robot.set_multiple_joint_positions(
                RobotDescription.current_robot_description.get_static_joint_chain("neck", "back"))
        if pose_in_shoulder.position.y <= 0 and abs(pose_in_shoulder.position.y) > abs(pose_in_shoulder.position.x):
            robot.set_multiple_joint_positions(
                RobotDescription.current_robot_description.get_static_joint_chain("neck", "neck_left"))

        pose_in_shoulder = local_transformer.transform_pose(target, robot.get_link_tf_frame("neck_shoulder_link"))

        new_pan = np.arctan2(pose_in_shoulder.position.y, pose_in_shoulder.position.x)

        robot.set_joint_position("neck_shoulder_pan_joint",
                                 new_pan + robot.get_joint_position("neck_shoulder_pan_joint"))


class BoxyDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig):
        robot = World.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_link_name = RobotDescription.current_robot_description.get_camera_link()
        # should be [0, 0, 1]
        front_facing_axis = RobotDescription.current_robot_description.get_default_camera().front_facing_axis

        objects = World.current_world.get_object_by_type(object_type)
        for obj in objects:
            if btr.visible(obj, robot.get_link_pose(cam_link_name), front_facing_axis):
                return obj


class BoxyManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = "boxy"

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return BoxyMoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return BoxyDetecting(self._detecting_lock)

BoxyManager()
