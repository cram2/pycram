from .default_process_modules import *
from ..utils import _apply_ik
from .default_process_modules import _move_arm_tcp


class DonbotMoveHead(DefaultMoveHead):
    """
    Moves the head of the iai_donbot robot to look at a specified point in the world coordinate frame.
    This point can be a position or an object, and the orientation is calculated based on the
    robot's base and camera alignment.
    """
    def _execute(self, desig):
        target = desig.target
        robot = World.robot

        base_frame_pose: Pose = robot.get_link_pose("ur5_base_link").copy()
        base_frame_pose.position.z += 0.4
        base_position = np.array(base_frame_pose.position_as_list())

        target_position = np.array(target.position_as_list())

        look_vector = target_position - base_position
        z_axis = look_vector / np.linalg.norm(look_vector)

        up = -np.array(RobotDescription.current_robot_description.cameras["camera_link"].front_facing_axis)

        x_axis = np.cross(up, z_axis)
        x_axis /= np.linalg.norm(x_axis)

        y_axis = np.cross(z_axis, x_axis)
        rotation_matrix = np.array([x_axis, y_axis, z_axis]).T

        orientation_quat = R.from_matrix(rotation_matrix).as_quat()
        adjusted_pose = Pose(base_position.tolist(), orientation_quat.tolist())
        _move_arm_tcp(adjusted_pose, robot, Arms.LEFT)

# TODO: Also need to do DonbotMoveHeadReal


class DonbotManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = "iai_donbot"

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DonbotMoveHead(self._looking_lock)
