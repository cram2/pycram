from .default_process_modules import *
from ..worlds.bullet_world import World
from ..process_module import ProcessModuleManager
from ..robot_description import RobotDescription
from ..datastructures.enums import Arms, ExecutionType, Grasp, StaticJointState
from .default_process_modules import _move_arm_tcp


class DonbotMoveHead(DefaultMoveHead):
    """
    Moves the head of the iai_donbot robot to look at a specified point in the world coordinate frame.
    This point can be a position or an object, and the orientation is calculated based on the
    robot's base and camera alignment.
    """

    def _execute(self, desig):
        target = desig.target.copy()
        robot = World.robot

        # Rotate the arm to have a nice starting seed for the IK. If we dont do this, we regularly run into
        # self-collision in the simulation. Another solution would be to look into (possibly collision gradient
        # based) collision avoidance for pinocchio.
        perceive_state = RobotDescription.current_robot_description.get_static_joint_chain("left_arm",
                                                                                           "looking")
        robot.set_multiple_joint_positions(perceive_state)
        pose_in_pan = LocalTransformer().transform_pose(target,
                                                        robot.get_link_tf_frame("ur5_shoulder_link")).position_as_list()
        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])
        current_pan = robot.get_joint_position("ur5_shoulder_pan_joint")
        robot.set_joint_position("ur5_shoulder_pan_joint", new_pan + current_pan)

        # Compute the position the camera should take when looking. Due to the geometry of the arm and the placements of
        # the links, we translate the shoulder link pose by 20cm in -y direction, since this is roughly aligned with the
        # gripper when donbot is in the parking pose. The z-position is currently hardcoded to a moderate height, a bit
        # higher and lower should still work just as well. Could in theory be dynamically set depending on the required
        # camera height to see an object, similar to setting up a torso to be able to see a certain object.
        base_frame_pose: Pose = robot.get_link_pose("ur5_shoulder_link").copy()
        base_frame_pose = base_frame_pose.translate_along_axis([0, -1, 0], 0.2)
        base_frame_pose.position.z += 0.6
        base_position = np.array(base_frame_pose.position_as_list())

        # Define axis directly pointing toward the target
        target_position = np.array(target.position_as_list())
        direction_vector = target_position - base_position
        direction_vector /= np.linalg.norm(direction_vector)

        # Define that z-axis denotes the world's "up" direction
        world_up_z = np.array([0, 0, 1])

        # Define y-axis perpendicular to X and Z, according to right-hand rule
        right_hand_y = np.cross(world_up_z, direction_vector)
        right_hand_y /= np.linalg.norm(right_hand_y)

        # Recompute Z-axis to disallow roll (since we want the camera to be somewhat level if possible)
        corrected_up_z = np.cross(direction_vector, right_hand_y)

        # Construct rotation matrix
        rotation_matrix = np.column_stack((direction_vector, right_hand_y, corrected_up_z))

        # Convert to quaternion, and construct pose
        orientation_quat = R.from_matrix(rotation_matrix).as_quat()
        adjusted_pose = Pose(base_position.tolist(), orientation_quat.tolist())

        # Get front grasp of gripper, apply it to the pose, and then roll 180°. This works for donbot, because
        # the camera has the same frame orientation, albeit, rotated by 180° around z (which is the "forward" axis of the frame)
        side_grasp, top_grasp, horizontal = (Grasp.FRONT, None, False)
        grasp_orientation = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).end_effector.get_grasp(
            side_grasp, top_grasp, horizontal)
        adjusted_pose.rotate_by_quaternion(grasp_orientation)
        adjusted_pose.rotate_by_quaternion([0, 0, 1, 0])

        # move the camera link to the computed pose
        camera_link = RobotDescription.current_robot_description.get_camera_link()
        _move_arm_tcp(adjusted_pose, robot, Arms.LEFT, camera_link)


class DonbotManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = "iai_donbot"

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DonbotMoveHead(self._looking_lock)
