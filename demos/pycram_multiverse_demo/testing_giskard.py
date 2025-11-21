import os

import actionlib
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
from giskard_msgs.msg import WorldResult
from typing_extensions import Optional

from giskardpy.python_interface.old_python_interface import OldGiskardWrapper as GiskardWrapper

from geometry_msgs.msg import PoseStamped, Point, Quaternion

from pycram.helper import find_multiverse_resources_path
import pycram.ros  # this to start the ros node
from pycram.logging import info


def spawn_urdf(name: str, urdf_path: str, pose: PoseStamped) -> WorldResult:
    """
    Spawns an URDF in giskard's belief state.

    :param name: Name of the URDF
    :param urdf_path: Path to the URDF file
    :param pose: Pose in which the URDF should be spawned
    :return: A WorldResult message
    """
    with open(urdf_path) as f:
        urdf_string = f.read()

    return giskard.add_urdf(name, urdf_string, pose)


def get_pose_stamped(position, orientation):
    pose_stamped = PoseStamped()
    pose_stamped.header.reference_frame = 'map'
    pose_stamped.pose.position = Point(**dict(zip(["x", "y", "z"], position)))
    pose_stamped.pose.orientation = Quaternion(**dict(zip(["x", "y", "z", "w"],orientation)))
    return pose_stamped


def park_arms():
    joint_goals = {'l_shoulder_pan_joint': 1.712, 'l_shoulder_lift_joint': -0.264, 'l_upper_arm_roll_joint': 1.38,
                   'l_elbow_flex_joint': -2.12, 'l_forearm_roll_joint': 16.996, 'l_wrist_flex_joint': -0.073,
                   'l_wrist_roll_joint': 0.0, 'r_shoulder_pan_joint': -1.712, 'r_shoulder_lift_joint': -0.256,
                   'r_upper_arm_roll_joint': -1.463, 'r_elbow_flex_joint': -2.12, 'r_forearm_roll_joint': 1.766,
                   'r_wrist_flex_joint': -0.07, 'r_wrist_roll_joint': 0.051}
    giskard.set_joint_goal(joint_goals)
    giskard.execute()


def get_urdf_string(urdf_path):
    with open(urdf_path) as f:
        return f.read()


def open_left_gripper():
    open_gripper("left")


def close_left_gripper():
    close_gripper("left")


def open_right_gripper():
    open_gripper("right")


def close_right_gripper():
    close_gripper("right")


def open_gripper(gripper: str):
    move_gripper("open", gripper)


def close_gripper(gripper: str):
    move_gripper("close", gripper)


def move_gripper(cmd: str, gripper: str):
    def activate_callback():
        info("Started gripper Movement")

    def done_callback(state, result):
        info(f"Reached goal: {result.reached_goal}")

    def feedback_callback(msg):
        pass

    goal = GripperCommandGoal()
    goal.command.position = 0.0 if cmd == "close" else 0.4
    goal.command.max_effort = 50.0
    if gripper == "right":
        controller_topic = "/real/pr2/right_gripper_controller/gripper_cmd"
    else:
        controller_topic = "/real/pr2/left_gripper_controller/gripper_cmd"
    client = actionlib.SimpleActionClient(controller_topic, GripperCommandAction)
    info("Waiting for action server")
    client.wait_for_server()
    client.send_goal(goal, active_cb=activate_callback, done_cb=done_callback, feedback_cb=feedback_callback)
    wait = client.wait_for_result()


def move_base(pose_stamped: PoseStamped):
    giskard.set_cart_goal(pose_stamped, 'base_link', 'map', add_monitor=False)
    giskard.execute()


def move_left_arm_tool(pose_stamped: PoseStamped):
    move_arm_tool(pose_stamped, 'left')


def move_right_arm_tool(pose_stamped: PoseStamped):
    move_arm_tool(pose_stamped, 'right')


def move_arm_tool(pose_stamped: PoseStamped, arm: str):
    tool_frame = 'l_gripper_tool_frame' if arm == 'left' else 'r_gripper_tool_frame'
    giskard.set_cart_goal(pose_stamped, tool_frame, 'torso_lift_link', add_monitor=False)
    giskard.execute()


if __name__ == '__main__':

    multiverse_resources = find_multiverse_resources_path()
    cached_dir = multiverse_resources + 'cached/'

    giskard = GiskardWrapper()

    giskard.add_urdf('apartment',
                     get_urdf_string(cached_dir + 'apartment.urdf'),
                     get_pose_stamped([0, 0, 0], [0, 0, 0, 1]))

    park_arms()

    move_base(get_pose_stamped([1.17, 2.655, 0], [0.0, 0.0, -0.992, 0.123]))

    open_left_gripper()

    move_left_arm_tool(get_pose_stamped([0.47, 2.48, 1.04], [0, 0, 0, 1]))

    close_left_gripper()

    move_left_arm_tool(get_pose_stamped([0.5, 2.48, 1.04], [0, 0, 0, 1]))

    park_arms()

    move_base(get_pose_stamped([1.4, 3.5, 0], [0.0, 0.0, 0, 1]))
