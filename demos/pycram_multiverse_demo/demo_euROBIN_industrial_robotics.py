import pycrap
from pycram.datastructures.enums import GripperState
from pycram.process_module import real_robot
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import PoseStamped
from pycram.worlds.multiverse import Multiverse
from pycram.robot_plans import *
from pycram.ros_utils.robot_state_updater import WorldStateUpdater


if __name__ == '__main__':
    # Create a new world
    world = Multiverse()
    WorldStateUpdater(tf_topic="/tf", joint_state_topic="/real/ur5e/joint_states")

    # Load the robot and the gripper
    robot = Object("ur5e", pycrap.Robot, "universal_robot/ur5e/urdf/ur5e.urdf")
    gripper = Object("gripper-2F-85", pycrap.Gripper, "robotiq/gripper-2F-85/gripper-2F-85.urdf")

    # Attach the gripper to the robot at the wrist_3_link with the correct pose
    wrist_3_tf_frame = robot.get_link_tf_frame("wrist_3_link")
    gripper.set_pose(PoseStamped.from_list([0, 0.1, 0], [1.0, 0.0, 0.0, -1.0], frame_id=wrist_3_tf_frame))
    robot.attach(gripper, parent_link="wrist_3_link")

    # Get the robot arms
    robot_arms = [chain.arm_type for chain in robot.robot_description.get_manipulator_chains()]

    # Perform the plan
    with real_robot:
        SetGripperAction(robot_arms, [GripperState.CLOSE]).resolve().perform()

    world.exit()
