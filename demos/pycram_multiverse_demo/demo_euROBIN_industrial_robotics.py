import logging
import os

from rospkg import RosPack

from pycram.datastructures.enums import ObjectType, GripperState, Arms
from pycram.process_module import simulated_robot, real_robot
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import Pose
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.ros_utils.joint_state_publisher import JointStatePublisher
from pycram.worlds.multiverse import Multiverse
from pycram.designators.action_designator import SetGripperAction

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PYCRAM_DIR = os.path.join(SCRIPT_DIR, os.pardir, os.pardir)
RESOURCE_DIR = os.path.join(PYCRAM_DIR, "resources")

SPAWNING_POSES = {
    "robot": Pose([0, 0, 0], [0.0, 0.0, 0.0, 1.0]),   # x,y,z,qx,qy,qz,qw
    "cereal": Pose([0.5, 0.5, 2.0], [0.0, 0.0, 0.0, 1.0])
}


def spawn_ur5e_with_gripper():
    robot = Object("ur5e", ObjectType.ROBOT, "universal_robot/ur5e/urdf/ur5e.urdf")
    gripper = Object("gripper-2F-85", ObjectType.GRIPPER, "robotiq/gripper-2F-85/gripper-2F-85.urdf")
    wrist_3_tf_frame = robot.get_link_tf_frame("wrist_3_link")
    gripper.set_pose(Pose([0, 0.1, 0], [1.0, 0.0, 0.0, -1.0], frame=wrist_3_tf_frame))
    robot.attach(gripper, parent_link="wrist_3_link")
    return robot, gripper


if __name__ == '__main__':
    root = logging.getLogger()
    root.setLevel(logging.INFO)

    world = Multiverse(simulation_name="ur5e_with_task_board")

    # Load environment, robot and objects
    rospack = RosPack()

    robot, gripper = spawn_ur5e_with_gripper()

    jsp = JointStatePublisher()
    # fts = ForceTorqueSensor("ee_fixed_joint")
    robot_arms = [chain.arm_type for chain in robot.robot_description.get_manipulator_chains()]
    with real_robot:
        SetGripperAction(robot_arms, [GripperState.CLOSE]).resolve().perform()

    world.simulate(1)
