import logging
import os

from rospkg import RosPack

from pycram.datastructures.enums import ObjectType
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import Pose
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.ros_utils.joint_state_publisher import JointStatePublisher
from pycram.worlds.multiverse import Multiverse

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PYCRAM_DIR = os.path.join(SCRIPT_DIR, os.pardir, os.pardir)
RESOURCE_DIR = os.path.join(PYCRAM_DIR, "resources")

SPAWNING_POSES = {
    "robot": Pose([0, 0, 0], [0.0, 0.0, 0.0, 1.0]),   # x,y,z,qx,qy,qz,qw
    "cereal": Pose([0.5, 0.5, 2.0], [0.0, 0.0, 0.0, 1.0])
}


if __name__ == '__main__':
    root = logging.getLogger()
    root.setLevel(logging.INFO)

    world = Multiverse(simulation_name="ur5e_with_task_board")

    # Load environment, robot and objects
    rospack = RosPack()

    robot = Object("ur5e", ObjectType.ROBOT, "ur5e_with_gripper/urdf/ur5e.urdf")
    jsp = JointStatePublisher()
    # fts = ForceTorqueSensor("ee_fixed_joint")

    world.simulate(1)
