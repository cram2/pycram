import logging
import os

from rospkg import RosPack
import pybullet as pb

from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.world import Object
from pycram.datastructures.pose import PoseStamped
from pycram.ros_utils.force_torque_sensor import ForceTorqueSensor
from pycram.ros_utils.joint_state_publisher import JointStatePublisher
from pycram.ros_utils.tf_broadcaster import TFBroadcaster

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PYCRAM_DIR = os.path.join(SCRIPT_DIR, os.pardir, os.pardir)
RESOURCE_DIR = os.path.join(PYCRAM_DIR, "resources")

SPAWNING_POSES = {
    "robot": PoseStamped.from_list([0, 0, 0], [0.0, 0.0, 0.0, 1.0]),   # x,y,z,qx,qy,qz,qw
    "cereal": PoseStamped.from_list([0.5, 0.5, 2.0], [0.0, 0.0, 0.0, 1.0])
}


if __name__ == '__main__':
    root = logging.getLogger()
    root.setLevel(logging.INFO)

    world = BulletWorld()
    pb.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=30, cameraPitch=-70, cameraTargetPosition=[0, 0, 0])

    # Load environment, robot and objects
    rospack = RosPack()
    kitchen_urdf_path = os.path.join(RESOURCE_DIR, "kitchen.urdf")
    robot_urdf_path = os.path.join(RESOURCE_DIR, "ur5_robotiq.urdf")

    plane = Object("floor", "environment", os.path.join(RESOURCE_DIR, "plane.urdf"), world=world)
    lab = Object("kitchen", "environment", kitchen_urdf_path)
    robot = Object("ur", "robot", robot_urdf_path, pose=SPAWNING_POSES["robot"])
    cereal = Object("cereal", "object", os.path.join(RESOURCE_DIR, "breakfast_cereal.stl"),
                    pose=SPAWNING_POSES["cereal"])
    BulletWorld.robot = robot

    tf_broadcaster = TFBroadcaster("projection", "odom", 1.0)
    jsp = JointStatePublisher()
    fts = ForceTorqueSensor("ee_fixed_joint")

    world.simulate(60)
