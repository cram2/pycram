import logging
import os

from rospkg import RosPack
import pybullet as pb

from pycram import robot_description
from pycram.bullet_world import BulletWorld, Object
from ros.force_torque_sensor import ForceTorqueSensor
from ros.joint_state_publisher import JointStatePublisher
from ros.tf_broadcaster import TFBroadcaster

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PYCRAM_DIR = os.path.join(SCRIPT_DIR, os.pardir, os.pardir)
RESOURCE_DIR = os.path.join(PYCRAM_DIR, "resources")

SPAWNING_POSES = {
    "robot": [0, 0, 0, 0.0, 0.0, 0.0, 1.0],   # x,y,z,qx,qy,qz,qw
    "cereal": [0.5, 0.5, 2.0, 0.0, 0.0, 0.0, 1.0],
}


if __name__ == '__main__':
    root = logging.getLogger()
    root.setLevel(logging.INFO)

    world = BulletWorld()
    pb.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=30, cameraPitch=-70, cameraTargetPosition=[0,0,0])
    world.set_gravity([0, 0, -9.8])

    # Load environment, robot and objects
    rospack = RosPack()
    kitchen_urdf_path = os.path.join(RESOURCE_DIR, "kitchen.urdf")
    robot_urdf_path = os.path.join(RESOURCE_DIR, "ur5_robotiq.urdf")

    plane = Object("floor", "environment", os.path.join(RESOURCE_DIR, "plane.urdf"), world=world)
    lab = Object("kitchen", "environment", kitchen_urdf_path)
    robot = Object("ur", "robot", robot_urdf_path, position=SPAWNING_POSES["robot"][:3],
                   orientation=SPAWNING_POSES["robot"][3:])
    cereal = Object("cereal", "object", os.path.join(RESOURCE_DIR, "breakfast_cereal.stl"),
                         position=SPAWNING_POSES["cereal"][:3], orientation=SPAWNING_POSES["cereal"][3:])
    BulletWorld.robot = robot

    tf_broadcaster = TFBroadcaster(world, "map", "odom", "projection", "iai-kitchen", interval=1.0)
    jsp = JointStatePublisher(world)
    fts = ForceTorqueSensor(world, "ee_fixed_joint")

    with tf_broadcaster, jsp, fts:
        world.simulate(60)
