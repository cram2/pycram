import sys

sys.path.insert(0, '/home/jovyan/workspace/ros/src/pycram')

from demos.pycram_virtual_building_demos.setup.setup_launch_robot import launch_pr2, launch_hsrb, launch_stretch, \
    launch_tiago
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.designators.object_designator import *
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


robot_param = rospy.get_param('/nbparam_robots')
environment_param = rospy.get_param('/nbparam_environments')
if robot_param == 'pr2':
    launch_pr2()
elif robot_param == 'hsrb':
    launch_hsrb()
elif robot_param == 'stretch':
    launch_stretch()
elif robot_param == 'tiago':
    launch_tiago()
extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.DIRECT)
VizMarkerPublisher()
tf = TFBroadcaster()
Object(robot_param, ObjectType.ROBOT, f"{robot_param}{extension}", pose=Pose([1, 2, 0]))
Object("environment", ObjectType.ENVIRONMENT, f"{environment_param}-small{extension}")
