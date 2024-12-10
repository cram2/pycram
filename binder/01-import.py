import sys

from demos.pycram_virtual_building_demos.setup.setup_utils import get_robot_name

sys.path.insert(0, '/home/jovyan/workspace/ros/src/pycram')

from demos.pycram_virtual_building_demos.setup.setup_launch_robot import launch_pr2, launch_hsrb, launch_stretch, \
    launch_tiago, launch_justin, launch_donbot, launch_armar, launch_icub, launch_fetch
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.designators.object_designator import *
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher, VizMarkerRobotPublisher
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
elif robot_param == 'justin':
    launch_justin()
elif robot_param == 'donbot':
    launch_donbot()
elif robot_param == 'armar':
    launch_armar()
elif robot_param == 'icub':
    launch_icub()
elif robot_param == 'fetch':
    launch_fetch()

robot_name = get_robot_name(robot_param)

extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)

# Set this to True to publish costmaps and axis marker during the demo. May slow down the simulation.
world.allow_publish_debug_poses = False

VizMarkerPublisher()

# robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))
# apartment = Object(environment_param, ObjectType.ENVIRONMENT, f"{environment_param}{extension}")
# if robot_param not in ["pr2"]:
#     VizMarkerRobotPublisher()
# else:
#     TFBroadcaster()
