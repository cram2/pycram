from ipywidgets import Output

from demos.pycram_virtual_building_demos.setup.setup_launch_robot import *
from demos.pycram_virtual_building_demos.setup.setup_utils import display_loading_gif_with_text, update_text
import rospy

from demos.pycram_virtual_building_demos.src.simple_examples import navigate_simple_example
from pycram.datastructures.enums import WorldMode
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.process_module import simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


def start_demo():
    global output
    output = Output()

    environment_param = rospy.get_param('/nbparam_environments')
    robot_param = rospy.get_param('/nbparam_robots')
    task_param = rospy.get_param('/nbparam_tasks')

    if robot_param == 'pr2':
        launch_pr2()
    elif robot_param == 'hsrb':
        launch_hsrb()
    elif robot_param == 'stretch':
        launch_stretch()
    elif robot_param == 'tiago':
        launch_tiago()

    extension = ObjectDescription.get_file_extension()
    BulletWorld(WorldMode.DIRECT)
    VizMarkerPublisher()
    Object('pycram_robot', ObjectType.ROBOT, f"{robot_param}{extension}", pose=Pose([1, 2, 0]))
    Object('pycram_environment', ObjectType.ENVIRONMENT, f"{environment_param}{extension}")

    # text_widget = display_loading_gif_with_text()
    # update_text(text_widget, 'Loading Everything...')
    # update_text(text_widget, 'Loading envi: ' + environment_param + ' robot: ' + robot_param + ' task: ' + task_param)
    # update_text(text_widget, 'Starting Demo')
    tf = TFBroadcaster()
start_demo()
    #
    # if task_param == "navigate":
    #     navigate_simple_example()
    #
    # update_text(text_widget, 'Done with the task...')
