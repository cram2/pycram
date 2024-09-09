import logging

from IPython.core.display_functions import clear_output
from ipywidgets import Output

from demos.pycram_virtual_building_demos.setup.setup_launch_robot import *
import threading
import sys

from demos.pycram_virtual_building_demos.src.transport_demo import transporting_demo
from pycram.utils import suppress_stdout_stderr

sys.path.insert(0, '/home/vee/robocup_workspaces/pycram_ws/src/pycram')

from ipywidgets import Output

from demos.pycram_virtual_building_demos.setup.setup_launch_robot import launch_pr2, launch_hsrb, launch_stretch, \
    launch_tiago
from demos.pycram_virtual_building_demos.setup.setup_utils import display_loading_gif_with_text, update_text, \
    get_robot_name
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
from IPython.display import display
import os
import sys
from contextlib import contextmanager

output = None

# THIS IS ALL DONE IN THE SETUP via IPYTHON KERNEL
# # actual world setup
# extension = ObjectDescription.get_file_extension()
# world = BulletWorld(WorldMode.DIRECT)
# VizMarkerPublisher()
# Object(robot_param, ObjectType.ROBOT, f"{robot_param}{extension}", pose=Pose([1, 2, 0]))
# Object(environment_param, ObjectType.ENVIRONMENT, f"{environment_param}{extension}")
# tf = TFBroadcaster()

def start_demo():
    # get params
    environment_param = rospy.get_param('/nbparam_environments')
    robot_param = rospy.get_param('/nbparam_robots')
    task_param = rospy.get_param('/nbparam_tasks')

    robot_name = get_robot_name(robot_param)

    extension = ObjectDescription.get_file_extension()
    # text widget for the virtual building
    text_widget = display_loading_gif_with_text()
    update_text(text_widget, 'Loading process~ Please wait...')

    world = BulletWorld(WorldMode.DIRECT)
    VizMarkerPublisher()
    robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))
    apartment = Object(environment_param, ObjectType.ENVIRONMENT, f"{environment_param}{extension}")
    tf = TFBroadcaster()

    clear_output(wait=True)

    update_text(text_widget, 'Executing Demo: ' + task_param)

    demo_selecting(apartment, robot, task_param)

    extension = ObjectDescription.get_file_extension()

    update_text(text_widget, 'Done with: ' + task_param)


def demo_selecting(apartment, robot, task_param):
    # display(output)
    # with output:
    if task_param == "navigate":
        navigate_simple_example()
    elif task_param == "transport":
        # rospy.loginfo('Starting transporting demo...')
        with suppress_stdout_stderr():
            transporting_demo(apartment, robot)