import logging

from IPython.core.display_functions import clear_output
from ipywidgets import Output

from demos.pycram_bullet_world_demo.demo import transporting_demo
from demos.pycram_virtual_building_demos.setup.setup_launch_robot import *
import threading

from ipywidgets import Output

from demos.pycram_virtual_building_demos.setup.setup_launch_robot import launch_pr2, launch_hsrb, launch_stretch, \
    launch_tiago
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
from IPython.display import display
import os
import sys
from contextlib import contextmanager

output = None

def start_demo():
    # get params
    environment_param = rospy.get_param('/nbparam_environments')
    robot_param = rospy.get_param('/nbparam_robots')
    task_param = rospy.get_param('/nbparam_tasks')
    # if robot_param == 'pr2':
    #     launch_pr2()
    # elif robot_param == 'hsrb':
    #     launch_hsrb()
    # elif robot_param == 'stretch':
    #     launch_stretch()
    # elif robot_param == 'tiago':
    #     launch_tiago()
    #     robot_param = 'tiago_dual'
    # elif robot_param == 'justin':
    #     launch_justin()
    #     robot_param = "rollin_justin"
    # clear_output(wait=True)
    # text widget for the virtual building
    text_widget = display_loading_gif_with_text()
    update_text(text_widget, 'Loading Everything...')
    update_text(text_widget, 'Loading envi: ' + environment_param + ' robot: ' + robot_param + ' task: ' + task_param)
    # actual world setup
    extension = ObjectDescription.get_file_extension()
    world = BulletWorld(WorldMode.DIRECT)
    VizMarkerPublisher()
    Object(robot_param, ObjectType.ROBOT, f"{robot_param}{extension}", pose=Pose([1, 2, 0]))
    apartment = Object(environment_param, ObjectType.ENVIRONMENT, f"{environment_param}{extension}")
    tf = TFBroadcaster()

    update_text(text_widget, 'Setup Done -> Starting Demo')

    demo_selecting(apartment, task_param)

    update_text(text_widget, 'Done with the task...')


def demo_selecting(apartment, task_param):
    display(output)
    with output:
        if task_param == "navigate":
            navigate_simple_example()
        elif task_param == "transport":
            rospy.loginfo('Starting transporting demo...')
            transporting_demo(apartment)
