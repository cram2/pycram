import sys

from IPython.core.display_functions import clear_output

from demos.pycram_virtual_building_demos.src.cleanup_demo import cleanup_demo
from demos.pycram_virtual_building_demos.src.generlized_actions_demo import start_generalized_demo
from demos.pycram_virtual_building_demos.src.transport_demo import transporting_demo
from pycram.utils import suppress_stdout_stderr

sys.path.insert(0, '/home/vee/robocup_workspaces/pycram_ws/src/pycram')

from demos.pycram_virtual_building_demos.setup.setup_utils import display_loading_gif_with_text, update_text, \
    get_robot_name

from demos.pycram_virtual_building_demos.src.simple_examples import navigate_simple_example
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.datastructures.enums import WorldMode
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

output = None


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


def start_demo_local():
    # get params
    environment_param = 'apartment'
    robot_param = 'pr2'
    task_param = 'pouring'

    robot_name = get_robot_name(robot_param)

    extension = ObjectDescription.get_file_extension()

    world = BulletWorld(WorldMode)
    VizMarkerPublisher()
    robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))
    apartment = Object(environment_param, ObjectType.ENVIRONMENT, f"{environment_param}{extension}")
    tf = TFBroadcaster()

    demo_selecting(apartment, robot, task_param)
    extension = ObjectDescription.get_file_extension()


def demo_selecting(apartment, robot, task_param):
    if task_param == "navigate":
        navigate_simple_example()
    elif task_param == "transport":
        with suppress_stdout_stderr():
            transporting_demo(apartment, robot)
    elif task_param == "cleanup":
        with suppress_stdout_stderr():
            cleanup_demo(apartment, robot)
    elif task_param in ["cutting", "mixing", "pouring"]:
        object_target = rospy.get_param('/nbparam_object')
        object_tool = rospy.get_param('/nbparam_object_tool')
        specialized_task = rospy.get_param('/nbparam_specialized_task')
        start_generalized_demo(task_param, object_tool, object_target, specialized_task)

start_demo_local()
