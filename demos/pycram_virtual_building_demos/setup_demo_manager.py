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
import os
import sys
from contextlib import contextmanager


class OutputSuppressor:
    def __init__(self):
        # Initialize placeholders for old stdout and stderr
        self._old_stdout_fd = None
        self._old_stderr_fd = None
        self._old_stdout = None
        self._old_stderr = None

        # Backup rospy log functions to restore them later
        self._original_loginfo = None
        self._original_logwarn = None
        self._original_logerr = None
        self._original_logdebug = None
        self._original_logfatal = None

    def backup_outputs(self):
        """Backup current stdout/stderr and rospy log functions."""
        self._old_stdout_fd = os.dup(1)  # Backup file descriptor for stdout
        self._old_stderr_fd = os.dup(2)  # Backup file descriptor for stderr
        self._old_stdout = sys.stdout  # Backup Python stdout
        self._old_stderr = sys.stderr  # Backup Python stderr

        # Backup rospy log functions
        self._original_loginfo = rospy.loginfo
        self._original_logwarn = rospy.logwarn
        self._original_logerr = rospy.logerr
        self._original_logdebug = rospy.logdebug
        self._original_logfatal = rospy.logfatal

    def restore_outputs(self):
        """Restore stdout/stderr and rospy log functions."""
        # Restore Python stdout and stderr first
        if self._old_stdout:
            sys.stdout = self._old_stdout
        if self._old_stderr:
            sys.stderr = self._old_stderr

        # Restore C-level stdout and stderr if valid
        if self._old_stdout_fd is not None:
            try:
                os.dup2(self._old_stdout_fd, 1)
                os.close(self._old_stdout_fd)
            except OSError as e:
                print(f"Error restoring stdout: {e}")

        if self._old_stderr_fd is not None:
            try:
                os.dup2(self._old_stderr_fd, 2)
                os.close(self._old_stderr_fd)
            except OSError as e:
                print(f"Error restoring stderr: {e}")

        # Restore original rospy log functions
        rospy.loginfo = self._original_loginfo
        rospy.logwarn = self._original_logwarn
        rospy.logerr = self._original_logerr
        rospy.logdebug = self._original_logdebug
        rospy.logfatal = self._original_logfatal

    @contextmanager
    def suppress_all_output(self):
        """Context manager to suppress all output."""
        self.backup_outputs()  # Backup outputs

        with open(os.devnull, 'w') as devnull:
            try:
                # Redirect Python stdout and stderr to /dev/null
                sys.stdout = devnull
                sys.stderr = devnull

                # Redirect C-level stdout and stderr to /dev/null (file descriptors)
                os.dup2(devnull.fileno(), 1)
                os.dup2(devnull.fileno(), 2)

                # Override rospy log functions to suppress them
                rospy.loginfo = lambda *args, **kwargs: None
                rospy.logwarn = lambda *args, **kwargs: None
                rospy.logerr = lambda *args, **kwargs: None
                rospy.logdebug = lambda *args, **kwargs: None
                rospy.logfatal = lambda *args, **kwargs: None

                # Yield control back to the function inside this context
                yield

            finally:
                # Restore original stdout, stderr, and log functions
                self.restore_outputs()


# def launch_robot_thread(robot_param):
#     supressor = OutputSuppressor()
#     try:
#         with supressor.suppress_all_output():
#             if robot_param == 'pr2':
#                 launch_pr2()
#             elif robot_param == 'hsrb':
#                 launch_hsrb()
#             elif robot_param == 'stretch':
#                 launch_stretch()
#             elif robot_param == 'tiago':
#                 launch_tiago()
#     finally:
#         supressor.restore_outputs()


def start_demo():
    global output
    output = Output()

    environment_param = rospy.get_param('/nbparam_environments')
    robot_param = rospy.get_param('/nbparam_robots')
    task_param = rospy.get_param('/nbparam_tasks')
    # launch_robot_thread(robot_param)
    # robot_thread = threading.Thread(target=launch_robot_thread, args=(robot_param,))
    # robot_thread.start()
    text_widget = display_loading_gif_with_text()
    update_text(text_widget, 'Loading Everything...')
    update_text(text_widget, 'Loading envi: ' + environment_param + ' robot: ' + robot_param + ' task: ' + task_param)

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
    print('Done with the task...')


start_demo()
    #
    # if task_param == "navigate":
    #     navigate_simple_example()
    #
    # update_text(text_widget, 'Done with the task...')
