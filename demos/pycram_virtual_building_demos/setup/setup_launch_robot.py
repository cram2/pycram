import logging
import time
from typing import List

import roslaunch
import rospy
import rospkg
import os
import sys
from contextlib import contextmanager


def launch_pr2():
    """
    Method to launch PR2 on the parameter server and start the ik service
    """
    # name = 'pr2'
    # urdf = 'pr2.urdf'
    executable = 'pr2_standalone.launch'
    launch_robot(executable)


def launch_hsrb():
    # name = 'hsrb'
    # urdf = 'hsrb.urdf'
    executable = 'hsrb_standalone.launch'
    launch_robot(executable)


def launch_stretch():
    # name = 'stretch'
    # urdf = 'stretch_description.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=stretch"]
    launch_robot(executable, args=args)

def launch_tiago():
    # name = 'tiago_dual'
    # urdf = 'tiago_dual.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=tiago_dual"]
    launch_robot(executable, args=args)


@contextmanager
def suppress_all_output():
    # Open /dev/null to suppress output
    with open(os.devnull, 'w') as devnull:
        # Save original file descriptors and Python stdout/stderr
        old_stdout_fd = os.dup(1)
        old_stderr_fd = os.dup(2)
        old_stdout = sys.stdout
        old_stderr = sys.stderr

        # Backup rospy log functions to restore them later
        original_loginfo = rospy.loginfo
        original_logwarn = rospy.logwarn
        original_logerr = rospy.logerr
        original_logdebug = rospy.logdebug
        original_logfatal = rospy.logfatal

        try:
            # Redirect Python stdout and stderr to /dev/null
            sys.stdout = devnull
            sys.stderr = devnull

            # Redirect C-level stdout and stderr to /dev/null (file descriptors)
            os.dup2(devnull.fileno(), 1)  # Redirect stdout (file descriptor 1)
            os.dup2(devnull.fileno(), 2)  # Redirect stderr (file descriptor 2)

            # Override rospy log functions to suppress them
            rospy.loginfo = lambda *args, **kwargs: None
            rospy.logwarn = lambda *args, **kwargs: None
            rospy.logerr = lambda *args, **kwargs: None
            rospy.logdebug = lambda *args, **kwargs: None
            rospy.logfatal = lambda *args, **kwargs: None

            # Yield control back to the function inside this context
            yield

        finally:
            # Restore Python stdout and stderr
            sys.stdout = old_stdout
            sys.stderr = old_stderr

            # Restore C-level stdout and stderr
            os.dup2(old_stdout_fd, 1)
            os.dup2(old_stderr_fd, 2)
            os.close(old_stdout_fd)
            os.close(old_stderr_fd)

            # Restore original rospy log functions
            rospy.loginfo = original_loginfo
            rospy.logwarn = original_logwarn
            rospy.logerr = original_logerr
            rospy.logdebug = original_logdebug
            rospy.logfatal = original_logfatal

def launch_robot(launch_file, package='pycram', launch_folder='/launch/', args: List[str] = None):
    """
    General method to start a specified launch file with given parameters.
    Default location for launch files here is in the folder 'launch' inside the pycram package

    :param launch_file: File name of the launch file
    :param package: Name of the package
    :param launch_folder: Location of the launch file inside the package
    :param args: List of arguments to pass onto the launch file
    """
    # Suppress all output from the function
    with suppress_all_output():
        try:
            rospath = rospkg.RosPack()

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch_args = rospath.get_path(package) + launch_folder + launch_file

            if args is None:
                args = [""]

            args.insert(0, launch_args)
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(args)[0], args[1:])]
            print(roslaunch_file)
            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            launch.start()

            rospy.loginfo(f'{launch_file} started')

            # Wait for ik server to launch
            time.sleep(2)
        except Exception:
         pass