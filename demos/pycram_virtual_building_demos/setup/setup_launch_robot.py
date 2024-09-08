import logging
import time
from typing import List

import roslaunch
import rospy
import rospkg


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


def launch_justin():
    # name = 'rollin_justin'
    # urdf = 'rollin_justin.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=justin"]
    launch_robot(executable, args=args)

def launch_donbot():
    # name = 'iai_donbot'
    # urdf = 'iai_donbot.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=donbot"]
    launch_robot(executable, args=args)


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
