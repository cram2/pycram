import time
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


# For future work / robots
# def launch_hsrb():
#    # name = 'hsrb'
#    # urdf = 'hsrb.urdf'
#    executable = 'hsrb_standalone.launch'
#    launch_robot(executable)


# def launch_armar6():
#    # name = 'armar6'
#    # urdf = 'armar6.urdf'
#    executable = 'armar6_standalone.launch'
#    launch_robot(executable)


def launch_robot(launch_file, package='pycram', launch_folder='/launch/'):
    """
    General method to start a specified launch file with given parameters.
    Default location for launch files here is in the folder 'launch' inside the pycram package

    :param launch_file: File name of the launch file
    :param package: Name of the package
    :param launch_folder: Location of the launch file inside the package
    """

    rospath = rospkg.RosPack()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [rospath.get_path(package) + launch_folder + launch_file])
    launch.start()

    rospy.loginfo(f'{launch_file} started')

    # Wait for ik server to launch
    time.sleep(2)
