from ..ros import get_ros_package_path

from ..robot_description import RobotDescriptionManager, RobotDescription

# Description for turtlebot3_waffle_pi
filename = get_ros_package_path("pycram") + "/resources/robots/" + "turtlebot" + ".urdf"

turtlebot = RobotDescription("turtlebot", "world", "base_link", "base_joint", filename)

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(turtlebot)
