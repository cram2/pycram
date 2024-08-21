import rospkg

from pycram.robot_description import RobotDescriptionManager, RobotDescription

# Description for turtlebot3_waffle_pi
rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "turtlebot" + '.urdf'

turtlebot = RobotDescription("turtlebot", "world", "base_link", "base_joint",
                             filename)

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(turtlebot)
