import rospkg

from pycram.robot_description import RobotDescriptionManager, RobotDescription

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + "turtlebot3_waffle_pi" + '.urdf'

turtlebot3_waffle_pi = RobotDescription("turtlebot3_waffle_pi", "world", "base_link", "base_joint",
                                        filename)

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(turtlebot3_waffle_pi)
