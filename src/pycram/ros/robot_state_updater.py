import rospy
import atexit
import tf
import time 

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from ..datastructures.world import World
from ..robot_descriptions import robot_description
from ..datastructures.pose import Pose


class RobotStateUpdater:
    """
    Updates the robot in the World with information of the real robot published to ROS topics.
    Infos used to update the robot are:

        * The current pose of the robot
        * The current joint state of the robot
    """

    def __init__(self, tf_topic: str, joint_state_topic: str):
        """
        The robot state updater uses a TF topic and a joint state topic to get the current state of the robot.

        :param tf_topic: Name of the TF topic, needs to publish geometry_msgs/TransformStamped
        :param joint_state_topic: Name of the joint state topic, needs to publish sensor_msgs/JointState
        """
        self.tf_listener = tf.TransformListener()
        time.sleep(1)
        self.tf_topic = tf_topic
        self.joint_state_topic = joint_state_topic

        self.tf_timer = rospy.Timer(rospy.Duration.from_sec(0.1), self._subscribe_tf)
        self.joint_state_timer = rospy.Timer(rospy.Duration.from_sec(0.1), self._subscribe_joint_state)

        atexit.register(self._stop_subscription)

    def _subscribe_tf(self, msg: TransformStamped) -> None:
        """
        Callback for the TF timer, will do a lookup of the transform between map frame and the robot base frame.

        :param msg: TransformStamped message published to the topic
        """
        trans, rot = self.tf_listener.lookupTransform("/map", robot_description.base_frame, rospy.Time(0))
        World.robot.set_pose(Pose(trans, rot))

    def _subscribe_joint_state(self, msg: JointState) -> None:
        """
        Sets the current joint configuration of the robot in the world to the configuration published on the
        topic. Since this uses rospy.wait_for_message which can have errors when used with threads there might be an
        attribute error in the rospy implementation.

        :param msg: JointState message published to the topic.
        """
        try:
            msg = rospy.wait_for_message(self.joint_state_topic, JointState)
            for name, position in zip(msg.name, msg.position):
                World.robot.set_joint_position(name, position)
        except AttributeError:
            pass

    def _stop_subscription(self) -> None:
        """
        Stops the Timer for TF and joint states and therefore the updating of the robot in the world.
        """
        self.tf_timer.shutdown()
        self.joint_state_timer.shutdown()
