import rospy
import threading

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from pycram.bullet_world import BulletWorld
from pycram.robot_descriptions import robot_description
from pycram.pose import Transform


class RobotStateUpdater:
    """
    Updates the robot in the Bullet World with information of the real robot published to ROS topics.
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
        self.tf_topic = tf_topic
        self.joint_state_topic = joint_state_topic
        self.kill_event = threading.Event()

        self.tf_subscriber = rospy.Subscriber(self.tf_topic, TransformStamped, self._subscribe_tf)
        self.joint_state_subscriber = rospy.Subscriber(self.joint_state_topic, JointState, self._subscribe_joint_state)

    def _subscribe_tf(self, msg: TransformStamped) -> None:
        """
        Callback for the given TF topic, discards every message which does not has the robot base frame as child_frame.
        If the message has the robot base frame as child_frame the current pose in the bullet world is set to the pose
        in the message.

        :param msg: TransformStamped message published to the topic
        """
        if msg.child_frame_id == robot_description.base_frame:
            BulletWorld.robot.set_pose(Transform.from_transform_stamped(msg).to_pose())

    def _subscribe_joint_state(self, msg: JointState) -> None:
        """
        Sets the current joint configuration of the robot in the bullet world to the configuration published on the topic.

        :param msg: JointState message published to the topic.
        """
        for name, position in zip(msg.name, msg.position):
            BulletWorld.robot.set_joint_state(name, position)

    def stop_subscription(self) -> None:
        """
        Stops the subscriber for TF and joint states and therefore the updating of the robot in the bullet world.
        """
        self.tf_subscriber.unregister()
        self.joint_state_subscriber.unregister()
