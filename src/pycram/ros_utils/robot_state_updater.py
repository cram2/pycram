import atexit
from datetime import timedelta
import time

import tf
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from typing_extensions import Optional

from ..datastructures.world import World
from ..robot_description import RobotDescription
from ..datastructures.pose import PoseStamped, Pose
from ..ros import  Time, Duration
from ..ros import  wait_for_message, create_timer


class WorldStateUpdater:
    """
    Updates the robot in the World with information of the real robot published to ROS topics.
    Infos used to update the robot are:

        * The current pose of the robot
        * The current joint state of the robot
    """

    def __init__(self, tf_topic: str, joint_state_topic: str, update_rate: timedelta = timedelta(milliseconds=100),
                 world: Optional[World] = None) -> None:
        """
        The robot state updater uses a TF topic and a joint state topic to get the current state of the robot.

        :param tf_topic: Name of the TF topic, needs to publish geometry_msgs/TransformStamped
        :param joint_state_topic: Name of the joint state topic, needs to publish sensor_msgs/JointState
        """
        self.tf_listener = tf.TransformListener()
        time.sleep(1)
        self.tf_topic = tf_topic
        self.joint_state_topic = joint_state_topic
        self.world: Optional[World] = world
        self.tf_timer = create_timer(Duration().from_sec(update_rate.total_seconds()), self._subscribe_tf)
        self.joint_state_timer = create_timer(Duration().from_sec(update_rate.total_seconds()),
                                              self._subscribe_joint_state)

        atexit.register(self._stop_subscription)

    def _subscribe_tf(self, msg: TransformStamped) -> None:
        """
        Callback for the TF timer, will do a lookup of the transform between map frame and the objects frames.

        :param msg: TransformStamped message published to the topic
        """
        if self.world is None:
            if not World.current_world.is_prospection_world:
                self.world = World.current_world
            else:
                return
        for obj in self.world.objects:
            if obj.name == self.world.robot.name:
                tf_frame = RobotDescription.current_robot_description.base_link
            elif obj.is_an_environment:
                continue
            else:
                tf_frame = obj.tf_frame

            trans, rot = self.tf_listener.lookupTransform("/map", tf_frame, Time(0))
            obj.set_pose(PoseStamped.from_list(trans, rot))

    def _subscribe_joint_state(self, msg: JointState) -> None:
        """
        Sets the current joint configuration of the robot in the world to the configuration published on the
        topic. Since this uses rospy.wait_for_message which can have errors when used with threads there might be an
        attribute error in the rospy implementation.

        :param msg: JointState message published to the topic.
        """
        try:
            msg = wait_for_message(self.joint_state_topic, JointState)
            joint_positions = dict(zip(msg.name, msg.position))
            World.robot.set_multiple_joint_positions(joint_positions)
        except AttributeError:
            pass

    def _stop_subscription(self) -> None:
        """
        Stops the Timer for TF and joint states and therefore the updating of the robot in the world.
        """
        self.tf_timer.shutdown()
        self.joint_state_timer.shutdown()

