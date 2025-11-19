import atexit
from datetime import timedelta
import time

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from typing_extensions import Optional

from ..datastructures.world import World
from ..robot_description import RobotDescription
from ..datastructures.pose import PoseStamped
from ..ros import  Time, Duration
from ..ros import  wait_for_message, create_timer, node


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
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        time.sleep(1)
        self.tf_topic = tf_topic
        self.joint_state_topic = joint_state_topic
        self.world: Optional[World] = world
        self.tf_timer = create_timer(update_rate.total_seconds(), self._subscribe_tf)
        self.joint_state_timer = create_timer(update_rate.total_seconds(), self._subscribe_joint_state)
        self.joint_state_subscriber = node.create_subscription(JointState, joint_state_topic, self.joint_state_callback, 1)
        self.joint_states = JointState()

        atexit.register(self._stop_subscription)

    def _subscribe_tf(self) -> None:
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
            elif obj.is_an_object:
                continue
            else:
                tf_frame = obj.tf_frame
            transform = self.tf_buffer.lookup_transform("map", tf_frame, Time(0))
            trans = [transform.transform.translation.x, transform.transform.translation.y,
                     transform.transform.translation.z]
            rot = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z,
                   transform.transform.rotation.w]
            obj.set_pose(PoseStamped.from_list(trans, rot))

    def _subscribe_joint_state(self) -> None:
        """
        Sets the current joint configuration of the robot in the world to the configuration published on the
        topic.

        :param msg: JointState message published to the topic.
        """
        try:
            msg = self.joint_states
            if msg:
                joint_positions = dict(zip(msg.name, msg.position))
                World.robot.set_multiple_joint_positions(joint_positions)
        except AttributeError or RuntimeError:
            pass

    def joint_state_callback(self, msg):
        self.joint_states = msg
        return

    def _stop_subscription(self) -> None:
        """
        Stops the Timer for TF and joint states and therefore the updating of the robot in the world.
        """
        self.tf_timer.cancel()
        self.joint_state_timer.cancel()
        node.destroy_subscription(self.joint_state_topic)

