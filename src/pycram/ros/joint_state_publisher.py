import time
import threading
import atexit

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pycram.world import World


class JointStatePublisher:
    """
    Joint state publisher for the robot currently loaded in the World
    """
    def __init__(self, joint_state_topic="/pycram/joint_state", interval=0.1):
        """
        Robot object is from :py:attr:`~pycram.world.World.robot` and current joint states are published to
        the given joint_state_topic as a JointState message.

        :param joint_state_topic: Topic name to which the joint states should be published
        :param interval: Interval at which the joint states should be published, in seconds
        """
        self.world = World.current_world

        self.joint_state_pub = rospy.Publisher(joint_state_topic, JointState, queue_size=10)
        self.interval = interval
        self.kill_event = threading.Event()
        self.thread = threading.Thread(target=self._publish)
        self.thread.start()

        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Publishes the current joint states of the :py:attr:`~pycram.world.World.robot` in an infinite loop.
        The joint states are published as long as the kill_event is not set by :py:meth:`~JointStatePublisher._stop_publishing`
        """
        robot = World.robot
        joint_names = [joint_name for joint_name in robot.joint_name_to_id.keys()]
        seq = 0

        while not self.kill_event.is_set():
            current_joint_states = [robot.get_joint_position(joint_name) for joint_name in joint_names]
            h = Header()
            h.stamp = rospy.Time.now()
            h.seq = seq
            h.frame_id = ""
            joint_state_msg = JointState()
            joint_state_msg.header = h
            joint_state_msg.name = joint_names
            joint_state_msg.position = current_joint_states
            # joint_state_msg.velocity = [joint_state[1] for joint_state in current_joint_states]
            self.joint_state_pub.publish(joint_state_msg)
            seq += 1
            time.sleep(self.interval)

    def _stop_publishing(self) -> None:
        """
        Sets the kill_event to terminate the publishing thread and joins the thread.
        """
        self.kill_event.set()
        self.thread.join()
