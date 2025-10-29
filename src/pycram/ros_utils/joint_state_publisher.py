import time

import threading
import atexit

from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.world import World
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from ..ros import  Time
from ..ros import  create_publisher


class JointStatePublisher:
    """
    Joint state publisher for all robots currently loaded in the World
    """
    def __init__(self, world: World, joint_state_topic="/pycram/joint_state", interval=0.1):
        """
        Robot object is from :py:attr:`~pycram.world.World.robot` and current joint states are published to
        the given joint_state_topic as a JointState message.

        :param world: World object from which the joint states should be read
        :param joint_state_topic: Topic name to which the joint states should be published
        :param interval: Interval at which the joint states should be published, in seconds
        """
        self.world = world

        self.joint_state_pub = create_publisher(joint_state_topic, JointState, queue_size=10)
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
        robot_views = self.world.get_semantic_annotations_by_type(AbstractRobot)
        dofs = {dof for robot_view in robot_views
                     for connection in self.world.get_connections_of_branch(robot_view.root)
                     for dof in connection.dofs}

        while not self.kill_event.is_set():
            current_joint_states = [self.world.state[dof.name].position for dof in dofs]
            h = Header()
            h.stamp = Time().now()
            h.frame_id = ""
            joint_state_msg = JointState()
            joint_state_msg.header = h
            joint_state_msg.name = [dof.name.name for dof in dofs]
            joint_state_msg.position = current_joint_states
            # joint_state_msg.velocity = [joint_state[1] for joint_state in current_joint_states]
            self.joint_state_pub.publish(joint_state_msg)
            time.sleep(self.interval)

    def _stop_publishing(self) -> None:
        """
        Sets the kill_event to terminate the publishing thread and joins the thread.
        """
        self.kill_event.set()
        self.thread.join()
