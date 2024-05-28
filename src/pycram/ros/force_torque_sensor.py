import atexit
import time
import threading

import rospy

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header
from ..datastructures.world import World


class ForceTorqueSensor:
    """
    Simulated force-torque sensor for a joint with a given name.
    Reads simulated forces and torques at that joint from world and publishes geometry_msgs/Wrench messages
    to the given topic.
    """
    def __init__(self, joint_name, fts_topic="/pycram/fts", interval=0.1):
        """
        The given joint_name has to be part of :py:attr:`~pycram.world.World.robot` otherwise a
        RuntimeError will be raised.

        :param joint_name: Name of the joint for which force-torque should be simulated
        :param fts_topic: Name of the ROS topic to which should be published
        :param interval: Interval at which the messages should be published, in seconds
        """
        self.world = World.current_world
        self.fts_joint_idx = None
        self.joint_name = joint_name
        if joint_name in self.world.robot.joint_name_to_id.keys():
            self.fts_joint_idx = self.world.robot.joint_name_to_id[joint_name]
        else:
            raise RuntimeError(f"Could not register ForceTorqueSensor: Joint {joint_name}"
                               f" does not exist in robot object")
        self.world.enable_joint_force_torque_sensor(self.world.robot, self.fts_joint_idx)

        self.fts_pub = rospy.Publisher(fts_topic, WrenchStamped, queue_size=10)
        self.interval = interval
        self.kill_event = threading.Event()

        self.thread = threading.Thread(target=self._publish)
        self.thread.start()

        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Continuously publishes the force-torque values for the simulated joint. Values are published as long as the
        kill_event is not set.
        """
        seq = 0
        while not self.kill_event.is_set():
            joint_ft = self.world.get_joint_reaction_force_torque(self.world.robot, self.fts_joint_idx)
            h = Header()
            h.seq = seq
            h.stamp = rospy.Time.now()
            h.frame_id = self.joint_name

            wrench_msg = WrenchStamped()
            wrench_msg.header = h
            wrench_msg.wrench.force.x = joint_ft[0]
            wrench_msg.wrench.force.y = joint_ft[1]
            wrench_msg.wrench.force.z = joint_ft[2]

            wrench_msg.wrench.torque.x = joint_ft[3]
            wrench_msg.wrench.torque.y = joint_ft[4]
            wrench_msg.wrench.torque.z = joint_ft[5]

            self.fts_pub.publish(wrench_msg)
            seq += 1
            time.sleep(self.interval)

    def _stop_publishing(self) -> None:
        """
        Sets the kill_event and therefore terminates the Thread publishing the force-torque values as well as join the
        threads.
        """
        self.kill_event.set()
        self.thread.join()
