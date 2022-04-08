import time

#import roslibpy
import rospy
import pybullet as pb

from ros.ros_topic_publisher import ROSTopicPublisher
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header

class ForceTorqueSensor(ROSTopicPublisher):
    """
    Simulated force-torque sensor for a joint with a given name.
    Reads simulated forces and torques at that joint from bullet_world and publishes geometry_msgs/Wrench messages
    to the given topic.
    """
    def __init__(self, bullet_world, joint_name, fts_topic="/pycram/fts", interval=0.1):
        super().__init__()
        self.world = bullet_world
        self.fts_joint_idx = None
        for joint_idx in range(pb.getNumJoints(self.world.robot.id)):
            joint_info = pb.getJointInfo(self.world.robot.id, joint_idx)
            if joint_info[1].decode("utf-8") == joint_name:
                self.fts_joint_idx = joint_idx
        if self.fts_joint_idx is None:
            raise RuntimeError(f"Could not register ForceTorqueSensor: Joint {joint_name} does not exist")
        pb.enableJointForceTorqueSensor(self.world.robot.id, self.fts_joint_idx, enableSensor=1)

        #self.fts_pub = roslibpy.Topic(self.ros_client, fts_topic, "geometry_msgs/WrenchStamped")
        self.fts_pub = rospy.Publisher(fts_topic, WrenchStamped, queue_size=10)
        self.interval = interval

    def _publish(self):
        seq = 0
        while not self.kill_event.is_set():
            current_joint_state = pb.getJointState(self.world.robot.id, self.fts_joint_idx)
            joint_ft = current_joint_state[2]
            h = Header()
            h.seq = seq
            h.stamp = rospy.Time.now()

            wrench_msg = WrenchStamped()
            wrench_msg.header = h
            wrench_msg.wrench.force.x = joint_ft[0]
            wrench_msg.wrench.force.y = joint_ft[1]
            wrench_msg.wrench.force.z = joint_ft[2]

            wrench_msg.wrench.torque.x = joint_ft[3]
            wrench_msg.wrench.torque.y = joint_ft[4]
            wrench_msg.wrench.torque.z = joint_ft[5]

            # wrench_msg = roslibpy.Message({
            #     "header": roslibpy.Header(seq, roslibpy.Time.now(), frame_id=""),
            #     "wrench":
            #         {
            #             "force": dict(zip(["x", "y", "z"], joint_ft[:3])),
            #             "torque": dict(zip(["x", "y", "z"], joint_ft[3:]))
            #         }
            # })
            self.fts_pub.publish(wrench_msg)
            seq += 1
            time.sleep(self.interval)
