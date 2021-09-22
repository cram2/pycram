import time

import roslibpy
import pybullet as pb

from ros.ros_topic_publisher import ROSTopicPublisher
from ros.rosbridge import ros_client


class JointStatePublisher(ROSTopicPublisher):
    def __init__(self, bullet_world, joint_state_topic="/pycram/joint_state", interval=0.1):
        super().__init__()
        self.world = bullet_world

        self.joint_state_pub = roslibpy.Topic(ros_client, joint_state_topic, "sensor_msgs/JointState")
        self.interval = interval

    def _publish(self):
        num_joints = pb.getNumJoints(self.world.robot.id)
        joint_names = []
        for joint_idx in range(num_joints):
            joint_info = pb.getJointInfo(self.world.robot.id, joint_idx)
            joint_names.append(joint_info[1].decode("utf-8"))
        joint_indices = list(range(num_joints))
        seq = 0

        while not self.kill_event.is_set():
            current_joint_states = pb.getJointStates(self.world.robot.id, joint_indices)
            joint_state_msg = roslibpy.Message({
                "header": roslibpy.Header(seq=seq, frame_id="", stamp=roslibpy.Time.now()),
                "name": joint_names,
                "position": [joint_state[0] for joint_state in current_joint_states],
                "velocity": [joint_state[1] for joint_state in current_joint_states]
            })
            self.joint_state_pub.publish(joint_state_msg)
            seq += 1
            time.sleep(self.interval)
