import logging
import threading
import time

import roslibpy
import pybullet as pb

from ros.rosbridge import ROSBridge


class JointStatePublisher(object):
    def __init__(self, bullet_world, joint_state_topic="/pycram/joint_state", interval=0.1):
        self.ros_client = roslibpy.Ros(*ROSBridge.get_ros_master_host_and_port())
        self.ros_client.run()

        self.world = bullet_world

        self.joint_state_pub = roslibpy.Topic(self.ros_client, joint_state_topic, "sensor_msgs/JointState")
        self.thread = None
        self.kill_event = threading.Event()
        self.interval = interval

    def __del__(self):
        self.ros_client.terminate()

    def start_publishing(self):
        logging.info("JointStatePublisher::start_publishing: Starting publisher thread...")
        if not self.ros_client.is_connected:
            raise RuntimeError("JointStatePublisher: Cannot start publishing, ROS client not connected")
        if self.kill_event.is_set():
            self.kill_event.clear()
        self.thread = threading.Thread(target=self._publish)
        self.thread.start()
        logging.info("JointStatePublisher::start_publishing: Publisher thread started")

    def stop_publishing(self):
        logging.info("JointStatePublisher::stop_publishing: Stopping publisher thread...")
        if self.thread:
            self.kill_event.set()
            self.thread.join()
            self.thread = None
            logging.info("JointStatePublisher::stop_publishing: Publisher thread stopped")
        else:
            logging.info("JointStatePublisher::stop_publishing: Publisher thread not running")

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

    def __enter__(self):
        self.start_publishing()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_publishing()
