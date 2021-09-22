import threading
import time

from pytransform3d.rotations import quaternion_xyzw_from_wxyz, quaternion_wxyz_from_xyzw, quaternion_from_matrix, \
    active_matrix_from_extrinsic_euler_xzx
from pytransform3d.transformations import transform_from_pq, pq_from_transform

from pycram.robot_description import InitializedRobotDescription as robot_description
from ros.ros_topic_publisher import ROSTopicPublisher

import roslibpy

import rospkg
import numpy as np

from urdf_parser_py.urdf import URDF

from ros.rosbridge import ros_client


class Pose(object):
    def __init__(self, mat):
        self.mat = mat

    def pos(self) -> np.ndarray:
        return pq_from_transform(self.mat)[:3]

    def ori_qxyzw(self) -> np.ndarray:
        return quaternion_xyzw_from_wxyz(self.ori_qwxyz())

    def ori_qwxyz(self) -> np.ndarray:
        return pq_from_transform(self.mat)[3:]

    @staticmethod
    def from_xyz_qxyzw(trans: np.ndarray, quat: np.ndarray):
        return Pose.from_xyz_qwxyz(trans, quaternion_wxyz_from_xyzw(quat))

    @staticmethod
    def from_xyz_qwxyz(trans: np.ndarray, quat: np.ndarray):
        return Pose(transform_from_pq(np.concatenate((trans, quat))))


class TFBroadcaster(ROSTopicPublisher):
    def __init__(self, bullet_world, map_frame, odom_frame, projection_namespace, kitchen_namespace, interval=0.1):
        super().__init__()
        self.world = bullet_world

        self.tf_static_publisher = roslibpy.Topic(ros_client, "/tf_static", "geometry_msgs/TransformStamped")
        self.tf_publisher = roslibpy.Topic(ros_client, "/tf", "geometry_msgs/TransformStamped")
        self.thread = None
        self.kill_event = threading.Event()
        self.interval = interval

        # Frame names of the map and odom frame
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        # Namespaces
        self.projection_namespace = projection_namespace
        self.kitchen_namespace = kitchen_namespace

        self.tf_stampeds = []
        self.static_tf_stampeds = []
        self.init_transforms_from_urdf()

    @staticmethod
    def make_tf_stamped(source_frame: str, target_frame: str, trans: np.ndarray, quat_xyzw: np.ndarray) -> roslibpy.Message:
        """
        http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/TransformStamped.html
        """
        return roslibpy.Message({
            "header": roslibpy.Header(0, roslibpy.Time.now(), source_frame),
            "child_frame_id": target_frame,
            "transform": {
                "translation": dict(zip(["x", "y", "z"], trans.tolist())),
                "rotation": dict(zip(["x", "y", "z", "w"], quat_xyzw.tolist()))
            }
        })

    def init_transforms_from_urdf(self):
        """
            This static function gets with the robot name in robot_description.py, the
            robots URDF file from the "resources" folder in the ROS "pycram" folder.
            All joints of the URDF are extracted. The TransformerROS is updated with a list of static and
            normal TFStamped.
        """
        # Get URDF file path
        robot_name = robot_description.i.name
        rospack = rospkg.RosPack()
        filename = rospack.get_path('pycram') + '/resources/' + robot_name + '.urdf'
        # ... and open it
        with open(filename) as f:
            s = f.read()
            robot = URDF.from_xml_string(s)
            # Save for every joint in the robot the source_frame, target_frame,
            # aswell the joints origin, where the translation xyz and rotation rpy
            # are saved
            for joint in robot.joints:

                source_frame = self.projection_namespace + '/' + joint.parent
                target_frame = self.projection_namespace + '/' + joint.child

                if joint.origin:
                    if joint.origin.xyz:
                        translation = joint.origin.xyz
                    else:
                        translation = [0, 0, 0]
                    if joint.origin.rpy:
                        rotation = quaternion_from_matrix(active_matrix_from_extrinsic_euler_xzx(joint.origin.rpy))
                    else:
                        rotation = [0, 0, 0, 1]

                if joint.name in robot_description.i.odom_joints:
                    # since pybullet wont update this joints, these are declared as static
                    translation = [0, 0, 0]
                    rotation = [0, 0, 0, 1]

                # Wrap the joint attributes in a TFStamped and append it to static_tf_stamped if the joint was fixed
                tf_stamped_msg = self.make_tf_stamped(source_frame, target_frame, np.array(translation), np.array(rotation))
                if (joint.type and joint.type == 'fixed') or joint.name in robot_description.i.odom_joints:
                    self.tf_static_publisher.publish(tf_stamped_msg)
                    self.static_tf_stampeds.append(tf_stamped_msg)
                else:
                    self.tf_publisher.publish(tf_stamped_msg)
                    self.tf_stampeds.append(tf_stamped_msg)

    def update(self):
        # Update static odom
        self._update_static_odom()
        # Update pose and state of robot
        self._update_robot()
        # Update pose of objects which are possibly attached on the robot
        self._update_objects()

    def _update_robot(self):
        self._update_robot_state()
        self._update_robot_pose()

    def _update_objects(self):
        for obj in self.world.objects:
            if obj.name == robot_description.i.name or obj.type == "environment":
                continue
            else:
                pb_pos, pb_ori = obj.get_position_and_orientation()
                pose = Pose.from_xyz_qxyzw(np.array(pb_pos), np.array(pb_ori))
                self._publish_object_pose(obj.name, pose)

    def _update_static_odom(self):
        self._publish_pose(self.map_frame, self.projection_namespace + '/' + self.odom_frame,
                           Pose.from_xyz_qxyzw(np.array([0, 0, 0]), np.array([0, 0, 0, 1])), static=True)

    def _update_robot_state(self):
        robot = self.world.robot
        if robot:
            # First publish (static) joint states to tf
            # Update tf stampeds which might have changed
            for tf_stamped in self.tf_stampeds:
                source_frame = tf_stamped.data["header"].data["frame_id"]
                target_frame = tf_stamped.data["child_frame_id"]
                # Push calculated transformation to the local transformer
                p, q = robot.get_link_relative_to_other_link(source_frame.replace(self.projection_namespace + "/", ""),
                                                             target_frame.replace(self.projection_namespace + "/", ""))
                self.tf_publisher.publish(self.make_tf_stamped(source_frame, target_frame, np.array(p), np.array(q)))

            # Update the static transforms
            for static_tf_stamped in self.static_tf_stampeds:
                self.tf_static_publisher.publish(static_tf_stamped)

    def _update_robot_pose(self):
        robot = self.world.robot
        if robot:
            # Then publish pose of robot
            try:
                pb_pos, pb_ori = robot.get_link_position_and_orientation(robot_description.i.base_frame)
            except KeyError:
                pb_pos, pb_ori = robot.get_position_and_orientation()
            robot_pose = Pose.from_xyz_qxyzw(pb_pos, pb_ori)
            self._publish_robot_pose(robot_pose)

    def _publish_pose(self, frame_id: str, child_frame_id: str, pose: Pose, static=False):
        if frame_id != child_frame_id:
            tf_stamped = self.make_tf_stamped(frame_id, child_frame_id, pose.pos(), pose.ori_qxyzw())
            if static:
                self.tf_static_publisher.publish(tf_stamped)
            else:
                self.tf_publisher.publish(tf_stamped)

    def _publish_object_pose(self, obj_name: str, pose: Pose):
        """
        Publishes the given pose of the object of given name in reference to the map frame to tf.

        :type obj_name: str
        :type pose: list or Pose
        """
        if obj_name in robot_description.i.name:
            return
        tf_name = self.projection_namespace + '/' + obj_name if self.projection_namespace else obj_name
        self._publish_pose(self.map_frame, tf_name, pose)

    def _publish_robot_pose(self, pose: Pose):
        "Publishes the base_frame of the robot in reference to the map frame to tf."
        tf_base_frame = self.projection_namespace + '/' + robot_description.i.base_frame \
            if self.projection_namespace \
            else robot_description.i.base_frame
        self._publish_pose(self.map_frame, tf_base_frame, pose)

    def publish_robots_link_pose(self, link: str, pose: Pose):
        "Publishes the frame link of the robot in reference to the base_frame of the robot to tf."
        tf_base_frame = self.projection_namespace + '/' + robot_description.i.base_frame \
            if self.projection_namespace \
            else robot_description.i.base_frame
        tf_link_frame = self.projection_namespace + '/' + link \
            if self.projection_namespace \
            else link
        if not tf_base_frame in tf_link_frame:
            self._publish_pose(tf_base_frame, tf_link_frame, pose)

    def _publish(self):
        while not self.kill_event.is_set():
            self.update()
            time.sleep(self.interval)
