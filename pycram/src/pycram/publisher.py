from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram import helper
from pycram.listener import transformer

import sys
from time import time
from threading import Thread, currentThread

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Transform, TransformStamped, Pose

# Global Variables
# Booleans if robot or robot poses should be published to /tf
publish_robot_base_pose = True
publish_objects_poses = True
publish_frequently = True

# Frame names of the map and odom frame
map_frame = "map"
odom_frame = "odom"

# Namespaces
projection_namespace = "projection"
kitchen_namespace = "iai_kitchen"


def publish_pose(frame_id, child_frame_id, pose):
    if not rospy.is_shutdown():
        pose = helper.ensure_pose(pose)
        tf = Transform(pose.position, pose.orientation)
        header = Header(0, rospy.Time(time()), frame_id)
        tf_stamped = TransformStamped(header, child_frame_id, tf)
        transformer.setTransform(tf_stamped)
        return True


def publish_object_pose(name, pose):
    """
    Publishes the given pose of the object of given name in reference to the map frame to tf.

    :type name: str
    :type pose: list or Pose
    """
    if name in robot_description.i.name:
        publish_robot_pose(pose)
    if publish_objects_poses and name not in robot_description.i.name:
        if not rospy.is_shutdown():
            pose = helper.ensure_pose(pose)
            if pose:
                return publish_pose(map_frame, projection_namespace + '/' + name, pose)
            else:
                rospy.logerr("(publisher) Could not publish given pose of %s since"
                             " it could not be converted to a Pose object.", name)
                return None


def publish_robot_pose(pose):
    "Publishes the base_frame of the robot in reference to the odom frame to tf."
    if publish_robot_base_pose:
        if not rospy.is_shutdown():
            robot_pose = helper.ensure_pose(pose)
            if robot_pose:
                return publish_pose(projection_namespace + '/' + odom_frame,
                                    projection_namespace + '/' + robot_description.i.base_frame,
                                    robot_pose)
            else:
                rospy.logerr("(publisher) Could not publish given pose of robot since"
                             " it could not be converted to a Pose object.")
                return None


if publish_frequently:

    if 'bullet_world' in sys.modules:
        rospy.logwarn("(publisher) Make sure that you are not loading this module from pycram.bullet_world.")

    from pycram.bullet_world import BulletWorld


    def object_pose_publisher(publish_rate=20):
        """
        Publishes the given pose of the object of given name in reference to the map frame to tf.

        :type name: str
        :type pose: list or Pose
        """
        if publish_objects_poses:
            rate = rospy.Rate(publish_rate)
            t = currentThread()
            while not rospy.is_shutdown() and getattr(t, "do_run", True):
                for obj in list(BulletWorld.current_bullet_world.objects):
                    if obj.name in robot_description.i.name:
                        published = publish_robot_pose(obj.get_position_and_orientation())
                    else:
                        published = publish_object_pose(obj.name, obj.get_position_and_orientation())
                    if not published:
                        rospy.logerr("(publisher) Could not publish given pose of %s.", obj.name)
                rate.sleep()
            return True


    def odom_T_base_frame_publisher(publish_rate=20):
        "Publishes the base_frame of the robot in reference to the odom frame to tf."
        if publish_robot_base_pose:
            rate = rospy.Rate(publish_rate)  # 20hz
            t = currentThread()
            while not rospy.is_shutdown() and getattr(t, "do_run", True):
                robot_pose = helper.ensure_pose(BulletWorld.robot.get_position_and_orientation())
                if robot_pose:
                    published = publish_robot_pose(robot_pose)
                    if not published:
                        rospy.logerr("(publisher) Could not publish given pose of robot.")
                    rate.sleep()
            return True


    def start_publishing(targets=None, names=None):
        if names is None:
            names = ["odom_T_base_frame_bullet_publisher_thread", "object_pose_publisher"]
        if targets is None:
            targets = [odom_T_base_frame_publisher, object_pose_publisher]
        ret = []
        for target, name in zip(targets, names):
            t = Thread(target=target, name=name)
            t.do_run = True
            t.start()
            ret.append(t)
        return ret


    def stop_publishing(threads):
        for t in threads:
            t.do_run = False
            t.join()
