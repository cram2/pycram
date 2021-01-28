from pycram.robot_description import InitializedRobotDescription as robot_description
import pycram.helper as helper
import pybullet as p

import sys
from threading import Thread, currentThread
from time import sleep
from time import time as current_time

from tf import TransformerROS, transformations
from rospy import Duration, Time, logerr, logwarn, Rate, is_shutdown
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

local_transformer = TransformerROS(interpolate=True, cache_time=Duration(10.0))
# Wait "waiting_time" seconds before lookup of transform or transforming of a pose
waiting_time = 0
# Timeout in seconds for lookup of transform and transforming of a pose
timeout = 5

# Global Variables
# Booleans if robot or robot poses should be published to /tf
publish_robot_base_pose = True
publish_objects_poses = True
publish_frequently = True
publish_robot_state = True
publish_static_odom = True

# Frame names of the map and odom frame
map_frame = "map"
odom_frame = "odom"

# Namespaces
projection_namespace = "projection"
kitchen_namespace = "iai_kitchen"

tf_stampeds = []
static_tf_stampeds = []


def get_init_transforms_from_urdf():
    with open('/home/thomas/pycram_ros/src/pycram/resources/donbot.urdf') as f:
        s = f.read()
        robot = URDF.from_xml_string(s)
        for joint in robot.joints:
            source_frame = projection_namespace + '/' + joint.parent
            target_frame = projection_namespace + '/' + joint.child
            if source_frame is target_frame or odom_frame in joint.name:
                continue
            if joint.origin:
                if joint.origin.xyz:
                    translation = joint.origin.xyz
                else:
                    translation = [0, 0, 0]
                if joint.origin.rpy:
                    roll = joint.origin.rpy[0]
                    pitch = joint.origin.rpy[1]
                    yaw = joint.origin.rpy[2]
                    rotation = transformations.quaternion_from_euler(roll, pitch, yaw)
                else:
                    rotation = [0, 0, 0, 1]
            tf_stamped = helper.list2tfstamped(source_frame, target_frame, [translation, rotation])
            if joint.type and joint.type == 'fixed':
                static_tf_stampeds.append(tf_stamped)
            else:
                tf_stampeds.append(tf_stamped)
    tf_stampeds.append(helper.list2tfstamped(map_frame, projection_namespace + '/' + odom_frame, [[0, 0, 0], [0, 0, 0, 1]]))
    return static_tf_stampeds, tf_stampeds


def update_local_transformer_from_btr():
    robot = BulletWorld.robot
    if robot:
        # Update tf stampeds which might have changed
        for tf_stamped in tf_stampeds:
            source_frame = tf_stamped.header.frame_id
            target_frame = tf_stamped.child_frame_id

            if 'odom' in source_frame or 'odom' in target_frame:
                # since the robots base_footprint = odom
                continue

            # Create Transformation Matrix from map to source frame
            map_T_source = robot.get_link_position_and_orientation(source_frame.replace(projection_namespace + '/', ""))
            map_T_source_trans = map_T_source[0]
            map_T_source_rot = map_T_source[1]
            map_T_source_m = transformations.concatenate_matrices(
                transformations.translation_matrix(map_T_source_trans),
                transformations.quaternion_matrix(map_T_source_rot))
            # Create Transformation Matrix from map to target frame
            map_T_target = robot.get_link_position_and_orientation(target_frame.replace(projection_namespace + '/', ""))
            map_T_target_trans = map_T_target[0]
            map_T_target_rot = map_T_target[1]
            map_T_target_m = transformations.concatenate_matrices(
                transformations.translation_matrix(map_T_target_trans),
                transformations.quaternion_matrix(map_T_target_rot))
            # Calculate Transformation from source to target frame
            source_T_map_m = transformations.inverse_matrix(map_T_source_m)
            source_T_target_m = transformations.concatenate_matrices(source_T_map_m, map_T_target_m)
            source_T_target_trans = transformations.translation_from_matrix(source_T_target_m)
            source_T_target_rot = transformations.quaternion_from_matrix(source_T_target_m)
            source_T_target = helper.list2tfstamped(source_frame, target_frame,
                                                    [source_T_target_trans, source_T_target_rot])
            # Push calculated transformation to the local transformer
            local_transformer.setTransform(source_T_target)
        # Update the static transforms
        for static_tf_stamped in static_tf_stampeds:
            local_transformer._buffer.set_transform_static(static_tf_stamped, "default_authority")


############################################################################################################
######################################## Update Local Transformer ##########################################
############################################################################################################


def tf_pose_transform(source_frame, target_frame, pose, time=None):
    # sleep(1)
    tf_pose = helper.ensure_pose(pose)
    tf_time = time if time else Time(0)
    tf_pose_stamped = PoseStamped(Header(0, tf_time, source_frame), tf_pose)
    return tf_pose_stamped_transform(target_frame, tf_pose_stamped)


def tf_pose_stamped_transform(target_frame, pose_stamped):
    # sleep(1)
    return helper.pose_stamped2tuple(local_transformer.transformPose(target_frame, pose_stamped))


def tf_transform(source_frame, target_frame, time=None):
    # sleep(1)
    tf_time = time if time else Time(0)
    return local_transformer.lookupTransform(source_frame, target_frame, tf_time)


def init_local_tf_with_tfs_from_urdf():
    static_tfstampeds, tfstampeds = get_init_transforms_from_urdf()
    for static_tfstamped in static_tfstampeds:
        local_transformer._buffer.set_transform_static(static_tfstamped, "default_authority")
    for tfstamped in tfstampeds:
        local_transformer.setTransform(tfstamped)


init_local_tf_with_tfs_from_urdf()


############################################################################################################
####################################### Publish to Local Transformer ###########'###########################
############################################################################################################

def publish_pose(frame_id, child_frame_id, pose):
    if frame_id is not child_frame_id:
        pose = helper.ensure_pose(pose)
        tf_stamped = helper.pose2tfstamped(frame_id, child_frame_id, pose)
        if tf_stamped:
            local_transformer.setTransform(tf_stamped)
            return True


def publish_object_pose(name, pose):
    """
    Publishes the given pose of the object of given name in reference to the map frame to tf.

    :type name: str
    :type pose: list or Pose
    """
    if name in robot_description.i.name:
        return None
    if publish_objects_poses and name not in robot_description.i.name:
        pose = helper.ensure_pose(pose)
        if pose:
            tf_name = projection_namespace + '/' + name if projection_namespace else name
            return publish_pose(map_frame, tf_name, pose)
        else:
            logerr("(publisher) Could not publish given pose of %s since"
                   " it could not be converted to a Pose object.", name)
            return None


def publish_robot_pose(pose):
    "Publishes the base_frame of the robot in reference to the odom frame to tf."
    if publish_robot_base_pose:
        robot_pose = helper.ensure_pose(pose)
        if robot_pose:
            tf_odom_frame = projection_namespace + '/' + odom_frame \
                if projection_namespace \
                else odom_frame
            tf_base_frame = projection_namespace + '/' + robot_description.i.base_frame \
                if projection_namespace \
                else robot_description.i.base_frame
            return publish_pose(tf_odom_frame, tf_base_frame, robot_pose)
        else:
            logerr("(publisher) Could not publish given pose of robot since"
                   " it could not be converted to a Pose object.")
            return None


def publish_robots_link_pose(link, pose):
    "Publishes the frame link of the robot in reference to the base_frame of the robot to tf."
    if publish_robot_base_pose:
        robot_pose = helper.ensure_pose(pose)
        if robot_pose:
            tf_base_frame = projection_namespace + '/' + robot_description.i.base_frame \
                if projection_namespace \
                else robot_description.i.base_frame
            tf_link_frame = projection_namespace + '/' + link \
                if projection_namespace \
                else link
            if not tf_base_frame in tf_link_frame:
                return publish_pose(tf_base_frame, tf_link_frame, robot_pose)
        else:
            logerr("(publisher) Could not publish given pose of robot since"
                   " it could not be converted to a Pose object.")
            return None


############################################################################################################
################################### Frequently Publish To Local Transformer ################################
############################################################################################################

if publish_frequently:

    if 'bullet_world' in sys.modules:
        logwarn("(publisher) Make sure that you are not loading this module from pycram.bullet_world.")

    from pycram.bullet_world import BulletWorld


    def local_object_pose_publisher(publish_rate=60):
        """
        Publishes the given pose of the object of given name in reference to the map frame to tf.

        :type name: str
        :type pose: list or Pose
        """
        if publish_objects_poses:
            rate = Rate(publish_rate)
            t = currentThread()
            while not is_shutdown() and getattr(t, "do_run", True):
                for obj in list(BulletWorld.current_bullet_world.objects):
                    if obj.name in robot_description.i.name:
                        published = publish_robot_pose(obj.get_position_and_orientation())
                    else:
                        published = publish_object_pose(obj.name, obj.get_position_and_orientation())
                    if not published:
                        logerr("(publisher) Could not publish given pose of %s.", obj.name)
                rate.sleep()
            return True


    def local_odom_T_base_frame_publisher(publish_rate=60):
        "Publishes the base_frame of the robot in reference to the odom frame to tf."
        if publish_robot_base_pose:
            rate = Rate(publish_rate)  # 20hz
            t = currentThread()
            while not is_shutdown() and getattr(t, "do_run", True):
                robot_pose = helper.ensure_pose(BulletWorld.robot.get_position_and_orientation())
                if robot_pose:
                    published = publish_robot_pose(robot_pose)
                    if not published:
                        logerr("(publisher) Could not publish given pose of robot.")
                    rate.sleep()
            return True


    def local_robot_state_publisher(publish_rate=60):
        if publish_robot_state:
            rate = Rate(publish_rate)  # 20hz
            t = currentThread()
            while not is_shutdown() and getattr(t, "do_run", True):
                robot = BulletWorld.robot
                robot_links = robot.links
                world_T_robot_pose = robot.get_position_and_orientation()
                robot_T_world_pose = p.invertTransform(world_T_robot_pose[0], world_T_robot_pose[1])
                for robot_link in robot_links:
                    if robot_description.i.base_frame in robot_link:
                        continue
                    world_T_link_pose = robot.get_link_position_and_orientation(robot_link)
                    robot_T_link_pose = p.multiplyTransforms(robot_T_world_pose[0], robot_T_world_pose[1],
                                                             world_T_link_pose[0], world_T_link_pose[1],
                                                             robot.world.client_id)
                    if robot_T_link_pose:
                        published = publish_robots_link_pose(robot_link, robot_T_link_pose)
                        if not published:
                            logerr("(publisher) Could not publish given link of robot.")
                        rate.sleep()
            return True


    def local_static_odom_publisher(publish_rate=60):
        if publish_static_odom:
            rate = Rate(publish_rate)  # 20hz
            t = currentThread()
            while not is_shutdown() and getattr(t, "do_run", True):
                published = publish_pose(map_frame, projection_namespace + '/' + odom_frame, [[0, 0, 0], [0, 0, 0, 1]])
                if not published:
                    logerr("(publisher) Could not publish static map to odom tf.")
                rate.sleep()
            return True


    def start_publishing(targets=None, names=None):
        if names is None:
            names = ["local_odom_T_base_frame_bullet_publisher_thread", "local_object_pose_publisher",
                     "local_robot_state_publisher", "local_static_odom_publisher"]
        if targets is None:
            targets = [local_odom_T_base_frame_publisher, local_object_pose_publisher,
                       local_robot_state_publisher, local_static_odom_publisher]
        ret = []
        for target, name in zip(targets, names):
            t = Thread(target=target, name=name)
            t.do_run = True
            t.start()
            ret.append(t)
        sleep(1)
        return ret


    def stop_publishing(threads):
        for t in threads:
            t.do_run = False
            t.join()
