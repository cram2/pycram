from .robot_description import InitializedRobotDescription as robot_description
from . import helper

import sys
if 'bullet_world' in sys.modules:
    logwarn("(publisher) Make sure that you are not loading this module from pycram.bullet_world.")
from .bullet_world import BulletWorld

import rospkg
import atexit
from threading import Thread, currentThread

from tf import TransformerROS, transformations
from rospy import Duration, Time, logerr, logwarn, Rate, is_shutdown
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped


# Boolean for publishing frequently poses/states
# Although frequently updating the local tf is tested and seems to work, there currently
# exist problems if the simulation is resetting (like currently for detection). During
# this "unstable" simulation process pose-querying from pybullet, seems to return non valid poses,
# which would destroy the validate local tf tree. Therefore, this mode is NOT activated.
# Instead to update the local tf tree, please call the memeber function update_local_transformer_from_btr()
# on the object local_transformer of class LocalTransformer.
publish_frequently = False

class LocalTransformer:
    """This class allows to use the TF class TransformerROS without using the ROS
    network system or the topic /tf, where transforms are usually published to.
    Instead a local transformer is saved and allows to publish local transforms,
    as well the use of TFs convenient lookup functions (see functions below).

    This class uses the robots (currently only one! supported) URDF file to
    initialize the tfs for the robot. Moreover, the function update_local_transformer_from_btr
    updates these tfs by copying the tfs state from the pybullet world."""

    def __init__(self, map_frame, odom_frame, projection_namespace, kitchen_namespace):
        # Frame names of the map and odom frame
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        # Namespaces
        self.projection_namespace = projection_namespace
        self.kitchen_namespace = kitchen_namespace
        # TF tf_stampeds and static_tf_stampeds of the Robot in the BulletWorld:
        # These are initialized with the function init_transforms_from_urdf and are
        # used to update the local transformer with the function update_local_transformer_from_btr
        self.tf_stampeds = []
        self.static_tf_stampeds = []
        self.local_transformer = TransformerROS(interpolate=True, cache_time=Duration(10.0))
        self.init_local_tf_with_tfs_from_urdf()

    def init_transforms_from_urdf(self):
        """
            This static function gets with the robot name in robot_description.py, the
            robots URDF file from the "resources" folder in the ROS "pycram" folder.
            All joints of the URDF are extracted and saved in a list of static and
            normal TFStamped.

            :returns: list of static_tf_stampeds, list of tf_stampeds
        """
        # Save joint translations and orientations in tf_stampeds
        # static_tf_stampeds saves only the static joint translations and orientations
        self.tf_stampeds = []
        self.static_tf_stampeds = []
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
                        roll = joint.origin.rpy[0]
                        pitch = joint.origin.rpy[1]
                        yaw = joint.origin.rpy[2]
                        rotation = transformations.quaternion_from_euler(roll, pitch, yaw)
                    else:
                        rotation = [0, 0, 0, 1]

                if joint.name in robot_description.i.odom_joints:
                    # since pybullet wont update this joints, these are declared as static
                    translation = [0, 0, 0]
                    rotation = [0, 0, 0, 1]

                # Wrap the joint attributes in a TFStamped and append it to static_tf_stamped if the joint was fixed
                tf_stamped = helper.list2tfstamped(source_frame, target_frame, [translation, rotation])
                if (joint.type and joint.type == 'fixed') or joint.name in robot_description.i.odom_joints:
                    self.static_tf_stampeds.append(tf_stamped)
                else:
                    self.tf_stampeds.append(tf_stamped)

    def init_local_tf_with_tfs_from_urdf(self):
        "This function initializes the local transformer with TFs from the robots URDF."
        self.init_transforms_from_urdf()
        for static_tfstamped in self.static_tf_stampeds:
            self.local_transformer._buffer.set_transform_static(static_tfstamped, "default_authority")
        for tfstamped in self.tf_stampeds:
            self.local_transformer.setTransform(tfstamped)

    def update_robot_state(self):
        robot = BulletWorld.robot
        if robot:
            # First publish (static) joint states to tf
            # Update tf stampeds which might have changed
            for tf_stamped in self.tf_stampeds:
                source_frame = tf_stamped.header.frame_id
                target_frame = tf_stamped.child_frame_id
                # Push calculated transformation to the local transformer
                p, q = robot.get_link_relative_to_other_link(source_frame.replace(self.projection_namespace + "/", ""),
                                                             target_frame.replace(self.projection_namespace + "/", ""))
                self.local_transformer.setTransform(helper.list2tfstamped(source_frame, target_frame, [p, q]))

            # Update the static transforms
            for static_tf_stamped in self.static_tf_stampeds:
                self.local_transformer._buffer.set_transform_static(static_tf_stamped, "default_authority")

    def update_robot_pose(self):
        robot = BulletWorld.robot
        if robot:
            # Then publish pose of robot
            try:
                robot_pose = BulletWorld.robot.get_link_position_and_orientation(robot_description.i.base_frame)
            except KeyError:
                robot_pose = BulletWorld.robot.get_position_and_orientation()
            self.publish_robot_pose(helper.ensure_pose(robot_pose))

    def update_robot(self):
        self.update_robot_state()
        self.update_robot_pose()

    def update_objects(self):
        if BulletWorld.current_bullet_world:
            for obj in list(BulletWorld.current_bullet_world.objects):
                if obj.name == robot_description.i.name or obj.type == "environment":
                    continue
                else:
                    published = local_transformer.publish_object_pose(obj.name,
                                                                      obj.get_position_and_orientation())
                if not published:
                    logerr("(publisher) Could not publish given pose of %s.", obj.name)

    def update_static_odom(self):
        self.publish_pose(self.map_frame, self.projection_namespace + '/' + self.odom_frame,
                          [[0, 0, 0], [0, 0, 0, 1]], static=True)

    def update_from_btr(self):
        # Update static odom
        self.update_static_odom()
        # Update pose and state of robot
        self.update_robot()
        # Update pose of objects which are possibly attached on the robot
        self.update_objects()

    ############################################################################################################
    ######################################### Query Local Transformer ##########################################
    ############################################################################################################

    def tf_pose_transform(self, source_frame, target_frame, pose, time=None):
        tf_pose = helper.ensure_pose(pose)
        tf_time = time if time else self.local_transformer.getLatestCommonTime(source_frame, target_frame)
        tf_pose_stamped = PoseStamped(Header(0, tf_time, source_frame), tf_pose)
        return self.tf_pose_stamped_transform(target_frame, tf_pose_stamped)

    def tf_pose_stamped_transform(self, target_frame, pose_stamped):
        return helper.pose_stamped2tuple(self.local_transformer.transformPose(target_frame, pose_stamped))

    def tf_transform(self, source_frame, target_frame, time=None):
        tf_time = time if time else self.local_transformer.getLatestCommonTime(source_frame, target_frame)
        return self.local_transformer.lookupTransform(source_frame, target_frame, tf_time)

    ############################################################################################################
    ####################################### Publish to Local Transformer ###########'###########################
    ############################################################################################################

    def publish_pose(self, frame_id, child_frame_id, pose, static=None):
        if frame_id != child_frame_id:
            pose = helper.ensure_pose(pose)
            tf_stamped = helper.pose2tfstamped(frame_id, child_frame_id, pose)
            if tf_stamped:
                if static:
                    self.local_transformer._buffer.set_transform_static(tf_stamped, "default_authority")
                else:
                    self.local_transformer.setTransform(tf_stamped)
                return True

    def publish_object_pose(self, name, pose):
        """
        Publishes the given pose of the object of given name in reference to the map frame to tf.

        :type name: str
        :type pose: list or Pose
        """
        if name in robot_description.i.name:
            return None
        if name not in robot_description.i.name:
            pose = helper.ensure_pose(pose)
            if pose:
                tf_name = self.projection_namespace + '/' + name if self.projection_namespace else name
                return self.publish_pose(self.map_frame, tf_name, pose)
            else:
                logerr("(publisher) Could not publish given pose of %s since"
                       " it could not be converted to a Pose object.", name)
                return None

    def publish_robot_pose(self, pose):
        "Publishes the base_frame of the robot in reference to the map frame to tf."
        robot_pose = helper.ensure_pose(pose)
        if robot_pose:
            tf_base_frame = self.projection_namespace + '/' + robot_description.i.base_frame \
                if self.projection_namespace \
                else robot_description.i.base_frame
            return self.publish_pose(self.map_frame, tf_base_frame, robot_pose)
        else:
            logerr("(publisher) Could not publish given pose of robot since"
                   " it could not be converted to a Pose object.")
            return None

    def publish_robots_link_pose(self, link, pose):
        "Publishes the frame link of the robot in reference to the base_frame of the robot to tf."
        robot_pose = helper.ensure_pose(pose)
        if robot_pose:
            tf_base_frame = self.projection_namespace + '/' + robot_description.i.base_frame \
                if self.projection_namespace \
                else robot_description.i.base_frame
            tf_link_frame = self.projection_namespace + '/' + link \
                if self.projection_namespace \
                else link
            if not tf_base_frame in tf_link_frame:
                return self.publish_pose(tf_base_frame, tf_link_frame, robot_pose)
        else:
            logerr("(publisher) Could not publish given pose of robot since"
                   " it could not be converted to a Pose object.")
            return None

# Initializing Local Transformer using ROS data types
local_transformer = LocalTransformer("map", "odom", "projection", "iai_kitchen")

############################################################################################################
################################### Frequently Publish To Local Transformer ################################
############################################################################################################

if publish_frequently:


    class LocalTransformerFreqPublisher:

        """
        This class publishes frequently poses of objects and the robot base poses as well its joint states into
        the local transformer.
        """

        def __init__(self, publish_robot_base_pose=None, publish_objects_poses=None, publish_robot_state=None,
                     publisher_funs=None, publisher_fun_names=None):
            """This function allows to specify which object poses or states of objects should be published by using
            the parameters. If None, everything below is published with information from the bullet world:

            - robots base pose (map -> base_frame)
            - poses of objects (map -> pos of the objects center of mass)
            - robots joint states (joint_i_source -> joint_i_target)
            - static odom publisher (map -> odom is the identity pose)

            The functions in publisher_funs are then each ran in a separate thread with the given name in
            publisher_fun_names. For implementing other frequent publisher functions, please take a look at
            the other functions below.
            """

            # Booleans for publishing frequently poses of robot base, object pose, robot states, static odom
            self.publish_robot_base_pose = publish_robot_base_pose if publish_robot_base_pose else False
            self.publish_objects_poses = publish_objects_poses if publish_objects_poses else True
            self.publish_robot_state = publish_robot_state if publish_robot_state else False
            self.publish_static_odom = True
            # Functions and threads for publishing above poses
            if publisher_fun_names is None:
                self.publisher_fun_names = ["local_odom_T_base_frame_bullet_publisher_thread", "local_object_pose_publisher",
                                            "local_robot_state_publisher", "local_static_odom_publisher"]
            if publisher_funs is None:
                self.publisher_funs = [self.local_robot_pose_publisher, self.local_object_pose_publisher,
                                       self.local_robot_state_publisher, self.local_static_odom_publisher]
            self.threads = []
            self.start_publishing()
            # Call stop_publishing if python environment is closed
            atexit.register(self.stop_publishing)

        def local_object_pose_publisher(self, publish_rate=20):
            """
            Publishes the given pose of the object of given name in reference to the map frame to tf.

            :type name: str
            :type pose: list or Pose
            """
            if self.publish_objects_poses:
                rate = Rate(publish_rate)
                t = currentThread()
                while not is_shutdown() and getattr(t, "do_run", True):
                    local_transformer.update_objects()
                    rate.sleep()
                return True

        def local_robot_pose_publisher(self, publish_rate=20):
            "Publishes the base_frame of the robot in reference to the odom frame to tf."
            if self.publish_robot_base_pose:
                rate = Rate(publish_rate)  # 20hz
                t = currentThread()
                while not is_shutdown() and getattr(t, "do_run", True):
                    local_transformer.update_robot_pose()
                    rate.sleep()
                return True

        def local_robot_state_publisher(self, publish_rate=20):
            if self.publish_robot_state:
                rate = Rate(publish_rate)  # 20hz
                t = currentThread()
                while not is_shutdown() and getattr(t, "do_run", True):
                    local_transformer.update_robot_state()
                    rate.sleep()
                return True

        def local_static_odom_publisher(self, publish_rate=20):
            if self.publish_static_odom:
                rate = Rate(publish_rate)  # 20hz
                t = currentThread()
                while not is_shutdown() and getattr(t, "do_run", True):
                    published = local_transformer.publish_pose(local_transformer.map_frame,
                                                               local_transformer.projection_namespace +
                                                               '/' +
                                                               local_transformer.odom_frame,
                                                               [[0, 0, 0], [0, 0, 0, 1]],
                                                               static=True)
                    if not published:
                        logerr("(publisher) Could not publish static map to odom tf.")
                    rate.sleep()
                return True

        def start_publishing(self):
            ret = []
            for target, name in zip(self.publisher_funs, self.publisher_fun_names):
                t = Thread(target=target, name=type(self).__name__ + ":" + name)
                t.do_run = True
                t.start()
                ret.append(t)
            self.threads = ret

        def stop_publishing(self):
            for t in self.threads:
                t.do_run = False
                t.join()


    freq_local_transformer = LocalTransformerFreqPublisher()
