import time
import threading
import atexit

from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

from ..datastructures.pose import PoseStamped
from ..datastructures.enums import ExecutionType
from tf2_msgs.msg import TFMessage

from ..ros import  create_publisher
from ..ros import  Time


class TFBroadcaster:
    """
    Broadcaster that publishes TF frames for every object in the World.
    """
    def __init__(self, world: World,  projection_namespace=ExecutionType.SIMULATED, odom_frame="odom", interval=0.1):
        """
        The broadcaster prefixes all published TF messages with a projection namespace to distinguish between the TF
        frames from the simulation and the one from the real robot.

        :param projection_namespace: Name with which the TF frames should be prefixed
        :param odom_frame: Name of the statically published odom frame
        :param interval: Interval at which the TFs should be published, in seconds
        """
        self.world = world

        self.tf_static_publisher = create_publisher("/tf_static", TFMessage, queue_size=10)
        self.tf_publisher = create_publisher("/tf", TFMessage, queue_size=10)
        self.thread = threading.Thread(target=self._publish, daemon=True)
        self.kill_event = threading.Event()
        self.interval = interval

        # Namespaces
        self.projection_namespace = projection_namespace
        self.odom_frame = odom_frame

        self.thread.start()

        atexit.register(self._stop_publishing)

    def update(self):
        """
        Updates the TFs for the static odom frame and all objects currently in the World.
        """
        # Update static odom
        self._update_static_odom()
        # Update pose of objects which are possibly attached on the robot
        self._update_objects()

    def _update_objects(self) -> None:
        """
        Publishes the current pose of all objects in the World. As well as the poses of all links of these objects.
        """
        for body in self.world.bodies:
            pose = PoseStamped.from_spatial_type(body.global_pose)
            self._publish_pose(f"{body.name.prefix}/{body.name.name}", pose)

    def _update_static_odom(self) -> None:
        """
        Publishes a static odom frame to the tf_static topic.
        """
        self._publish_pose(self.odom_frame,
                           PoseStamped.from_list(self.world.root, [0, 0, 0], [0, 0, 0, 1]), static=True)

    def _publish_pose(self, child_frame_id: str, pose: PoseStamped, static=False) -> None:
        """
        Publishes the given pose to the ROS TF topic. First the pose is converted to a Transform between pose.frame and
        the given child_frame_id. Afterward, the frames of the Transform are prefixed with the projection namespace.

        :param child_frame_id: Name of the TF frame which the pose points to
        :param pose: Pose that should be published
        :param static: If the pose should be published to the tf_static topic
        """
        frame_id = pose.frame_id
        if frame_id != child_frame_id:
            tf_stamped = pose.to_transform_stamped(None)
            tf_stamped_msg = tf_stamped.ros_message()
            tf_stamped_msg.header.frame_id = self.projection_namespace.name + "/" + tf_stamped.header.frame_id.name.name
            tf_stamped_msg.child_frame_id = self.projection_namespace.name + "/" + tf_stamped.child_frame_id
            tf2_msg = TFMessage()
            tf2_msg.transforms.append(tf_stamped)
            if static:
                self.tf_static_publisher.publish(tf2_msg)
            else:
                self.tf_publisher.publish(tf2_msg)

    def _publish(self) -> None:
        """
        Constantly publishes the positions of all objects in the World.
        """
        while not self.kill_event.is_set():
            self.update()
            time.sleep(self.interval)

    def _stop_publishing(self) -> None:
        """
        Called when the process ends, sets the kill_event which terminates the thread that publishes to the TF topic.
        """
        self.kill_event.set()
        self.thread.join()
