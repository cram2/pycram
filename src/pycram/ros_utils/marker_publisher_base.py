import time
from abc import abstractmethod, ABC
from dataclasses import dataclass
from functools import cached_property
from typing import List, Optional

import numpy as np
from geometry_msgs.msg import Vector3, Point, Pose
from rclpy.publisher import Publisher
from semantic_digital_twin.world_description.world_entity import Body
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..datastructures.dataclasses import BoxVisualShape, CylinderVisualShape, MeshVisualShape, SphereVisualShape, \
    BoundingBox
from ..datastructures.pose import PoseStamped, TransformStamped
from ..ros import  Duration, Time
from ..ros import  create_publisher
from ..tf_transformations import quaternion_multiply


@dataclass
class MarkerPublisherBase(ABC):
    """
    Base class for publishing visualization markers of specific data in ROS/RViz.

    This class provides common utilities for creating and publishing various types of
    markers, such as arrows, cubes, lines, and geometry-based visualizations. Subclasses
    must implement the `visualize` method to define how their specific data is converted
    into `Marker` objects.
    """

    topic: str
    """The name of the topic to publish to"""

    @cached_property
    def publisher(self) -> Publisher:
        """
        Create a publisher that publishes data to topic.

        returns: The created publisher.
        """
        pub = create_publisher(self.topic, MarkerArray, queue_size=10)
        time.sleep(0.5)  # sync time
        return pub

    @abstractmethod
    def visualize(self, data: object) -> None:
        """
        Subclasses must implement this to create and publish a MarkerArray
        """
        pass

    def _create_arrow_marker(self, p1: Point, p2: Point, index: int, frame_id: str,
                             color: Optional[List] = (1.0, 0.0, 1.0, 1.0), scale: Optional[List] = (0.01, 0.02, 0.02),
                             duration: Optional[int] = 60) -> Marker:
        """
        Create an arrow marker.

        :param p1: first point of the arrow
        :param p2: second point of the arrow
        :param index: index of the created arrow
        :param frame_id: frame id of the created arrow marker
        :param color: color of the created arrow marker
        :param scale: scale of the created arrow marker
        :param duration: duration of visualizing the arrow

        returns: The created arrow marker.
        """

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.id = index
        marker.ns = f"arrow_marker_{marker.id}"
        marker.action = Marker.ADD
        marker.type = Marker.ARROW
        marker.lifetime = Duration(duration)

        marker.points.append(Point(x=p1.x, y=p1.y, z=p1.z))
        marker.points.append(Point(x=p2.x, y=p2.y, z=p2.z))

        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        return marker

    def _create_cube_marker(self, box: BoundingBox, index: int, duration: Optional[int] = 60) -> Marker:
        """
        Create a cube marker.

        :param box: the bounding box the cube marker should be drawn on
        :param index: index of the created cube
        :param duration: duration of visualizing the cube marker

        returns: The created cube marker.
        """

        origin = box.transform.position
        marker = Marker()
        marker.header.frame_id = box.transform.frame_id
        marker.id = index
        marker.ns = f"cube_marker_{marker.id}"
        marker.action = Marker.ADD
        marker.type = Marker.CUBE
        marker.lifetime = Duration(duration)

        marker.pose = Pose()
        marker.pose.position = Point(x=origin.x, y=origin.y, z=origin.z)

        marker.scale.x = box.depth
        marker.scale.y = box.width
        marker.scale.z = box.height

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5

        return marker

    def _create_line_marker(self, pose: PoseStamped, axis: List[float], color_rgba: List[float], index: int,
                            duration: Optional[int] = 60, length: Optional[float] = 0.1,
                            width: Optional[float] = 0.02) -> Marker:
        """
        Create a line marker.

        :param pose: the pose of the line
        :param axis: the given axis
        :param color_rgba: the color of the created line marker
        :param index: id of the created line marker
        :param duration: duration of visualizing the line marker
        :param length: length of the axis
        :param width: width of the axis

        returns: The created line marker.
        """
        def rotate_axis_by_quaternion(axis: List[float], quaternion: PoseStamped.orientation) -> List[float]:
            """
            Rotates axis by quaternion.

            :param axis: the axis to rotate
            :param quaternion: the quaternion the axis is rotated

            returns: Rotated axis
            """

            qx, qy, qz, qw = quaternion.to_list()
            axis_quat = (*axis, 0)
            q = (qx, qy, qz, qw)
            q_inverse = (-qx, -qy, -qz, qw)
            rotated_quat = quaternion_multiply(quaternion_multiply(q, axis_quat), q_inverse)
            return rotated_quat[:3]

        start_point = Point(x=pose.position.x, y=pose.position.y, z=pose.position.z)
        rotated = rotate_axis_by_quaternion(axis, pose.orientation)

        end_point = Point(
            x=start_point.x + rotated[0] * length,
            y=start_point.y + rotated[1] * length,
            z=start_point.z + rotated[2] * length,
        )

        marker = Marker()
        marker.header.frame_id = pose.frame_id
        marker.header.stamp = Time().now()
        marker.ns = f'axis_visualization_{index}'
        marker.id = index
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color = ColorRGBA(**dict(zip(["r", "g", "b", "a"], color_rgba)))
        marker.lifetime = Duration(duration)

        marker.points.append(start_point)
        marker.points.append(end_point)

        return marker

    def _create_geometry_marker(self, geom: MeshVisualShape | CylinderVisualShape | BoxVisualShape | SphereVisualShape,
                                obj: Body, link: str, i: int, link_pose_with_origin: TransformStamped, reference_frame: str,
                                use_prospection_world: Optional[bool] = False) -> Marker:
        """
        Creates a Marker for the given geometry type.

        :param geom: the geometry to create the marker for
        :param obj: the object to create the marker for
        :param link: the link to create the marker for
        :param i: the id of the created marker
        :param link_pose_with_origin: the pose of the link
        :param reference_frame: the reference frame of the link
        :param use_prospection_world: uif prospection world is used or not

        returns: the created marker.
        """
        marker = Marker()
        marker.header.frame_id = reference_frame
        marker.ns = obj.name
        marker.id = obj.link_name_to_id[link] * 10000 + i
        marker.action = Marker.ADD
        marker.lifetime = Duration(5)

        marker.pose = link_pose_with_origin.to_pose_stamped().pose.ros_message()

        color = obj.get_link_color(link).get_rgba()
        marker.color = ColorRGBA(**dict(zip(["r", "g", "b", "a"], color)))
        if use_prospection_world:
            marker.color.a = 0.5

        # Geometry-specific handling
        if isinstance(geom, MeshVisualShape):
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = "file://" + geom.file_name
            marker.scale = Vector3(**dict(zip(["x", "y", "z"], geom.scale or (1.0, 1.0, 1.0))))
            marker.mesh_use_embedded_materials = True
        elif isinstance(geom, CylinderVisualShape):
            marker.type = Marker.CYLINDER
            marker.scale = Vector3(x=geom.radius * 2, y=geom.radius * 2, z=geom.length)
        elif isinstance(geom, BoxVisualShape):
            marker.type = Marker.CUBE
            size = np.array(geom.size) * 2
            marker.scale = Vector3(x=float(size[0]), y=float(size[1]), z=float(size[2]))
        elif isinstance(geom, SphereVisualShape):
            marker.type = Marker.SPHERE
            marker.scale = Vector3(x=geom.radius * 2, y=geom.radius * 2, z=geom.radius * 2)

        return marker

    def _create_object_marker(self, pose: PoseStamped, name: str, path: str, current_id: int) -> Marker:
        """
        Create a marker for an object.

        :param pose: the pose of the object
        :param name: the name of the object
        :param path: the path of the object
        :param current_id: the id of the current object

        returns: The created marker.
        """
        marker = Marker()
        marker.id = current_id
        marker.header.frame_id = pose.header.frame_id
        marker.ns = name
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.mesh_resource = 'file://' + path

        return marker