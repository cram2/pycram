import atexit
import threading
import time
from abc import abstractmethod, ABC
from functools import cached_property
from typing import List, Optional

import numpy as np
from geometry_msgs.msg import Vector3, Point, Pose
from rclpy.publisher import Publisher
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..datastructures.dataclasses import BoxVisualShape, CylinderVisualShape, MeshVisualShape, SphereVisualShape, \
    BoundingBoxCollection, Color, BoundingBox
from ..datastructures.enums import AxisIdentifier
from ..datastructures.pose import PoseStamped, TransformStamped
from ..datastructures.world import World
from ..designator import ObjectDesignatorDescription
from ..ros import  Duration, Time
from ..ros import  loginfo, logwarn, logerr
from ..ros import  create_publisher
from ..ros import  sleep
from ..tf_transformations import quaternion_multiply
from ..world_concepts.world_object import Object


class SingleTypePublisher(ABC):
    """
    Publishes a single type of data to visualize it in rviz.
    """

    def __init__(self, topic: str):
        """
        Initializes the name of the topic.

        :param topic: The topic to publish to.
        """

        self.topic = topic

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
                                obj: Object, link: str, i: int, link_pose_with_origin: TransformStamped, reference_frame: str,
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

class VizMarkerPublisher(SingleTypePublisher):
    """
    Publishes an Array of visualization marker which represent the situation in the World
    """

    def __init__(self, topic="/pycram/viz_marker", interval=0.1, reference_frame="map", use_prospection_world=False,
                 publish_visuals=False):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
        World. This Array is published with a rate of interval.

        :param topic: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        :param reference_frame: The reference frame of the visualization marker.
        :param use_prospection_world: If True, the visualization marker will be published for the prospection world.
        :param publish_visuals: If True, the visualization marker will be published.
        """

        self.use_prospection_world = use_prospection_world
        super().__init__(topic="/pycram/prospection_viz_marker" if use_prospection_world else topic)
        self.interval = interval
        self.reference_frame = reference_frame
        self.publish_visuals = publish_visuals

        if self.use_prospection_world:
            self.main_world = World.current_world.prospection_world
        else:
            self.main_world = World.current_world if not World.current_world.is_prospection_world else World.current_world.world_sync.world

        self.lock = self.main_world.object_lock
        self.kill_event = threading.Event()
        self.thread = threading.Thread(target=self.visualize)
        self.thread.start()
        atexit.register(self._stop_publishing)

    def _get_data(self) -> List[tuple[MeshVisualShape | CylinderVisualShape | BoxVisualShape | SphereVisualShape,
                            Object, str, int, TransformStamped]]:
        """
        Collects all geometry + pose data from the current world.
        Returns a list of tuples: (geom, obj, link, i, link_pose_with_origin)
        """
        result = []
        for obj in self.main_world.objects:
            if obj.name == "floor":
                continue
            for link in obj.link_name_to_id.keys():
                geoms = obj.get_link_visual_geometry(link) if self.publish_visuals else obj.get_link_geometry(link)
                if not isinstance(geoms, list):
                    geoms = [geoms]

                # filter out None values immediately
                geoms = [g for g in geoms if g is not None]
                if not geoms:
                    continue

                link_pose = obj.get_link_transform(link)
                link_origin = obj.get_link_origin_transform(link) if obj.get_link_origin(
                    link) else TransformStamped.from_list()
                link_pose_with_origin = link_pose * link_origin

                for i, geom in enumerate(geoms):
                    result.append((geom, obj, link, i, link_pose_with_origin))
        return result

    def _make_marker_array(self, data: List[tuple[MeshVisualShape | CylinderVisualShape | BoxVisualShape | SphereVisualShape,
                                        Object, str, int, TransformStamped]]) -> MarkerArray:
        """
        Build MarkerArray from prepared link data.
        If no data is given, automatically gather it from the world.
        """
        marker_array = MarkerArray()
        for geom, obj, link, i, link_pose_with_origin in data:
            msg = self._create_geometry_marker(
                geom, obj, link, i, link_pose_with_origin,
                self.reference_frame, self.use_prospection_world
            )
            marker_array.markers.append(msg)
        return marker_array


    def visualize(self, data: List[tuple[MeshVisualShape | CylinderVisualShape | BoxVisualShape | SphereVisualShape,
                            Object, str, int, TransformStamped]] = None) -> None:
        while not self.kill_event.is_set():
            self.lock.acquire()
            data = self._get_data()
            marker_array = self._make_marker_array(data)
            self.lock.release()
            self.publisher.publish(marker_array)
            time.sleep(self.interval)

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()


class ManualMarkerPublisher(SingleTypePublisher):
    """
    Class to manually add and remove marker of objects and poses.
    """

    def __init__(self, topic: str = '/pycram/manual_marker', interval: float = 0.1):
        """
        The Publisher creates an Array of Visualization marker with a marker for a pose or object.
        This Array is published with a rate of interval.

        :param topic: Name of the marker topic
        :param interval: Interval at which the marker should be published
        """
        super().__init__(topic=topic)
        self.start_time = None
        self.marker_array = MarkerArray()
        self.marker_overview = {}
        self.current_id = 0
        self.interval = interval

    def visualize(self, pose: PoseStamped, color: Optional[List] = None, bw_object: Optional[ObjectDesignatorDescription] = None,
                name: Optional[str] = None) -> None:
        """
        Publish a pose or an object into the MarkerArray.
        Priorities to add an object if possible

        :param pose: Pose of the marker
        :param color: Color of the marker if no object is given
        :param bw_object: Object to add as a marker
        :param name: Name of the marker
        """

        if color is None:
            color = [1, 0, 1, 1]

        self.start_time = time.time()
        thread = threading.Thread(target=self._publish, args=(pose, bw_object, name, color))
        thread.start()
        thread.join()

    def _publish(self, pose: PoseStamped, bw_object: Optional[ObjectDesignatorDescription] = None, name: Optional[str] = None,
                 color: Optional[List] = None) -> None:
        """
        Publish the marker into the MarkerArray
        """
        stop_thread = False
        duration = 2

        while not stop_thread:
            if time.time() - self.start_time > duration:
                stop_thread = True
            if bw_object is None:
                self._publish_pose(name=name, pose=pose, color=color)
            else:
                self._publish_object(name=name, pose=pose, bw_object=bw_object)

            sleep(self.interval)

    def _publish_pose(self, name: str, pose: PoseStamped, color: Optional[List] = None) -> None:
        """
        Publish a Pose as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param color: Color of the marker
        """

        if name is None:
            name = 'pose_marker'

        if color is None:
            color = [1, 0, 1, 1]

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_pose=pose)
            return

        # Create arrow marker from pose (pose → arrow from position to orientation offset)
        start_point = Point(
            x=pose.pose.position.x,
            y=pose.pose.position.y,
            z=pose.pose.position.z
        )

        # Simple offset for arrow end
        end_point = Point(
            x=start_point.x + 0.05,
            y=start_point.y,
            z=start_point.z
        )

        marker = self._create_arrow_marker(start_point, end_point, self.current_id,
                                           pose.header.frame_id, color=color)
        marker.ns = name

        self.marker_array.markers.append(marker)
        self.marker_overview[name] = marker.id
        self.current_id += 1

        self.publisher.publish(self.marker_array)
        loginfo(f"Pose '{name}' published")

    def _publish_object(self, name: Optional[str], pose: PoseStamped, bw_object: ObjectDesignatorDescription) -> None:
        """
        Publish an Object as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param bw_object: ObjectDesignatorDescription for the marker
        """

        bw_real = bw_object.resolve()

        if name is None:
            name = bw_real.name

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_pose=pose)
            return

        path = bw_real.world_object.root_link.geometry.file_name
        marker = self._create_object_marker(pose, name, path, self.current_id)

        self.marker_array.markers.append(marker)
        self.marker_overview[name] = marker.id
        self.current_id += 1

        self.publisher.publish(self.marker_array)
        loginfo(f"Object '{name}' published")

    def _update_marker(self, marker_id: int, new_pose: PoseStamped) -> bool:
        """
        Update an existing marker to a new pose

        :param marker_id: id of the marker that should be updated
        :param new_pose: Pose where the updated marker is set

        :return: True if update was successful, False otherwise
        """

        # Find the marker with the specified ID
        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                # Update successful
                marker.pose = new_pose
                self.log_message = f"Marker '{marker.ns}' updated"
                self.publisher.publish(self.marker_array)
                return True

        # Update was not successful
        logwarn(f"Marker {marker_id} not found for update")
        return False

    def remove_marker(self, bw_object: Optional[ObjectDesignatorDescription] = None, name: Optional[str] = None) -> None:
        """
        Remove a marker by object or name

        :param bw_object: Object which marker should be removed
        :param name: Name of object that should be removed
        """

        if bw_object is not None:
            bw_real = bw_object.resolve()
            name = bw_real.name

        if name is None:
            logerr('No name for object given, cannot remove marker')
            return

        marker_id = self.marker_overview.pop(name)

        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                marker.action = Marker.DELETE
        self.publisher.publish(self.marker_array)
        self.marker_array.markers.pop(marker_id)

        loginfo(f"Removed Marker '{name}'")

    def clear_all_marker(self) -> None:
        """
        Clear all existing markers
        """
        for marker in self.marker_array.markers:
            marker.action = Marker.DELETE

        self.marker_overview = {}
        self.publisher.publish(self.marker_array)
        loginfo('Removed all markers')


class TrajectoryPublisher(SingleTypePublisher):
    """
    Publishes a trajectory as a MarkerArray to visualize it in rviz.
    """

    def __init__(self, topic: str = "/pycram/trajectory") -> None:
        super().__init__(topic)

    def visualize(self, trajectory: List[PoseStamped]) -> None:
        """
        Visualize a trajectory in rviz as a series of arrows.

        :param trajectory: The trajectory to visualize as a list of points where an arrow should be drawn
        from point to point.
        """

        marker_array = MarkerArray()
        for index, (p1, p2) in enumerate(zip(trajectory, trajectory[1:])):
            marker = self._create_arrow_marker(
                p1.position, p2.position, index, p1.frame_id
            )
            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)


class BoundingBoxPublisher(SingleTypePublisher):
    """
    Publishes a list of bounding boxes as a MarkerArray to visualize it in rviz.
    """

    def __init__(self, topic: str = "/pycram/bounding_boxes") -> None:
        super().__init__(topic)

    def visualize(self, boxes: BoundingBoxCollection, duration: Optional[int] = 60) -> None:
        """
        Visualize a collection of bounding boxes in rviz as a series of cubes.
        """
        marker_array = MarkerArray()
        for index, box in enumerate(boxes):
            marker = self._create_cube_marker(box, index, duration)
            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)


class CoordinateAxisPublisher(SingleTypePublisher):
    """
    Publishes coordinate axes as a MarkerArray to visualize them in rviz.
    """

    def __init__(self, topic: str = "/pycram/coordinate_axis") -> None:
        super().__init__(topic)

    def visualize(self, poses: List[PoseStamped], duration: Optional[int] = 60, length: Optional[float] = 0.1) -> None:
        """
        Visualize coordinate axes in rviz'
        """
        marker_array = MarkerArray()
        for index, pose in enumerate(poses):
            for axis, color in zip(
                    [AxisIdentifier.X.value, AxisIdentifier.Y.value, AxisIdentifier.Z.value],
                    [Color.from_rgb([1., 0., 0.]), Color.from_rgb([0., 1., 0.]), Color.from_rgb([0., 0., 1.])]
            ):
                marker = self._create_line_marker(pose, axis, color.get_rgba(), index, duration, length)
                marker_array.markers.append(marker)

        self.publisher.publish(marker_array)

def plot_axis_in_rviz(poses: List[PoseStamped], duration: Optional[int] = 60, length: float = 0.3):
    def make_publisher():
        return CoordinateAxisPublisher()

    publisher = make_publisher()
    publisher.visualize(poses, duration, length)