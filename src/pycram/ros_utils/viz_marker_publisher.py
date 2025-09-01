import atexit
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .marker_publisher_base import MarkerPublisherBase
from ..datastructures.dataclasses import BoundingBoxCollection, Color
from ..datastructures.enums import AxisIdentifier
from ..datastructures.pose import PoseStamped, TransformStamped
from ..datastructures.world import World
from ..designator import ObjectDesignatorDescription
from ..ros import  loginfo, logwarn, logerr
from ..ros import  sleep

@dataclass
class VizMarkerPublisher(MarkerPublisherBase):
    """
    Publishes an Array of visualization marker which represent the situation in the World
    """

    topic: str = "/pycram/viz_marker"
    """The name of the topic to which the Visualization Marker should be published."""

    interval: float = 0.1
    """The interval at which the visualization marker should be published in seconds."""

    reference_frame: str = "map"
    """The reference frame of the visualization marker."""

    use_prospection_world: bool = False
    """Is prospection_world used?"""

    publish_visuals: bool = False
    """Should the visualization marker be published?"""

    def __post_init__(self):
        super().__init__(topic="/pycram/prospection_viz_marker" if self.use_prospection_world else self.topic)

        if self.use_prospection_world:
            self.main_world = World.current_world.prospection_world
        else:
            self.main_world = (
                World.current_world
                if not World.current_world.is_prospection_world
                else World.current_world.world_sync.world
            )

        self.lock = self.main_world.object_lock
        self.kill_event = threading.Event()
        self.thread = threading.Thread(target=self.visualize)
        self.thread.start()
        atexit.register(self._stop_publishing)

    def _get_data(self) -> List[dict]:
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
                    result.append({
                        "geom": geom,
                        "obj": obj,
                        "link": link,
                        "index": i,
                        "pose": link_pose_with_origin
                    })

        return result

    def _make_marker_array(self, data: List[dict]) -> MarkerArray:
        """
        Build MarkerArray from prepared link data.
        If no data is given, automatically gather it from the world.
        """
        marker_array = MarkerArray()
        for entry in data:
            msg = self._create_geometry_marker(
                entry["geom"], entry["obj"], entry["link"], entry["index"], entry["pose"],
                self.reference_frame, self.use_prospection_world)
            marker_array.markers.append(msg)
        return marker_array


    def visualize(self, data: List[dict] = None) -> None:
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

@dataclass
class ManualMarkerPublisher(MarkerPublisherBase):
    """
    Class to manually add and remove marker of objects and poses.
    """

    topic: str = "/pycram/manual_marker"
    """The name of the topic to which the Visualization Marker should be published."""

    interval: float = 0.1
    """The interval at which the visualization marker should be published in seconds."""

    def __post_init__(self):
        """
        The Publisher creates an Array of Visualization marker with a marker for a pose or object.
        """

        self.start_time = None
        self.marker_array = MarkerArray()
        self.marker_overview = {}
        self.current_id = 0

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


@dataclass
class TrajectoryPublisher(MarkerPublisherBase):
    """
    Publishes a trajectory as a MarkerArray to visualize it in rviz.
    """

    topic: str = "/pycram/trajectory"
    """The name of the topic that the trajectory should be published to"""

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


@dataclass
class BoundingBoxPublisher(MarkerPublisherBase):
    """
    Publishes a list of bounding boxes as a MarkerArray to visualize it in rviz.
    """

    topic: str = "/pycram/bounding_boxes"
    """The name of the topic that the bounding boxes should be published to"""

    def visualize(self, boxes: BoundingBoxCollection, duration: Optional[int] = 60) -> None:
        """
        Visualize a collection of bounding boxes in rviz as a series of cubes.
        """
        marker_array = MarkerArray()
        for index, box in enumerate(boxes):
            marker = self._create_cube_marker(box, index, duration)
            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)


@dataclass
class CoordinateAxisPublisher(MarkerPublisherBase):
    """
    Publishes coordinate axes as a MarkerArray to visualize them in rviz.
    """

    topic: str = "/pycram/coordinate_axis"
    """The name of the topic that the coordinate axis should be published to"""

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