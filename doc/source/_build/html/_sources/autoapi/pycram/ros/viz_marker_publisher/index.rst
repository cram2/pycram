:py:mod:`pycram.ros.viz_marker_publisher`
=========================================

.. py:module:: pycram.ros.viz_marker_publisher


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.ros.viz_marker_publisher.VizMarkerPublisher
   pycram.ros.viz_marker_publisher.ManualMarkerPublisher




.. py:class:: VizMarkerPublisher(topic_name='/pycram/viz_marker', interval=0.1)


   Publishes an Array of visualization marker which represent the situation in the World

   The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
   World. This Array is published with a rate of interval.

   :param topic_name: The name of the topic to which the Visualization Marker should be published.
   :param interval: The interval at which the visualization marker should be published, in seconds.

   .. py:method:: _publish() -> None

      Constantly publishes the Marker Array. To the given topic name at a fixed rate.


   .. py:method:: _make_marker_array() -> visualization_msgs.msg.MarkerArray

      Creates the Marker Array to be published. There is one Marker for link for each object in the Array, each Object
      creates a name space in the visualization Marker. The type of Visualization Marker is decided by the collision
      tag of the URDF.

      :return: An Array of Visualization Marker


   .. py:method:: _stop_publishing() -> None

      Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.



.. py:class:: ManualMarkerPublisher(topic_name: str = '/pycram/manual_marker', interval: float = 0.1)


   Class to manually add and remove marker of objects and poses.

   The Publisher creates an Array of Visualization marker with a marker for a pose or object.
   This Array is published with a rate of interval.

   :param topic_name: Name of the marker topic
   :param interval: Interval at which the marker should be published

   .. py:method:: publish(pose: pycram.datastructures.pose.Pose, color: Optional[List] = None, bw_object: Optional[pycram.designator.ObjectDesignatorDescription] = None, name: Optional[str] = None)

      Publish a pose or an object into the MarkerArray.
      Priorities to add an object if possible

      :param pose: Pose of the marker
      :param color: Color of the marker if no object is given
      :param bw_object: Object to add as a marker
      :param name: Name of the marker


   .. py:method:: _publish(pose: pycram.datastructures.pose.Pose, bw_object: Optional[pycram.designator.ObjectDesignatorDescription] = None, name: Optional[str] = None, color: Optional[List] = None)

      Publish the marker into the MarkerArray


   .. py:method:: _publish_pose(name: str, pose: pycram.datastructures.pose.Pose, color: Optional[List] = None)

      Publish a Pose as a marker

      :param name: Name of the marker
      :param pose: Pose of the marker
      :param color: Color of the marker


   .. py:method:: _publish_object(name: Optional[str], pose: pycram.datastructures.pose.Pose, bw_object: pycram.designator.ObjectDesignatorDescription)

      Publish an Object as a marker

      :param name: Name of the marker
      :param pose: Pose of the marker
      :param bw_object: ObjectDesignatorDescription for the marker


   .. py:method:: _make_marker_array(name, marker_type: int, marker_pose: pycram.datastructures.pose.Pose, marker_scales: Tuple = (1.0, 1.0, 1.0), color_rgba: std_msgs.msg.ColorRGBA = ColorRGBA(*[1.0, 1.0, 1.0, 1.0]), path_to_resource: Optional[str] = None)

      Create a Marker and add it to the MarkerArray

      :param name: Name of the Marker
      :param marker_type: Type of the marker to create
      :param marker_pose: Pose of the marker
      :param marker_scales: individual scaling of the markers axes
      :param color_rgba: Color of the marker as RGBA
      :param path_to_resource: Path to the resource of a Bulletworld object


   .. py:method:: _update_marker(marker_id: int, new_pose: pycram.datastructures.pose.Pose) -> bool

      Update an existing marker to a new pose

      :param marker_id: id of the marker that should be updated
      :param new_pose: Pose where the updated marker is set

      :return: True if update was successful, False otherwise


   .. py:method:: remove_marker(bw_object: Optional[pycram.designator.ObjectDesignatorDescription] = None, name: Optional[str] = None)

      Remove a marker by object or name

      :param bw_object: Object which marker should be removed
      :param name: Name of object that should be removed


   .. py:method:: clear_all_marker()

      Clear all existing markers



