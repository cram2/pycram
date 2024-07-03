:py:mod:`pycram.ros.tf_broadcaster`
===================================

.. py:module:: pycram.ros.tf_broadcaster


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.ros.tf_broadcaster.TFBroadcaster




.. py:class:: TFBroadcaster(projection_namespace='simulated', odom_frame='odom', interval=0.1)


   Broadcaster that publishes TF frames for every object in the World.

   The broadcaster prefixes all published TF messages with a projection namespace to distinguish between the TF
   frames from the simulation and the one from the real robot.

   :param projection_namespace: Name with which the TF frames should be prefixed
   :param odom_frame: Name of the statically published odom frame
   :param interval: Interval at which the TFs should be published, in seconds

   .. py:method:: update()

      Updates the TFs for the static odom frame and all objects currently in the World.


   .. py:method:: _update_objects() -> None

      Publishes the current pose of all objects in the World. As well as the poses of all links of these objects.


   .. py:method:: _update_static_odom() -> None

      Publishes a static odom frame to the tf_static topic.


   .. py:method:: _publish_pose(child_frame_id: str, pose: pycram.datastructures.pose.Pose, static=False) -> None

      Publishes the given pose to the ROS TF topic. First the pose is converted to a Transform between pose.frame and
      the given child_frame_id. Afterward, the frames of the Transform are prefixed with the projection namespace.

      :param child_frame_id: Name of the TF frame which the pose points to
      :param pose: Pose that should be published
      :param static: If the pose should be published to the tf_static topic


   .. py:method:: _publish() -> None

      Constantly publishes the positions of all objects in the World.


   .. py:method:: _stop_publishing() -> None

      Called when the process ends, sets the kill_event which terminates the thread that publishes to the TF topic.



