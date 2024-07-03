:py:mod:`pycram.local_transformer`
==================================

.. py:module:: pycram.local_transformer


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.local_transformer.LocalTransformer




.. py:class:: LocalTransformer


   Bases: :py:obj:`tf.TransformerROS`

   This class allows to use the TF class TransformerROS without using the ROS
   network system or the topic /tf, where transforms are usually published to.
   Instead, a local transformer is saved and allows to publish local transforms,
   as well the use of TFs convenient lookup functions (see functions below).

   This class uses the robots (currently only one! supported) URDF file to
   initialize the tfs for the robot. Moreover, the function update_local_transformer_from_btr
   updates these tfs by copying the tfs state from the world.

   This class extends the TransformerRos, you can find documentation for TransformerROS here:
   `TFDoc <http://wiki.ros.org/tf/TfUsingPython>`_

   .. py:attribute:: _instance

      

   .. py:method:: transform_to_object_frame(pose: pycram.datastructures.pose.Pose, world_object: world_concepts.world_object.Object, link_name: str = None) -> typing_extensions.Union[pycram.datastructures.pose.Pose, None]

      Transforms the given pose to the coordinate frame of the given World object. If no link name is given the
      base frame of the Object is used, otherwise the link frame is used as target for the transformation.

      :param pose: Pose that should be transformed
      :param world_object: BulletWorld Object in which frame the pose should be transformed
      :param link_name: A link of the BulletWorld Object which will be used as target coordinate frame instead
      :return: The new pose the in coordinate frame of the object


   .. py:method:: transform_pose(pose: pycram.datastructures.pose.Pose, target_frame: str) -> typing_extensions.Union[pycram.datastructures.pose.Pose, None]

      Transforms a given pose to the target frame after updating the transforms for all objects in the current world.

      :param pose: Pose that should be transformed
      :param target_frame: Name of the TF frame into which the Pose should be transformed
      :return: A transformed pose in the target frame


   .. py:method:: lookup_transform_from_source_to_target_frame(source_frame: str, target_frame: str, time: typing_extensions.Optional[rospy.rostime.Time] = None) -> pycram.datastructures.pose.Transform

      Update the transforms for all world objects then Look up for the latest known transform that transforms a point
       from source frame to target frame. If no time is given the last common time between the two frames is used.

      :param time: Time at which the transform should be looked up


   .. py:method:: update_transforms(transforms: typing_extensions.Iterable[pycram.datastructures.pose.Transform], time: rospy.Time = None) -> None

      Updates transforms by updating the time stamps of the header of each transform. If no time is given the current
      time is used.


   .. py:method:: get_all_frames() -> typing_extensions.List[str]

      Returns all know coordinate frames as a list with human-readable entries.

      :return: A list of all know coordinate frames.


   .. py:method:: transformPose(target_frame, ps) -> pycram.datastructures.pose.Pose

      Alias for :func:`~LocalTransformer.transform_pose_to_target_frame` to avoid confusion since a similar method
       exists in the super class.



