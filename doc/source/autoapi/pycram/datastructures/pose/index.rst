:py:mod:`pycram.datastructures.pose`
====================================

.. py:module:: pycram.datastructures.pose


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.datastructures.pose.Pose
   pycram.datastructures.pose.Transform



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.datastructures.pose.get_normalized_quaternion



.. py:function:: get_normalized_quaternion(quaternion: numpy.ndarray) -> geometry_msgs.msg.Quaternion

   Normalizes a given quaternion such that it has a magnitude of 1.

   :param quaternion: The quaternion that should be normalized
   :return: The normalized quaternion


.. py:class:: Pose(position: typing_extensions.Optional[typing_extensions.List[float]] = None, orientation: typing_extensions.Optional[typing_extensions.List[float]] = None, frame: str = 'map', time: rospy.Time = None)


   Bases: :py:obj:`geometry_msgs.msg.PoseStamped`

   Pose representation for PyCRAM, this class extends the PoseStamped ROS message from geometry_msgs. Thus making it
   compatible with every ROS service and message expecting a PoseStamped message.

   Naming convention for Poses:
       Pose: Instances of this class, representing a cartesian position and a quaternion for orientation

       Position: Only the cartesian position in xyz

       Orientation: Only the quaternion as xyzw

   Poses can be initialized by a position and orientation given as lists, this is optional. By default, Poses are
   initialized with the position being [0, 0, 0], the orientation being [0, 0, 0, 1] and the frame being 'map'.

   :param position: An optional position of this Pose
   :param orientation: An optional orientation of this Pose
   :param frame: An optional frame in which this pose is
   :param time: The time at which this Pose is valid, as ROS time

   .. py:property:: frame
      :type: str

      Property for the frame_id such that it is easier accessible. Instead of Pose.header.frame_id it is Pose.frame

      :return: The TF frame of this Pose

   .. py:property:: position
      :type: geometry_msgs.msg.Point

      Property that points to the position of this pose

   .. py:property:: orientation
      :type: geometry_msgs.msg.Quaternion

      Property that points to the orientation of this pose

   .. py:method:: from_pose_stamped(pose_stamped: geometry_msgs.msg.PoseStamped) -> Pose
      :staticmethod:

      Converts a geometry_msgs/PoseStamped message to a Pose object. Should be used for compatability with ROS.

      :param pose_stamped: The pose stamped message which should be converted
      :return: A Pose object with the same information as the given message


   .. py:method:: to_list() -> typing_extensions.List[typing_extensions.List[float]]

      Returns the position and orientation of this pose as a list containing two list.

      :return: The position and orientation as lists


   .. py:method:: to_transform(child_frame: str) -> Transform

      Converts this pose to a Transform from the TF frame of the pose to the given child_frame

      :param child_frame: Child frame id to which the Transform points
      :return: A new Transform


   .. py:method:: copy() -> Pose

      Creates a deep copy of this pose.

      :return: A copy of this pose


   .. py:method:: position_as_list() -> typing_extensions.List[float]

      Returns only the position as a list of xyz.

      :return: The position as a list


   .. py:method:: orientation_as_list() -> typing_extensions.List[float]

      Returns only the orientation as a list of a quaternion

      :return: The orientation as a quaternion with xyzw


   .. py:method:: dist(other_pose: Pose) -> float

      Calculates the euclidian distance between this Pose and the given one. For distance calculation only the
      position is used.

      :param other_pose: Pose to which the distance should be calculated
      :return: The distance between the Poses


   .. py:method:: __eq__(other: Pose) -> bool

      Overloads the '==' operator to check for equality between two Poses. Only compares the position, orientation and
      frame. Timestamps of Poses are not takes into account.

      :param other: Other pose which should be compared
      :return: True if both Poses have the same position, orientation and frame. False otherwise


   .. py:method:: set_position(new_position: typing_extensions.List[float]) -> None

      Sets the position of this Pose to the given position. Position has to be given as a vector in cartesian space.

      :param new_position: New position as a vector of xyz


   .. py:method:: set_orientation(new_orientation: typing_extensions.List[float]) -> None

      Sets the orientation to the given quaternion. The new orientation has to be given as a quaternion.

      :param new_orientation: New orientation as a quaternion with xyzw


   .. py:method:: to_sql() -> pycram.orm.base.Pose


   .. py:method:: insert(session: sqlalchemy.orm.Session) -> pycram.orm.base.Pose


   .. py:method:: multiply_quaternions(quaternion: typing_extensions.List) -> None

      Multiply the quaternion of this Pose with the given quaternion, the result will be the new orientation of this
      Pose.

      :param quaternion: The quaternion by which the orientation of this Pose should be multiplied


   .. py:method:: set_orientation_from_euler(axis: typing_extensions.List, euler_angles: typing_extensions.List[float]) -> None

      Convert axis-angle to quaternion.

      :param axis: (x, y, z) tuple representing rotation axis.
      :param angle: rotation angle in degree
      :return: The quaternion representing the axis angle



.. py:class:: Transform(translation: typing_extensions.Optional[typing_extensions.List[float]] = None, rotation: typing_extensions.Optional[typing_extensions.List[float]] = None, frame: typing_extensions.Optional[str] = 'map', child_frame: typing_extensions.Optional[str] = '', time: rospy.Time = None)


   Bases: :py:obj:`geometry_msgs.msg.TransformStamped`

   Represents a Transformation from one TF frame to another in PyCRAM. Like with Poses this class inherits from the ROS
   message TransformStamped form geometry_msgs and is therefore compatible with ROS services and messages that require
   a TransformStamped message.

   Naming Convention for Transforms:
       Transform: Instances of this class, representing a translation and rotation from frame_id to child_frame_id

       Translation: A vector representing the conversion in cartesian space

       Rotation: A quaternion representing the conversion of rotation between both frames

   Transforms take a translation, rotation, frame and child_frame as optional arguments. If nothing is given the
   Transform will be initialized with [0, 0, 0] for translation, [0, 0, 0, 1] for rotation, 'map' for frame and an
   empty string for child_frame

   :param translation: Optional translation from frame to child_frame in cartesian space
   :param rotation: Optional rotation from frame to child frame given as quaternion
   :param frame: Origin TF frame of this Transform
   :param child_frame: Target frame for this Transform
   :param time: The time at which this Transform is valid, as ROS time

   .. py:property:: frame
      :type: str

      Property for the frame_id such that it is easier accessible. Instead of Pose.header.frame_id it is Pose.frame

      :return: The TF frame of this Pose

   .. py:property:: translation
      :type: None

      Property that points to the translation of this Transform

   .. py:property:: rotation
      :type: None

      Property that points to the rotation of this Transform

   .. py:method:: from_pose_and_child_frame(pose: Pose, child_frame_name: str) -> Transform
      :classmethod:


   .. py:method:: from_transform_stamped(transform_stamped: geometry_msgs.msg.TransformStamped) -> Transform
      :staticmethod:

      Creates a Transform instance from a geometry_msgs/TransformStamped message. Should be used for compatibility with
      ROS.

      :param transform_stamped: The transform stamped message that should be converted
      :return: An Transform with the same information as the transform stamped message


   .. py:method:: copy() -> Transform

      Creates a deep copy of this pose.

      :return: A copy of this pose


   .. py:method:: translation_as_list() -> typing_extensions.List[float]

      Returns the translation of this Transform as a list.

      :return: The translation as a list of xyz


   .. py:method:: rotation_as_list() -> typing_extensions.List[float]

      Returns the rotation of this Transform as a list representing a quaternion.

      :return: The rotation of this Transform as a list with xyzw


   .. py:method:: to_pose() -> Pose

      Converts this Transform to a Pose, in this process the child_frame_id is lost.

      :return: A new pose with same translation as position and rotation as orientation


   .. py:method:: invert() -> Transform

      Inverts this Transform, the new Transform points from the child_frame_id to the frame_id

      :return: A new inverted Transform


   .. py:method:: __mul__(other: Transform) -> typing_extensions.Union[Transform, None]

      Multiplies this Transform with another one. The resulting Transform points from the frame_id of this Transform
      to the child_frame_id of the other Transform.

      :param other: The Transform which should be multiplied with this one.
      :return: The resulting Transform from the multiplication


   .. py:method:: inverse_times(other_transform: Transform) -> Transform

      Like a 'minus' for Transforms, subtracts the other_transform from this one.

      :param other_transform: Transform which should be subtracted from this one
      :return: The resulting Transform form the calculation


   .. py:method:: __eq__(other: Transform) -> bool

      Overloads the '==' operator to check for equality between two Transforms. Only compares the translation,
      rotation, frame and child frame. Timestamps of Poses are not takes into account.

      :param other: Other pose which should be compared
      :return: True if both Transforms have the same translation, rotation, frame and child frame. False otherwise


   .. py:method:: set_translation(new_translation: typing_extensions.List[float]) -> None

      Sets the translation of this Transform to the newly given one. Translation has to be a vector in cartesian space

      :param new_translation: The new translation as a vector with xyz.


   .. py:method:: set_rotation(new_rotation: typing_extensions.List[float]) -> None

      Sets the rotation of this Transform to the newly given one. Rotation has to be a quaternion.

      :param new_rotation: The new rotation as a quaternion with xyzw



