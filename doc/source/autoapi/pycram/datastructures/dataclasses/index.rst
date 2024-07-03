:py:mod:`pycram.datastructures.dataclasses`
===========================================

.. py:module:: pycram.datastructures.dataclasses


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.datastructures.dataclasses.Color
   pycram.datastructures.dataclasses.AxisAlignedBoundingBox
   pycram.datastructures.dataclasses.CollisionCallbacks
   pycram.datastructures.dataclasses.MultiBody
   pycram.datastructures.dataclasses.VisualShape
   pycram.datastructures.dataclasses.BoxVisualShape
   pycram.datastructures.dataclasses.SphereVisualShape
   pycram.datastructures.dataclasses.CapsuleVisualShape
   pycram.datastructures.dataclasses.CylinderVisualShape
   pycram.datastructures.dataclasses.MeshVisualShape
   pycram.datastructures.dataclasses.PlaneVisualShape
   pycram.datastructures.dataclasses.State
   pycram.datastructures.dataclasses.LinkState
   pycram.datastructures.dataclasses.JointState
   pycram.datastructures.dataclasses.ObjectState
   pycram.datastructures.dataclasses.WorldState



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.datastructures.dataclasses.get_point_as_list



.. py:function:: get_point_as_list(point: pycram.datastructures.pose.Point) -> typing_extensions.List[float]

   Returns the point as a list.

   :param point: The point.
   :return: The point as a list


.. py:class:: Color


   Dataclass for storing rgba_color as an RGBA value.
   The values are stored as floats between 0 and 1.
   The default rgba_color is white. 'A' stands for the opacity.

   .. py:attribute:: R
      :type: float
      :value: 1

      

   .. py:attribute:: G
      :type: float
      :value: 1

      

   .. py:attribute:: B
      :type: float
      :value: 1

      

   .. py:attribute:: A
      :type: float
      :value: 1

      

   .. py:method:: from_list(color: typing_extensions.List[float])
      :classmethod:

      Sets the rgba_color from a list of RGBA values.

      :param color: The list of RGBA values


   .. py:method:: from_rgb(rgb: typing_extensions.List[float])
      :classmethod:

      Sets the rgba_color from a list of RGB values.

      :param rgb: The list of RGB values


   .. py:method:: from_rgba(rgba: typing_extensions.List[float])
      :classmethod:

      Sets the rgba_color from a list of RGBA values.

      :param rgba: The list of RGBA values


   .. py:method:: get_rgba() -> typing_extensions.List[float]

      Returns the rgba_color as a list of RGBA values.

      :return: The rgba_color as a list of RGBA values


   .. py:method:: get_rgb() -> typing_extensions.List[float]

      Returns the rgba_color as a list of RGB values.

      :return: The rgba_color as a list of RGB values



.. py:class:: AxisAlignedBoundingBox


   Dataclass for storing an axis-aligned bounding box.

   .. py:attribute:: min_x
      :type: float

      

   .. py:attribute:: min_y
      :type: float

      

   .. py:attribute:: min_z
      :type: float

      

   .. py:attribute:: max_x
      :type: float

      

   .. py:attribute:: max_y
      :type: float

      

   .. py:attribute:: max_z
      :type: float

      

   .. py:method:: from_min_max(min_point: typing_extensions.List[float], max_point: typing_extensions.List[float])
      :classmethod:

      Sets the axis-aligned bounding box from a minimum and maximum point.

      :param min_point: The minimum point
      :param max_point: The maximum point


   .. py:method:: get_min_max_points() -> typing_extensions.Tuple[pycram.datastructures.pose.Point, pycram.datastructures.pose.Point]

      Returns the axis-aligned bounding box as a tuple of minimum and maximum points.

      :return: The axis-aligned bounding box as a tuple of minimum and maximum points


   .. py:method:: get_min_point() -> pycram.datastructures.pose.Point

      Returns the axis-aligned bounding box as a minimum point.

      :return: The axis-aligned bounding box as a minimum point


   .. py:method:: get_max_point() -> pycram.datastructures.pose.Point

      Returns the axis-aligned bounding box as a maximum point.

      :return: The axis-aligned bounding box as a maximum point


   .. py:method:: get_min_max() -> typing_extensions.Tuple[typing_extensions.List[float], typing_extensions.List[float]]

      Returns the axis-aligned bounding box as a tuple of minimum and maximum points.

      :return: The axis-aligned bounding box as a tuple of minimum and maximum points


   .. py:method:: get_min() -> typing_extensions.List[float]

      Returns the minimum point of the axis-aligned bounding box.

      :return: The minimum point of the axis-aligned bounding box


   .. py:method:: get_max() -> typing_extensions.List[float]

      Returns the maximum point of the axis-aligned bounding box.

      :return: The maximum point of the axis-aligned bounding box



.. py:class:: CollisionCallbacks


   .. py:attribute:: on_collision_cb
      :type: typing_extensions.Callable

      

   .. py:attribute:: no_collision_cb
      :type: typing_extensions.Optional[typing_extensions.Callable]

      


.. py:class:: MultiBody


   .. py:attribute:: base_visual_shape_index
      :type: int

      

   .. py:attribute:: base_pose
      :type: pycram.datastructures.pose.Pose

      

   .. py:attribute:: link_visual_shape_indices
      :type: typing_extensions.List[int]

      

   .. py:attribute:: link_poses
      :type: typing_extensions.List[pycram.datastructures.pose.Pose]

      

   .. py:attribute:: link_masses
      :type: typing_extensions.List[float]

      

   .. py:attribute:: link_inertial_frame_poses
      :type: typing_extensions.List[pycram.datastructures.pose.Pose]

      

   .. py:attribute:: link_parent_indices
      :type: typing_extensions.List[int]

      

   .. py:attribute:: link_joint_types
      :type: typing_extensions.List[pycram.datastructures.enums.JointType]

      

   .. py:attribute:: link_joint_axis
      :type: typing_extensions.List[pycram.datastructures.pose.Point]

      

   .. py:attribute:: link_collision_shape_indices
      :type: typing_extensions.List[int]

      


.. py:class:: VisualShape


   Bases: :py:obj:`abc.ABC`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:property:: visual_geometry_type
      :type: pycram.datastructures.enums.Shape
      :abstractmethod:

      Returns the visual geometry type of the visual shape (e.g. box, sphere).

   .. py:attribute:: rgba_color
      :type: Color

      

   .. py:attribute:: visual_frame_position
      :type: typing_extensions.List[float]

      

   .. py:method:: shape_data() -> typing_extensions.Dict[str, typing_extensions.Any]
      :abstractmethod:

      Returns the shape data of the visual shape (e.g. half extents for a box, radius for a sphere).



.. py:class:: BoxVisualShape


   Bases: :py:obj:`VisualShape`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:property:: visual_geometry_type
      :type: pycram.datastructures.enums.Shape

      Returns the visual geometry type of the visual shape (e.g. box, sphere).

   .. py:property:: size
      :type: typing_extensions.List[float]


   .. py:attribute:: half_extents
      :type: typing_extensions.List[float]

      

   .. py:method:: shape_data() -> typing_extensions.Dict[str, typing_extensions.List[float]]

      Returns the shape data of the visual shape (e.g. half extents for a box, radius for a sphere).



.. py:class:: SphereVisualShape


   Bases: :py:obj:`VisualShape`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:property:: visual_geometry_type
      :type: pycram.datastructures.enums.Shape

      Returns the visual geometry type of the visual shape (e.g. box, sphere).

   .. py:attribute:: radius
      :type: float

      

   .. py:method:: shape_data() -> typing_extensions.Dict[str, float]

      Returns the shape data of the visual shape (e.g. half extents for a box, radius for a sphere).



.. py:class:: CapsuleVisualShape


   Bases: :py:obj:`VisualShape`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:property:: visual_geometry_type
      :type: pycram.datastructures.enums.Shape

      Returns the visual geometry type of the visual shape (e.g. box, sphere).

   .. py:attribute:: radius
      :type: float

      

   .. py:attribute:: length
      :type: float

      

   .. py:method:: shape_data() -> typing_extensions.Dict[str, float]

      Returns the shape data of the visual shape (e.g. half extents for a box, radius for a sphere).



.. py:class:: CylinderVisualShape


   Bases: :py:obj:`CapsuleVisualShape`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:property:: visual_geometry_type
      :type: pycram.datastructures.enums.Shape

      Returns the visual geometry type of the visual shape (e.g. box, sphere).


.. py:class:: MeshVisualShape


   Bases: :py:obj:`VisualShape`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:property:: visual_geometry_type
      :type: pycram.datastructures.enums.Shape

      Returns the visual geometry type of the visual shape (e.g. box, sphere).

   .. py:attribute:: scale
      :type: typing_extensions.List[float]

      

   .. py:attribute:: file_name
      :type: str

      

   .. py:method:: shape_data() -> typing_extensions.Dict[str, typing_extensions.Union[typing_extensions.List[float], str]]

      Returns the shape data of the visual shape (e.g. half extents for a box, radius for a sphere).



.. py:class:: PlaneVisualShape


   Bases: :py:obj:`VisualShape`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:property:: visual_geometry_type
      :type: pycram.datastructures.enums.Shape

      Returns the visual geometry type of the visual shape (e.g. box, sphere).

   .. py:attribute:: normal
      :type: typing_extensions.List[float]

      

   .. py:method:: shape_data() -> typing_extensions.Dict[str, typing_extensions.List[float]]

      Returns the shape data of the visual shape (e.g. half extents for a box, radius for a sphere).



.. py:class:: State


   Bases: :py:obj:`abc.ABC`

   Helper class that provides a standard way to create an ABC using
   inheritance.


.. py:class:: LinkState


   Bases: :py:obj:`State`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:attribute:: constraint_ids
      :type: typing_extensions.Dict[pycram.description.Link, int]

      


.. py:class:: JointState


   Bases: :py:obj:`State`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:attribute:: position
      :type: float

      


.. py:class:: ObjectState


   Bases: :py:obj:`State`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:attribute:: pose
      :type: pycram.datastructures.pose.Pose

      

   .. py:attribute:: attachments
      :type: typing_extensions.Dict[pycram.world_concepts.world_object.Object, pycram.world_concepts.constraints.Attachment]

      

   .. py:attribute:: link_states
      :type: typing_extensions.Dict[int, LinkState]

      

   .. py:attribute:: joint_states
      :type: typing_extensions.Dict[int, JointState]

      


.. py:class:: WorldState


   Bases: :py:obj:`State`

   Helper class that provides a standard way to create an ABC using
   inheritance.

   .. py:attribute:: simulator_state_id
      :type: int

      

   .. py:attribute:: object_states
      :type: typing_extensions.Dict[str, ObjectState]

      


