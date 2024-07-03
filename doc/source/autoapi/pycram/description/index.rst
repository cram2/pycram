:py:mod:`pycram.description`
============================

.. py:module:: pycram.description


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.description.EntityDescription
   pycram.description.LinkDescription
   pycram.description.JointDescription
   pycram.description.ObjectEntity
   pycram.description.Link
   pycram.description.RootLink
   pycram.description.Joint
   pycram.description.ObjectDescription




.. py:class:: EntityDescription


   Bases: :py:obj:`abc.ABC`

   A class that represents a description of an entity. This can be a link, joint or object description.

   .. py:property:: origin
      :type: pycram.datastructures.pose.Pose
      :abstractmethod:

      Returns the origin of this entity.

   .. py:property:: name
      :type: str
      :abstractmethod:

      Returns the name of this entity.


.. py:class:: LinkDescription(parsed_link_description: typing_extensions.Any)


   Bases: :py:obj:`EntityDescription`

   A class that represents a link description of an object.

   .. py:property:: geometry
      :type: typing_extensions.Union[pycram.datastructures.dataclasses.VisualShape, None]
      :abstractmethod:

      Returns the geometry type of the collision element of this link.


.. py:class:: JointDescription(parsed_joint_description: typing_extensions.Any)


   Bases: :py:obj:`EntityDescription`

   A class that represents the description of a joint.

   .. py:property:: type
      :type: pycram.datastructures.enums.JointType
      :abstractmethod:

      The type of this joint.

      :type: return

   .. py:property:: axis
      :type: geometry_msgs.msg.Point
      :abstractmethod:

      The axis of this joint, for example the rotation axis for a revolute joint.

      :type: return

   .. py:property:: has_limits
      :type: bool
      :abstractmethod:

      Checks if this joint has limits.

      :return: True if the joint has limits, False otherwise.

   .. py:property:: limits
      :type: typing_extensions.Tuple[float, float]

      The lower and upper limits of this joint.

      :type: return

   .. py:property:: lower_limit
      :type: typing_extensions.Union[float, None]
      :abstractmethod:

      The lower limit of this joint, or None if the joint has no limits.

      :type: return

   .. py:property:: upper_limit
      :type: typing_extensions.Union[float, None]
      :abstractmethod:

      The upper limit of this joint, or None if the joint has no limits.

      :type: return

   .. py:property:: parent_link_name
      :type: str
      :abstractmethod:

      The name of the parent link of this joint.

      :type: return

   .. py:property:: child_link_name
      :type: str
      :abstractmethod:

      The name of the child link of this joint.

      :type: return

   .. py:property:: damping
      :type: float
      :abstractmethod:

      The damping of this joint.

      :type: return

   .. py:property:: friction
      :type: float
      :abstractmethod:

      The friction of this joint.

      :type: return


.. py:class:: ObjectEntity(_id: int, obj: pycram.world_concepts.world_object.Object)


   Bases: :py:obj:`pycram.datastructures.world.WorldEntity`

   An abstract base class that represents a physical part/entity of an Object.
   This can be a link or a joint of an Object.

   .. py:property:: pose
      :type: pycram.datastructures.pose.Pose
      :abstractmethod:

      The pose of this entity relative to the world frame.

      :type: return

   .. py:property:: transform
      :type: pycram.datastructures.pose.Transform

      Returns the transform of this entity.

      :return: The transform of this entity.

   .. py:property:: tf_frame
      :type: str
      :abstractmethod:

      Returns the tf frame of this entity.

      :return: The tf frame of this entity.

   .. py:property:: object_id
      :type: int

      the id of the object to which this entity belongs.

      :type: return


.. py:class:: Link(_id: int, link_description: LinkDescription, obj: pycram.world_concepts.world_object.Object)


   Bases: :py:obj:`ObjectEntity`, :py:obj:`LinkDescription`, :py:obj:`abc.ABC`

   Represents a link of an Object in the World.

   .. py:property:: current_state
      :type: pycram.datastructures.dataclasses.LinkState


   .. py:property:: is_root
      :type: bool

      Returns whether this link is the root link of the object.

      :return: True if this link is the root link, False otherwise.

   .. py:property:: position
      :type: geometry_msgs.msg.Point

      The getter for the position of the link relative to the world frame.

      :return: A Point object containing the position of the link relative to the world frame.

   .. py:property:: position_as_list
      :type: typing_extensions.List[float]

      The getter for the position of the link relative to the world frame as a list.

      :return: A list containing the position of the link relative to the world frame.

   .. py:property:: orientation
      :type: geometry_msgs.msg.Quaternion

      The getter for the orientation of the link relative to the world frame.

      :return: A Quaternion object containing the orientation of the link relative to the world frame.

   .. py:property:: orientation_as_list
      :type: typing_extensions.List[float]

      The getter for the orientation of the link relative to the world frame as a list.

      :return: A list containing the orientation of the link relative to the world frame.

   .. py:property:: pose
      :type: pycram.datastructures.pose.Pose

      The pose of the link relative to the world frame.

      :return: A Pose object containing the pose of the link relative to the world frame.

   .. py:property:: pose_as_list
      :type: typing_extensions.List[typing_extensions.List[float]]

      The pose of the link relative to the world frame as a list.

      :return: A list containing the position and orientation of the link relative to the world frame.

   .. py:property:: color
      :type: pycram.datastructures.dataclasses.Color

      The getter for the rgba_color of this link.

      :return: A Color object containing the rgba_color of this link.

   .. py:property:: origin_transform
      :type: pycram.datastructures.pose.Transform

      The transform from world to origin of entity.

      :type: return

   .. py:property:: tf_frame
      :type: str

      The name of the tf frame of this link.

   .. py:method:: add_fixed_constraint_with_link(child_link: Link) -> int

      Adds a fixed constraint between this link and the given link, used to create attachments for example.

      :param child_link: The child link to which a fixed constraint should be added.
      :return: The unique id of the constraint.


   .. py:method:: remove_constraint_with_link(child_link: Link) -> None

      Removes the constraint between this link and the given link.

      :param child_link: The child link of the constraint that should be removed.


   .. py:method:: update_transform(transform_time: typing_extensions.Optional[rospy.Time] = None) -> None

      Updates the transformation of this link at the given time.

      :param transform_time: The time at which the transformation should be updated.


   .. py:method:: get_transform_to_link(link: Link) -> pycram.datastructures.pose.Transform

      Returns the transformation from this link to the given link.

      :param link: The link to which the transformation should be returned.
      :return: A Transform object with the transformation from this link to the given link.


   .. py:method:: get_transform_from_link(link: Link) -> pycram.datastructures.pose.Transform

      Returns the transformation from the given link to this link.

      :param link: The link from which the transformation should be returned.
      :return: A Transform object with the transformation from the given link to this link.


   .. py:method:: get_pose_wrt_link(link: Link) -> pycram.datastructures.pose.Pose

      Returns the pose of this link with respect to the given link.

      :param link: The link with respect to which the pose should be returned.
      :return: A Pose object with the pose of this link with respect to the given link.


   .. py:method:: get_axis_aligned_bounding_box() -> pycram.datastructures.dataclasses.AxisAlignedBoundingBox

      Returns the axis aligned bounding box of this link.

      :return: An AxisAlignedBoundingBox object with the axis aligned bounding box of this link.


   .. py:method:: _update_pose() -> None

      Updates the current pose of this link from the world.


   .. py:method:: get_origin_transform() -> pycram.datastructures.pose.Transform

      Returns the transformation between the link frame and the origin frame of this link.


   .. py:method:: __eq__(other)

      Return self==value.


   .. py:method:: __copy__()


   .. py:method:: __hash__()

      Return hash(self).



.. py:class:: RootLink(obj: pycram.world_concepts.world_object.Object)


   Bases: :py:obj:`Link`, :py:obj:`abc.ABC`

   Represents the root link of an Object in the World.
   It differs from the normal AbstractLink class in that the pose ande the tf_frame is the same as that of the object.

   .. py:property:: tf_frame
      :type: str

      Returns the tf frame of the root link, which is the same as the tf frame of the object.

   .. py:method:: _update_pose() -> None

      Updates the current pose of this link from the world.


   .. py:method:: __copy__()



.. py:class:: Joint(_id: int, joint_description: JointDescription, obj: pycram.world_concepts.world_object.Object)


   Bases: :py:obj:`ObjectEntity`, :py:obj:`JointDescription`, :py:obj:`abc.ABC`

   Represents a joint of an Object in the World.

   .. py:property:: tf_frame
      :type: str

      The tf frame of a joint is the tf frame of the child link.

   .. py:property:: pose
      :type: pycram.datastructures.pose.Pose

      Returns the pose of this joint. The pose is the pose of the child link of this joint.

      :return: The pose of this joint.

   .. py:property:: parent_link
      :type: Link

      Returns the parent link of this joint.

      :return: The parent link as a AbstractLink object.

   .. py:property:: child_link
      :type: Link

      Returns the child link of this joint.

      :return: The child link as a AbstractLink object.

   .. py:property:: position
      :type: float


   .. py:property:: current_state
      :type: pycram.datastructures.dataclasses.JointState


   .. py:method:: _update_position() -> None

      Updates the current position of the joint from the physics simulator.


   .. py:method:: reset_position(position: float) -> None


   .. py:method:: get_object_id() -> int

      Returns the id of the object to which this joint belongs.

      :return: The integer id of the object to which this joint belongs.


   .. py:method:: enable_force_torque_sensor() -> None


   .. py:method:: disable_force_torque_sensor() -> None


   .. py:method:: get_reaction_force_torque() -> typing_extensions.List[float]


   .. py:method:: get_applied_motor_torque() -> float


   .. py:method:: __copy__()


   .. py:method:: __eq__(other)

      Return self==value.


   .. py:method:: __hash__()

      Return hash(self).



.. py:class:: ObjectDescription(path: typing_extensions.Optional[str] = None)


   Bases: :py:obj:`EntityDescription`

   A class that represents the description of an object.

   :param path: The path of the file to update the description data from.

   .. py:class:: Link(_id: int, link_description: LinkDescription, obj: pycram.world_concepts.world_object.Object)


      Bases: :py:obj:`Link`, :py:obj:`abc.ABC`

      Represents a link of an Object in the World.


   .. py:class:: RootLink(obj: pycram.world_concepts.world_object.Object)


      Bases: :py:obj:`RootLink`, :py:obj:`abc.ABC`

      Represents the root link of an Object in the World.
      It differs from the normal AbstractLink class in that the pose ande the tf_frame is the same as that of the object.


   .. py:class:: Joint(_id: int, joint_description: JointDescription, obj: pycram.world_concepts.world_object.Object)


      Bases: :py:obj:`Joint`, :py:obj:`abc.ABC`

      Represents a joint of an Object in the World.


   .. py:property:: parsed_description
      :type: typing_extensions.Any

      Return the object parsed from the description file.

   .. py:property:: links
      :type: typing_extensions.List[LinkDescription]
      :abstractmethod:

      A list of links descriptions of this object.

      :type: return

   .. py:property:: joints
      :type: typing_extensions.List[JointDescription]
      :abstractmethod:

      A list of joints descriptions of this object.

      :type: return

   .. py:attribute:: mesh_extensions
      :type: typing_extensions.Tuple[str]
      :value: ('.obj', '.stl', '.dae')

      The file extensions of the mesh files that can be used to generate a description file.

   .. py:method:: update_description_from_file(path: str) -> None

      Updates the description of this object from the file at the given path.

      :param path: The path of the file to update from.


   .. py:method:: load_description(path: str) -> typing_extensions.Any
      :abstractmethod:

      Loads the description from the file at the given path.

      :param path: The path to the source file, if only a filename is provided then the resources directories will be
       searched.


   .. py:method:: generate_description_from_file(path: str, name: str, extension: str) -> str

      Generates and preprocesses the description from the file at the given path and returns the preprocessed
      description as a string.

      :param path: The path of the file to preprocess.
      :param name: The name of the object.
      :param extension: The file extension of the file to preprocess.
      :return: The processed description string.


   .. py:method:: get_file_name(path_object: pathlib.Path, extension: str, object_name: str) -> str

      Returns the file name of the description file.

      :param path_object: The path object of the description file or the mesh file.
      :param extension: The file extension of the description file or the mesh file.
      :param object_name: The name of the object.
      :return: The file name of the description file.


   .. py:method:: generate_from_mesh_file(path: str, name: str) -> str
      :classmethod:
      :abstractmethod:

      Generates a description file from one of the mesh types defined in the mesh_extensions and
      returns the path of the generated file.

      :param path: The path to the .obj file.
      :param name: The name of the object.
      :return: The path of the generated description file.


   .. py:method:: generate_from_description_file(path: str) -> str
      :classmethod:
      :abstractmethod:

      Preprocesses the given file and returns the preprocessed description string.

      :param path: The path of the file to preprocess.
      :return: The preprocessed description string.


   .. py:method:: generate_from_parameter_server(name: str) -> str
      :classmethod:
      :abstractmethod:

      Preprocesses the description from the ROS parameter server and returns the preprocessed description string.

      :param name: The name of the description on the parameter server.
      :return: The preprocessed description string.


   .. py:method:: get_link_by_name(link_name: str) -> LinkDescription
      :abstractmethod:

      :return: The link description with the given name.


   .. py:method:: get_joint_by_name(joint_name: str) -> JointDescription
      :abstractmethod:

      :return: The joint description with the given name.


   .. py:method:: get_root() -> str
      :abstractmethod:

      :return: the name of the root link of this object.


   .. py:method:: get_chain(start_link_name: str, end_link_name: str) -> typing_extensions.List[str]
      :abstractmethod:

      :return: the chain of links from 'start_link_name' to 'end_link_name'.


   .. py:method:: get_file_extension() -> str
      :staticmethod:
      :abstractmethod:

      :return: The file extension of the description file.



