:py:mod:`pycram.object_descriptors.urdf`
========================================

.. py:module:: pycram.object_descriptors.urdf


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.object_descriptors.urdf.LinkDescription
   pycram.object_descriptors.urdf.JointDescription
   pycram.object_descriptors.urdf.ObjectDescription




.. py:class:: LinkDescription(urdf_description: urdf_parser_py.urdf.Link)


   Bases: :py:obj:`pycram.description.LinkDescription`

   A class that represents a link description of an object.

   .. py:property:: geometry
      :type: typing_extensions.Union[pycram.datastructures.dataclasses.VisualShape, None]

      Returns the geometry type of the URDF collision element of this link.

   .. py:property:: origin
      :type: typing_extensions.Union[pycram.datastructures.pose.Pose, None]

      Returns the origin of this entity.

   .. py:property:: name
      :type: str

      Returns the name of this entity.

   .. py:property:: collision
      :type: urdf_parser_py.urdf.Collision


   .. py:method:: _get_visual_shape(urdf_geometry) -> typing_extensions.Union[pycram.datastructures.dataclasses.VisualShape, None]
      :staticmethod:

      Returns the VisualShape of the given URDF geometry.



.. py:class:: JointDescription(urdf_description: urdf_parser_py.urdf.Joint)


   Bases: :py:obj:`pycram.description.JointDescription`

   A class that represents the description of a joint.

   .. py:property:: origin
      :type: pycram.datastructures.pose.Pose

      Returns the origin of this entity.

   .. py:property:: name
      :type: str

      Returns the name of this entity.

   .. py:property:: has_limits
      :type: bool

      Checks if this joint has limits.

      :return: True if the joint has limits, False otherwise.

   .. py:property:: type
      :type: pycram.datastructures.enums.JointType

      The type of this joint.

      :type: return

   .. py:property:: axis
      :type: geometry_msgs.msg.Point

      The axis of this joint, for example the rotation axis for a revolute joint.

      :type: return

   .. py:property:: lower_limit
      :type: typing_extensions.Union[float, None]

      The lower limit of this joint, or None if the joint has no limits.

      :type: return

   .. py:property:: upper_limit
      :type: typing_extensions.Union[float, None]

      The upper limit of this joint, or None if the joint has no limits.

      :type: return

   .. py:property:: parent_link_name
      :type: str

      The name of the parent link of this joint.

      :type: return

   .. py:property:: child_link_name
      :type: str

      The name of the child link of this joint.

      :type: return

   .. py:property:: damping
      :type: float

      The damping of this joint.

      :type: return

   .. py:property:: friction
      :type: float

      The friction of this joint.

      :type: return

   .. py:attribute:: urdf_type_map

      


.. py:class:: ObjectDescription(path: typing_extensions.Optional[str] = None)


   Bases: :py:obj:`pycram.description.ObjectDescription`

   A class that represents an object description of an object.

   :param path: The path of the file to update the description data from.

   .. py:class:: Link(_id: int, link_description: LinkDescription, obj: pycram.world_concepts.world_object.Object)


      Bases: :py:obj:`pycram.description.ObjectDescription.Link`, :py:obj:`LinkDescription`

      Represents a link of an Object in the World.


   .. py:class:: RootLink(obj: pycram.world_concepts.world_object.Object)


      Bases: :py:obj:`pycram.description.ObjectDescription.RootLink`, :py:obj:`Link`

      Represents the root link of an Object in the World.
      It differs from the normal AbstractLink class in that the pose ande the tf_frame is the same as that of the object.


   .. py:class:: Joint(_id: int, joint_description: JointDescription, obj: pycram.world_concepts.world_object.Object)


      Bases: :py:obj:`pycram.description.ObjectDescription.Joint`, :py:obj:`JointDescription`

      Represents a joint of an Object in the World.


   .. py:property:: links
      :type: typing_extensions.List[LinkDescription]

      A list of links descriptions of this object.

      :type: return

   .. py:property:: joints
      :type: typing_extensions.List[JointDescription]

      A list of joints descriptions of this object.

      :type: return

   .. py:property:: origin
      :type: pycram.datastructures.pose.Pose

      Returns the origin of this entity.

   .. py:property:: name
      :type: str

      Returns the name of this entity.

   .. py:method:: load_description(path) -> urdf_parser_py.urdf.URDF

      Loads the description from the file at the given path.

      :param path: The path to the source file, if only a filename is provided then the resources directories will be
       searched.


   .. py:method:: generate_from_mesh_file(path: str, name: str, color: typing_extensions.Optional[pycram.datastructures.dataclasses.Color] = Color()) -> str

      Generates an URDf file with the given .obj or .stl file as mesh. In addition, the given rgba_color will be
      used to create a material tag in the URDF.

      :param path: The path to the mesh file.
      :param name: The name of the object.
      :param color: The color of the object.
      :return: The absolute path of the created file


   .. py:method:: generate_from_description_file(path: str) -> str

      Preprocesses the given file and returns the preprocessed description string.

      :param path: The path of the file to preprocess.
      :return: The preprocessed description string.


   .. py:method:: generate_from_parameter_server(name: str) -> str

      Preprocesses the description from the ROS parameter server and returns the preprocessed description string.

      :param name: The name of the description on the parameter server.
      :return: The preprocessed description string.


   .. py:method:: get_link_by_name(link_name: str) -> LinkDescription

      :return: The link description with the given name.


   .. py:method:: get_joint_by_name(joint_name: str) -> JointDescription

      :return: The joint description with the given name.


   .. py:method:: get_root() -> str

      :return: the name of the root link of this object.


   .. py:method:: get_chain(start_link_name: str, end_link_name: str) -> typing_extensions.List[str]

      :return: the chain of links from 'start_link_name' to 'end_link_name'.


   .. py:method:: correct_urdf_string(urdf_string: str) -> str

      Changes paths for files in the URDF from ROS paths to paths in the file system. Since World (PyBullet legacy)
      can't deal with ROS package paths.

      :param urdf_string: The name of the URDf on the parameter server
      :return: The URDF string with paths in the filesystem instead of ROS packages


   .. py:method:: fix_missing_inertial(urdf_string: str) -> str
      :staticmethod:

      Insert inertial tags for every URDF link that has no inertia.
      This is used to prevent Legacy(PyBullet) from dumping warnings in the terminal

      :param urdf_string: The URDF description as string
      :returns: The new, corrected URDF description as string.


   .. py:method:: remove_error_tags(urdf_string: str) -> str
      :staticmethod:

      Removes all tags in the removing_tags list from the URDF since these tags are known to cause errors with the
      URDF_parser

      :param urdf_string: String of the URDF from which the tags should be removed
      :return: The URDF string with the tags removed


   .. py:method:: fix_link_attributes(urdf_string: str) -> str
      :staticmethod:

      Removes the attribute 'type' from links since this is not parsable by the URDF parser.

      :param urdf_string: The string of the URDF from which the attributes should be removed
      :return: The URDF string with the attributes removed


   .. py:method:: get_file_extension() -> str
      :staticmethod:

      :return: The file extension of the URDF file.



