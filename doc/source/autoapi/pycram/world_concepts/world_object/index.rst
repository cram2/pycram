:py:mod:`pycram.world_concepts.world_object`
============================================

.. py:module:: pycram.world_concepts.world_object


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.world_concepts.world_object.Object




Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.world_concepts.world_object.Link


.. py:data:: Link

   

.. py:class:: Object(name: str, obj_type: pycram.datastructures.enums.ObjectType, path: str, description: typing_extensions.Optional[typing_extensions.Type[pycram.description.ObjectDescription]] = URDFObject, pose: typing_extensions.Optional[pycram.datastructures.pose.Pose] = None, world: typing_extensions.Optional[pycram.datastructures.world.World] = None, color: typing_extensions.Optional[pycram.datastructures.dataclasses.Color] = Color(), ignore_cached_files: typing_extensions.Optional[bool] = False)


   Bases: :py:obj:`pycram.datastructures.world.WorldEntity`

   Represents a spawned Object in the World.

   The constructor loads the description file into the given World, if no World is specified the
   :py:attr:`~World.current_world` will be used. It is also possible to load .obj and .stl file into the World.
   The rgba_color parameter is only used when loading .stl or .obj files,
   for URDFs :func:`~Object.set_color` can be used.

   :param name: The name of the object
   :param obj_type: The type of the object as an ObjectType enum.
   :param path: The path to the source file, if only a filename is provided then the resources directories will be searched.
   :param description: The ObjectDescription of the object, this contains the joints and links of the object.
   :param pose: The pose at which the Object should be spawned
   :param world: The World in which the object should be spawned, if no world is specified the :py:attr:`~World.current_world` will be used.
   :param color: The rgba_color with which the object should be spawned.
   :param ignore_cached_files: If true the file will be spawned while ignoring cached files.

   .. py:property:: pose


   .. py:property:: link_names
      :type: typing_extensions.List[str]

      The name of each link as a list.

      :type: return

   .. py:property:: joint_names
      :type: typing_extensions.List[str]

      The name of each joint as a list.

      :type: return

   .. py:property:: base_origin_shift
      :type: numpy.ndarray

      The shift between the base of the object and the origin of the object.

      :return: A numpy array with the shift between the base of the object and the origin of the object.

   .. py:property:: current_state
      :type: pycram.datastructures.dataclasses.ObjectState


   .. py:property:: link_states
      :type: typing_extensions.Dict[int, pycram.datastructures.dataclasses.LinkState]

      Returns the current state of all links of this object.

      :return: A dictionary with the link id as key and the current state of the link as value.

   .. py:property:: joint_states
      :type: typing_extensions.Dict[int, pycram.datastructures.dataclasses.JointState]

      Returns the current state of all joints of this object.

      :return: A dictionary with the joint id as key and the current state of the joint as value.

   .. py:property:: root_link
      :type: pycram.description.ObjectDescription.Link

      Returns the root link of this object.

      :return: The root link of this object.

   .. py:property:: root_link_name
      :type: str

      Returns the name of the root link of this object.

      :return: The name of the root link of this object.

   .. py:property:: links_colors
      :type: typing_extensions.Dict[str, pycram.datastructures.dataclasses.Color]

      The color of each link as a dictionary with link names as keys and RGBA colors as values.

   .. py:attribute:: prospection_world_prefix
      :type: str
      :value: 'prospection/'

      The ObjectDescription of the object, this contains the name and type of the object as well as the path to the source
      file.

   .. py:method:: _load_object_and_get_id(path: typing_extensions.Optional[str] = None, ignore_cached_files: typing_extensions.Optional[bool] = False) -> typing_extensions.Tuple[int, typing_extensions.Union[str, None]]

      Loads an object to the given World with the given position and orientation. The rgba_color will only be
      used when an .obj or .stl file is given.
      If a .obj or .stl file is given, before spawning, an urdf file with the .obj or .stl as mesh will be created
      and this URDf file will be loaded instead.
      When spawning a URDf file a new file will be created in the cache directory, if there exists none.
      This new file will have resolved mesh file paths, meaning there will be no references
      to ROS packges instead there will be absolute file paths.

      :param path: The path to the description file, if None then no file will be loaded, this is useful when the PyCRAM is not responsible for loading the file but another system is.
      :param ignore_cached_files: Whether to ignore files in the cache directory.
      :return: The unique id of the object and the path of the file that was loaded.


   .. py:method:: _init_joint_name_and_id_map() -> None

      Creates a dictionary which maps the joint names to their unique ids and vice versa.


   .. py:method:: _init_link_name_and_id_map() -> None

      Creates a dictionary which maps the link names to their unique ids and vice versa.


   .. py:method:: _init_links_and_update_transforms() -> None

      Initializes the link objects from the URDF file and creates a dictionary which maps the link names to the
      corresponding link objects.


   .. py:method:: _init_joints()

      Initialize the joint objects from the URDF file and creates a dictionary which mas the joint names to the
      corresponding joint objects


   .. py:method:: _add_to_world_sync_obj_queue() -> None

      Adds this object to the objects queue of the WorldSync object of the World.


   .. py:method:: get_link(link_name: str) -> pycram.description.ObjectDescription.Link

      Returns the link object with the given name.

      :param link_name: The name of the link.
      :return: The link object.


   .. py:method:: get_link_pose(link_name: str) -> pycram.datastructures.pose.Pose

      Returns the pose of the link with the given name.

      :param link_name: The name of the link.
      :return: The pose of the link.


   .. py:method:: get_link_position(link_name: str) -> geometry_msgs.msg.Point

      Returns the position of the link with the given name.

      :param link_name: The name of the link.
      :return: The position of the link.


   .. py:method:: get_link_position_as_list(link_name: str) -> typing_extensions.List[float]

      Returns the position of the link with the given name.

      :param link_name: The name of the link.
      :return: The position of the link.


   .. py:method:: get_link_orientation(link_name: str) -> geometry_msgs.msg.Quaternion

      Returns the orientation of the link with the given name.

      :param link_name: The name of the link.
      :return: The orientation of the link.


   .. py:method:: get_link_orientation_as_list(link_name: str) -> typing_extensions.List[float]

      Returns the orientation of the link with the given name.

      :param link_name: The name of the link.
      :return: The orientation of the link.


   .. py:method:: get_link_tf_frame(link_name: str) -> str

      Returns the tf frame of the link with the given name.

      :param link_name: The name of the link.
      :return: The tf frame of the link.


   .. py:method:: get_link_axis_aligned_bounding_box(link_name: str) -> pycram.datastructures.dataclasses.AxisAlignedBoundingBox

      Returns the axis aligned bounding box of the link with the given name.

      :param link_name: The name of the link.
      :return: The axis aligned bounding box of the link.


   .. py:method:: get_transform_between_links(from_link: str, to_link: str) -> pycram.datastructures.pose.Transform

      Returns the transform between two links.

      :param from_link: The name of the link from which the transform should be calculated.
      :param to_link: The name of the link to which the transform should be calculated.


   .. py:method:: get_link_color(link_name: str) -> pycram.datastructures.dataclasses.Color

      Returns the color of the link with the given name.

      :param link_name: The name of the link.
      :return: The color of the link.


   .. py:method:: set_link_color(link_name: str, color: typing_extensions.List[float]) -> None

      Sets the color of the link with the given name.

      :param link_name: The name of the link.
      :param color: The new color of the link.


   .. py:method:: get_link_geometry(link_name: str) -> typing_extensions.Union[pycram.datastructures.dataclasses.VisualShape, None]

      Returns the geometry of the link with the given name.

      :param link_name: The name of the link.
      :return: The geometry of the link.


   .. py:method:: get_link_transform(link_name: str) -> pycram.datastructures.pose.Transform

      Returns the transform of the link with the given name.

      :param link_name: The name of the link.
      :return: The transform of the link.


   .. py:method:: get_link_origin(link_name: str) -> pycram.datastructures.pose.Pose

      Returns the origin of the link with the given name.

      :param link_name: The name of the link.
      :return: The origin of the link as a 'Pose'.


   .. py:method:: get_link_origin_transform(link_name: str) -> pycram.datastructures.pose.Transform

      Returns the origin transform of the link with the given name.

      :param link_name: The name of the link.
      :return: The origin transform of the link.


   .. py:method:: __repr__()


   .. py:method:: remove() -> None

      Removes this object from the World it currently resides in.
      For the object to be removed it has to be detached from all objects it
      is currently attached to. After this is done a call to world remove object is done
      to remove this Object from the simulation/world.


   .. py:method:: reset(remove_saved_states=True) -> None

      Resets the Object to the state it was first spawned in.
      All attached objects will be detached, all joints will be set to the
      default position of 0 and the object will be set to the position and
      orientation in which it was spawned.

      :param remove_saved_states: If True the saved states will be removed.


   .. py:method:: attach(child_object: Object, parent_link: typing_extensions.Optional[str] = None, child_link: typing_extensions.Optional[str] = None, bidirectional: typing_extensions.Optional[bool] = True) -> None

      Attaches another object to this object. This is done by
      saving the transformation between the given link, if there is one, and
      the base pose of the other object. Additionally, the name of the link, to
      which the object is attached, will be saved.
      Furthermore, a simulator constraint will be created so the attachment
      also works while simulation.
      Loose attachments means that the attachment will only be one-directional. For example, if this object moves the
      other, attached, object will also move but not the other way around.

      :param child_object: The other object that should be attached.
      :param parent_link: The link name of this object.
      :param child_link: The link name of the other object.
      :param bidirectional: If the attachment should be a loose attachment.


   .. py:method:: detach(child_object: Object) -> None

      Detaches another object from this object. This is done by
      deleting the attachment from the attachments dictionary of both objects
      and deleting the constraint of the simulator.
      Afterward the detachment event of the corresponding World will be fired.

      :param child_object: The object which should be detached


   .. py:method:: detach_all() -> None

      Detach all objects attached to this object.


   .. py:method:: update_attachment_with_object(child_object: Object)


   .. py:method:: get_position() -> geometry_msgs.msg.Point

      Returns the position of this Object as a list of xyz.

      :return: The current position of this object


   .. py:method:: get_orientation() -> pycram.datastructures.pose.Pose.orientation

      Returns the orientation of this object as a list of xyzw, representing a quaternion.

      :return: A list of xyzw


   .. py:method:: get_position_as_list() -> typing_extensions.List[float]

      Returns the position of this Object as a list of xyz.

      :return: The current position of this object


   .. py:method:: get_base_position_as_list() -> typing_extensions.List[float]

      Returns the position of this Object as a list of xyz.

      :return: The current position of this object


   .. py:method:: get_orientation_as_list() -> typing_extensions.List[float]

      Returns the orientation of this object as a list of xyzw, representing a quaternion.

      :return: A list of xyzw


   .. py:method:: get_pose() -> pycram.datastructures.pose.Pose

      Returns the position of this object as a list of xyz. Alias for :func:`~Object.get_position`.

      :return: The current pose of this object


   .. py:method:: set_pose(pose: pycram.datastructures.pose.Pose, base: typing_extensions.Optional[bool] = False, set_attachments: typing_extensions.Optional[bool] = True) -> None

      Sets the Pose of the object.

      :param pose: New Pose for the object
      :param base: If True places the object base instead of origin at the specified position and orientation
      :param set_attachments: Whether to set the poses of the attached objects to this object or not.


   .. py:method:: reset_base_pose(pose: pycram.datastructures.pose.Pose)


   .. py:method:: update_pose()

      Updates the current pose of this object from the world, and updates the poses of all links.


   .. py:method:: _update_all_links_poses()

      Updates the poses of all links by getting them from the simulator.


   .. py:method:: move_base_to_origin_pose() -> None

      Move the object such that its base will be at the current origin position.
      This is useful when placing objects on surfaces where you want the object base in contact with the surface.


   .. py:method:: save_state(state_id) -> None

      Saves the state of this object by saving the state of all links and attachments.

      :param state_id: The unique id of the state.


   .. py:method:: save_links_states(state_id: int) -> None

      Saves the state of all links of this object.

      :param state_id: The unique id of the state.


   .. py:method:: save_joints_states(state_id: int) -> None

      Saves the state of all joints of this object.

      :param state_id: The unique id of the state.


   .. py:method:: set_attachments(attachments: typing_extensions.Dict[Object, pycram.world_concepts.constraints.Attachment]) -> None

      Sets the attachments of this object to the given attachments.

      :param attachments: A dictionary with the object as key and the attachment as value.


   .. py:method:: remove_saved_states() -> None

      Removes all saved states of this object.


   .. py:method:: remove_links_saved_states() -> None

      Removes all saved states of the links of this object.


   .. py:method:: remove_joints_saved_states() -> None

      Removes all saved states of the joints of this object.


   .. py:method:: _set_attached_objects_poses(already_moved_objects: typing_extensions.Optional[typing_extensions.List[Object]] = None) -> None

      Updates the positions of all attached objects. This is done
      by calculating the new pose in world coordinate frame and setting the
      base pose of the attached objects to this new pose.
      After this the _set_attached_objects method of all attached objects
      will be called.

      :param already_moved_objects: A list of Objects that were already moved, these will be excluded to prevent loops in the update.


   .. py:method:: set_position(position: typing_extensions.Union[pycram.datastructures.pose.Pose, geometry_msgs.msg.Point, typing_extensions.List], base=False) -> None

      Sets this Object to the given position, if base is true the bottom of the Object will be placed at the position
      instead of the origin in the center of the Object. The given position can either be a Pose,
      in this case only the position is used or a geometry_msgs.msg/Point which is the position part of a Pose.

      :param position: Target position as xyz.
      :param base: If the bottom of the Object should be placed or the origin in the center.


   .. py:method:: set_orientation(orientation: typing_extensions.Union[pycram.datastructures.pose.Pose, geometry_msgs.msg.Quaternion, typing_extensions.List, typing_extensions.Tuple, numpy.ndarray]) -> None

      Sets the orientation of the Object to the given orientation. Orientation can either be a Pose, in this case only
      the orientation of this pose is used or a geometry_msgs.msg/Quaternion which is the orientation of a Pose.

      :param orientation: Target orientation given as a list of xyzw.


   .. py:method:: get_joint_id(name: str) -> int

      Returns the unique id for a joint name. As used by the world/simulator.

      :param name: The joint name
      :return: The unique id


   .. py:method:: get_root_link_description() -> pycram.description.LinkDescription

      Returns the root link of the URDF of this object.

      :return: The root link as defined in the URDF of this object.


   .. py:method:: get_root_link_id() -> int

      Returns the unique id of the root link of this object.

      :return: The unique id of the root link of this object.


   .. py:method:: get_link_id(link_name: str) -> int

      Returns a unique id for a link name.

      :param link_name: The name of the link.
      :return: The unique id of the link.


   .. py:method:: get_link_by_id(link_id: int) -> pycram.description.ObjectDescription.Link

      Returns the link for a given unique link id

      :param link_id: The unique id of the link.
      :return: The link object.


   .. py:method:: reset_all_joints_positions() -> None

      Sets the current position of all joints to 0. This is useful if the joints should be reset to their default


   .. py:method:: set_joint_positions(joint_poses: dict) -> None

      Sets the current position of multiple joints at once, this method should be preferred when setting
      multiple joints at once instead of running :func:`~Object.set_joint_position` in a loop.

      :param joint_poses:


   .. py:method:: set_joint_position(joint_name: str, joint_position: float) -> None

      Sets the position of the given joint to the given joint pose and updates the poses of all attached objects.

      :param joint_name: The name of the joint
      :param joint_position: The target pose for this joint


   .. py:method:: get_joint_position(joint_name: str) -> float

      :param joint_name: The name of the joint
      :return: The current position of the given joint


   .. py:method:: get_joint_damping(joint_name: str) -> float

      :param joint_name: The name of the joint
      :return: The damping of the given joint


   .. py:method:: get_joint_upper_limit(joint_name: str) -> float

      :param joint_name: The name of the joint
      :return: The upper limit of the given joint


   .. py:method:: get_joint_lower_limit(joint_name: str) -> float

      :param joint_name: The name of the joint
      :return: The lower limit of the given joint


   .. py:method:: get_joint_axis(joint_name: str) -> geometry_msgs.msg.Point

      :param joint_name: The name of the joint
      :return: The axis of the given joint


   .. py:method:: get_joint_type(joint_name: str) -> pycram.datastructures.enums.JointType

      :param joint_name: The name of the joint
      :return: The type of the given joint


   .. py:method:: get_joint_limits(joint_name: str) -> typing_extensions.Tuple[float, float]

      :param joint_name: The name of the joint
      :return: The lower and upper limits of the given joint


   .. py:method:: get_joint_child_link(joint_name: str) -> pycram.description.ObjectDescription.Link

      :param joint_name: The name of the joint
      :return: The child link of the given joint


   .. py:method:: get_joint_parent_link(joint_name: str) -> pycram.description.ObjectDescription.Link

      :param joint_name: The name of the joint
      :return: The parent link of the given joint


   .. py:method:: find_joint_above_link(link_name: str, joint_type: pycram.datastructures.enums.JointType) -> str

      Traverses the chain from 'link' to the URDF origin and returns the first joint that is of type 'joint_type'.

      :param link_name: AbstractLink name above which the joint should be found
      :param joint_type: Joint type that should be searched for
      :return: Name of the first joint which has the given type


   .. py:method:: get_positions_of_all_joints() -> typing_extensions.Dict[str, float]

      Returns the positions of all joints of the object as a dictionary of joint names and joint positions.

      :return: A dictionary with all joints positions'.


   .. py:method:: update_link_transforms(transform_time: typing_extensions.Optional[rospy.Time] = None) -> None

      Updates the transforms of all links of this object using time 'transform_time' or the current ros time.


   .. py:method:: contact_points() -> typing_extensions.List

      Returns a list of contact points of this Object with other Objects.

      :return: A list of all contact points with other objects


   .. py:method:: contact_points_simulated() -> typing_extensions.List

      Returns a list of all contact points between this Object and other Objects after stepping the simulation once.

      :return: A list of contact points between this Object and other Objects


   .. py:method:: set_color(rgba_color: pycram.datastructures.dataclasses.Color) -> None

      Changes the color of this object, the color has to be given as a list
      of RGBA values.

      :param rgba_color: The color as Color object with RGBA values between 0 and 1


   .. py:method:: get_color() -> typing_extensions.Union[pycram.datastructures.dataclasses.Color, typing_extensions.Dict[str, pycram.datastructures.dataclasses.Color]]

      This method returns the rgba_color of this object. The return is either:

          1. A Color object with RGBA values, this is the case if the object only has one link (this
              happens for example if the object is spawned from a .obj or .stl file)
          2. A dict with the link name as key and the rgba_color as value. The rgba_color is given as a Color Object.
              Please keep in mind that not every link may have a rgba_color. This is dependent on the URDF from which
               the object is spawned.

      :return: The rgba_color as Color object with RGBA values between 0 and 1 or a dict with the link name as key and the rgba_color as value.


   .. py:method:: get_axis_aligned_bounding_box() -> pycram.datastructures.dataclasses.AxisAlignedBoundingBox

      :return: The axis aligned bounding box of this object.


   .. py:method:: get_base_origin() -> pycram.datastructures.pose.Pose

      :return: the origin of the base/bottom of this object.


   .. py:method:: get_joint_by_id(joint_id: int) -> pycram.description.Joint

      Returns the joint object with the given id.

      :param joint_id: The unique id of the joint.
      :return: The joint object.


   .. py:method:: copy_to_prospection() -> Object

      Copies this object to the prospection world.

      :return: The copied object in the prospection world.


   .. py:method:: __copy__() -> Object

      Returns a copy of this object. The copy will have the same name, type, path, description, pose, world and color.

      :return: A copy of this object.


   .. py:method:: __eq__(other)


   .. py:method:: __hash__()



