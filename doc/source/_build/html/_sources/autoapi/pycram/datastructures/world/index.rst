:py:mod:`pycram.datastructures.world`
=====================================

.. py:module:: pycram.datastructures.world


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.datastructures.world.StateEntity
   pycram.datastructures.world.WorldEntity
   pycram.datastructures.world.World
   pycram.datastructures.world.UseProspectionWorld
   pycram.datastructures.world.WorldSync




.. py:class:: StateEntity


   The StateEntity class is used to store the state of an object or the physics simulator. This is used to save and
   restore the state of the World.

   .. py:property:: saved_states
      :type: typing_extensions.Dict[int, pycram.datastructures.dataclasses.State]

      Returns the saved states of this entity.

   .. py:property:: current_state
      :type: pycram.datastructures.dataclasses.State
      :abstractmethod:

      Returns the current state of this entity.

      :return: The current state of this entity.

   .. py:method:: save_state(state_id: int) -> int

      Saves the state of this entity with the given state id.

      :param state_id: The unique id of the state.


   .. py:method:: restore_state(state_id: int) -> None

      Restores the state of this entity from a saved state using the given state id.

      :param state_id: The unique id of the state.


   .. py:method:: remove_saved_states() -> None

      Removes all saved states of this entity.



.. py:class:: WorldEntity(_id: int, world: typing_extensions.Optional[World] = None)


   Bases: :py:obj:`StateEntity`, :py:obj:`abc.ABC`

   A data class that represents an entity of the world, such as an object or a link.


.. py:class:: World(mode: pycram.datastructures.enums.WorldMode, is_prospection_world: bool, simulation_frequency: float)


   Bases: :py:obj:`StateEntity`, :py:obj:`abc.ABC`

   The World Class represents the physics Simulation and belief state, it is the main interface for reasoning about
   the World. This is implemented as a singleton, the current World can be accessed via the static variable
   current_world which is managed by the World class itself.

   Creates a new simulation, the mode decides if the simulation should be a rendered window or just run in the
   background. There can only be one rendered simulation.
   The World object also initializes the Events for attachment, detachment and for manipulating the world.

   :param mode: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default parameter is "GUI"
   :param is_prospection_world: For internal usage, decides if this World should be used as a prospection world.


   .. py:property:: simulation_time_step

      The time step of the simulation in seconds.

   .. py:property:: current_state
      :type: pycram.datastructures.dataclasses.WorldState

      Returns the current state of this entity.

      :return: The current state of this entity.

   .. py:property:: object_states
      :type: typing_extensions.Dict[str, pycram.datastructures.dataclasses.ObjectState]

      Returns the states of all objects in the World.

      :return: A dictionary with the object id as key and the object state as value.

   .. py:attribute:: simulation_frequency
      :type: float

      Global reference for the simulation frequency (Hz), used in calculating the equivalent real time in the simulation.

   .. py:attribute:: current_world
      :type: typing_extensions.Optional[World]

      Global reference to the currently used World, usually this is the
      graphical one. However, if you are inside a UseProspectionWorld() environment the current_world points to the
      prospection world. In this way you can comfortably use the current_world, which should point towards the World
      used at the moment.

   .. py:attribute:: robot
      :type: typing_extensions.Optional[pycram.world_concepts.world_object.Object]

      Global reference to the spawned Object that represents the robot. The robot is identified by checking the name in
      the URDF with the name of the URDF on the parameter server.

   .. py:attribute:: data_directory
      :type: typing_extensions.List[str]

      Global reference for the data directories, this is used to search for the description files of the robot
      and the objects.

   .. py:attribute:: cache_dir

      Global reference for the cache directory, this is used to cache the description files of the robot and the objects.

   .. py:method:: _init_world(mode: pycram.datastructures.enums.WorldMode)
      :abstractmethod:

      Initializes the physics simulation.


   .. py:method:: _init_events()

      Initializes dynamic events that can be used to react to changes in the World.


   .. py:method:: _init_and_sync_prospection_world()

      Initializes the prospection world and the synchronization between the main and the prospection world.


   .. py:method:: _update_local_transformer_worlds()

      Updates the local transformer worlds with the current world and prospection world.


   .. py:method:: _init_prospection_world()

      Initializes the prospection world, if this is a prospection world itself it will not create another prospection,
      world, but instead set the prospection world to None, else it will create a prospection world.


   .. py:method:: _sync_prospection_world()

      Synchronizes the prospection world with the main world, this means that every object in the main world will be
      added to the prospection world and vice versa.


   .. py:method:: update_cache_dir_with_object(path: str, ignore_cached_files: bool, obj: pycram.world_concepts.world_object.Object) -> str

      Updates the cache directory with the given object.

      :param path: The path to the object.
      :param ignore_cached_files: If the cached files should be ignored.
      :param obj: The object to be added to the cache directory.


   .. py:method:: load_object_and_get_id(path: typing_extensions.Optional[str] = None, pose: typing_extensions.Optional[pycram.datastructures.pose.Pose] = None) -> int
      :abstractmethod:

      Loads a description file (e.g. URDF) at the given pose and returns the id of the loaded object.

      :param path: The path to the description file, if None the description file is assumed to be already loaded.
      :param pose: The pose at which the object should be loaded.
      :return: The id of the loaded object.


   .. py:method:: get_object_by_name(name: str) -> typing_extensions.List[pycram.world_concepts.world_object.Object]

      Returns a list of all Objects in this World with the same name as the given one.

      :param name: The name of the returned Objects.
      :return: A list of all Objects with the name 'name'.


   .. py:method:: get_object_by_type(obj_type: pycram.datastructures.enums.ObjectType) -> typing_extensions.List[pycram.world_concepts.world_object.Object]

      Returns a list of all Objects which have the type 'obj_type'.

      :param obj_type: The type of the returned Objects.
      :return: A list of all Objects that have the type 'obj_type'.


   .. py:method:: get_object_by_id(obj_id: int) -> pycram.world_concepts.world_object.Object

      Returns the single Object that has the unique id.

      :param obj_id: The unique id for which the Object should be returned.
      :return: The Object with the id 'id'.


   .. py:method:: remove_object_by_id(obj_id: int) -> None
      :abstractmethod:

      Removes the object with the given id from the world.

      :param obj_id: The unique id of the object to be removed.


   .. py:method:: remove_object_from_simulator(obj: pycram.world_concepts.world_object.Object) -> None
      :abstractmethod:

      Removes an object from the physics simulator.

      :param obj: The object to be removed.


   .. py:method:: remove_object(obj: pycram.world_concepts.world_object.Object) -> None

      Removes this object from the current world.
      For the object to be removed it has to be detached from all objects it
      is currently attached to. After this is done a call to world remove object is done
      to remove this Object from the simulation/world.

      :param obj: The object to be removed.


   .. py:method:: add_fixed_constraint(parent_link: pycram.description.Link, child_link: pycram.description.Link, child_to_parent_transform: pycram.datastructures.pose.Transform) -> int

      Creates a fixed joint constraint between the given parent and child links,
      the joint frame will be at the origin of the child link frame, and would have the same orientation
      as the child link frame.

      :param parent_link: The constrained link of the parent object.
      :param child_link: The constrained link of the child object.
      :param child_to_parent_transform: The transform from the child link frame to the parent link frame.
      :return: The unique id of the created constraint.


   .. py:method:: add_constraint(constraint: pycram.world_concepts.constraints.Constraint) -> int
      :abstractmethod:

      Add a constraint between two objects links so that they become attached for example.

      :param constraint: The constraint data used to create the constraint.


   .. py:method:: remove_constraint(constraint_id) -> None
      :abstractmethod:

      Remove a constraint by its ID.

      :param constraint_id: The unique id of the constraint to be removed.


   .. py:method:: get_joint_position(joint: pycram.description.Joint) -> float
      :abstractmethod:

      Get the position of a joint of an articulated object

      :param joint: The joint to get the position for.
      :return: The joint position as a float.


   .. py:method:: get_object_joint_names(obj: pycram.world_concepts.world_object.Object) -> typing_extensions.List[str]
      :abstractmethod:

      Returns the names of all joints of this object.

      :param obj: The object.
      :return: A list of joint names.


   .. py:method:: get_link_pose(link: pycram.description.Link) -> pycram.datastructures.pose.Pose
      :abstractmethod:

      Get the pose of a link of an articulated object with respect to the world frame.

      :param link: The link as a AbstractLink object.
      :return: The pose of the link as a Pose object.


   .. py:method:: get_object_link_names(obj: pycram.world_concepts.world_object.Object) -> typing_extensions.List[str]
      :abstractmethod:

      Returns the names of all links of this object.

      :param obj: The object.
      :return: A list of link names.


   .. py:method:: simulate(seconds: float, real_time: typing_extensions.Optional[bool] = False) -> None

      Simulates Physics in the World for a given amount of seconds. Usually this simulation is faster than real
      time. By setting the 'real_time' parameter this simulation is slowed down such that the simulated time is equal
      to real time.

      :param seconds: The amount of seconds that should be simulated.
      :param real_time: If the simulation should happen in real time or faster.


   .. py:method:: update_all_objects_poses() -> None

      Updates the positions of all objects in the world.


   .. py:method:: get_object_pose(obj: pycram.world_concepts.world_object.Object) -> pycram.datastructures.pose.Pose
      :abstractmethod:

      Get the pose of an object in the world frame from the current object pose in the simulator.


   .. py:method:: perform_collision_detection() -> None
      :abstractmethod:

      Checks for collisions between all objects in the World and updates the contact points.


   .. py:method:: get_object_contact_points(obj: pycram.world_concepts.world_object.Object) -> typing_extensions.List
      :abstractmethod:

      Returns a list of contact points of this Object with all other Objects.

      :param obj: The object.
      :return: A list of all contact points with other objects


   .. py:method:: get_contact_points_between_two_objects(obj1: pycram.world_concepts.world_object.Object, obj2: pycram.world_concepts.world_object.Object) -> typing_extensions.List
      :abstractmethod:

      Returns a list of contact points between obj1 and obj2.

      :param obj1: The first object.
      :param obj2: The second object.
      :return: A list of all contact points between the two objects.


   .. py:method:: reset_joint_position(joint: pycram.description.Joint, joint_position: float) -> None
      :abstractmethod:

      Reset the joint position instantly without physics simulation

      :param joint: The joint to reset the position for.
      :param joint_position: The new joint pose.


   .. py:method:: reset_object_base_pose(obj: pycram.world_concepts.world_object.Object, pose: pycram.datastructures.pose.Pose)
      :abstractmethod:

      Reset the world position and orientation of the base of the object instantaneously,
      not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation.

      :param obj: The object.
      :param pose: The new pose as a Pose object.


   .. py:method:: step()
      :abstractmethod:

      Step the world simulation using forward dynamics


   .. py:method:: set_link_color(link: pycram.description.Link, rgba_color: pycram.datastructures.dataclasses.Color)
      :abstractmethod:

      Changes the rgba_color of a link of this object, the rgba_color has to be given as Color object.

      :param link: The link which should be colored.
      :param rgba_color: The rgba_color as Color object with RGBA values between 0 and 1.


   .. py:method:: get_link_color(link: pycram.description.Link) -> pycram.datastructures.dataclasses.Color
      :abstractmethod:

      This method returns the rgba_color of this link.

      :param link: The link for which the rgba_color should be returned.
      :return: The rgba_color as Color object with RGBA values between 0 and 1.


   .. py:method:: get_colors_of_object_links(obj: pycram.world_concepts.world_object.Object) -> typing_extensions.Dict[str, pycram.datastructures.dataclasses.Color]
      :abstractmethod:

      Get the RGBA colors of each link in the object as a dictionary from link name to rgba_color.

      :param obj: The object
      :return: A dictionary with link names as keys and a Color object for each link as value.


   .. py:method:: get_object_axis_aligned_bounding_box(obj: pycram.world_concepts.world_object.Object) -> pycram.datastructures.dataclasses.AxisAlignedBoundingBox
      :abstractmethod:

      Returns the axis aligned bounding box of this object. The return of this method are two points in
      world coordinate frame which define a bounding box.

      :param obj: The object for which the bounding box should be returned.
      :return: AxisAlignedBoundingBox object containing the min and max points of the bounding box.


   .. py:method:: get_link_axis_aligned_bounding_box(link: pycram.description.Link) -> pycram.datastructures.dataclasses.AxisAlignedBoundingBox
      :abstractmethod:

      Returns the axis aligned bounding box of the link. The return of this method are two points in
      world coordinate frame which define a bounding box.


   .. py:method:: set_realtime(real_time: bool) -> None
      :abstractmethod:

      Enables the real time simulation of Physics in the World. By default, this is disabled and Physics is only
      simulated to reason about it.

      :param real_time: Whether the World should simulate Physics in real time.


   .. py:method:: set_gravity(gravity_vector: typing_extensions.List[float]) -> None
      :abstractmethod:

      Sets the gravity that is used in the World. By default, it is set to the gravity on earth ([0, 0, -9.8]).
       Gravity is given as a vector in x,y,z. Gravity is only applied while simulating Physic.

      :param gravity_vector: The gravity vector that should be used in the World.


   .. py:method:: set_robot_if_not_set(robot: pycram.world_concepts.world_object.Object) -> None

      Sets the robot if it is not set yet.

      :param robot: The Object reference to the Object representing the robot.


   .. py:method:: set_robot(robot: typing_extensions.Union[pycram.world_concepts.world_object.Object, None]) -> None
      :staticmethod:

      Sets the global variable for the robot Object This should be set on spawning the robot.

      :param robot: The Object reference to the Object representing the robot.


   .. py:method:: robot_is_set() -> bool
      :staticmethod:

      Returns whether the robot has been set or not.

      :return: True if the robot has been set, False otherwise.


   .. py:method:: exit() -> None

      Closes the World as well as the prospection world, also collects any other thread that is running.


   .. py:method:: exit_prospection_world_if_exists() -> None

      Exits the prospection world if it exists.


   .. py:method:: disconnect_from_physics_server() -> None
      :abstractmethod:

      Disconnects the world from the physics server.


   .. py:method:: reset_current_world() -> None

      Resets the pose of every object in the World to the pose it was spawned in and sets every joint to 0.


   .. py:method:: reset_robot() -> None

      Sets the robot class variable to None.


   .. py:method:: join_threads() -> None
      :abstractmethod:

      Join any running threads. Useful for example when exiting the world.


   .. py:method:: terminate_world_sync() -> None

      Terminates the world sync thread.


   .. py:method:: save_state(state_id: typing_extensions.Optional[int] = None) -> int

      Returns the id of the saved state of the World. The saved state contains the states of all the objects and
      the state of the physics simulator.

      :return: A unique id of the state


   .. py:method:: save_objects_state(state_id: int) -> None

      Saves the state of all objects in the World according to the given state using the unique state id.

      :param state_id: The unique id representing the state.


   .. py:method:: save_physics_simulator_state() -> int
      :abstractmethod:

      Saves the state of the physics simulator and returns the unique id of the state.

      :return: The unique id representing the state.


   .. py:method:: remove_physics_simulator_state(state_id: int) -> None
      :abstractmethod:

      Removes the state of the physics simulator with the given id.

      :param state_id: The unique id representing the state.


   .. py:method:: restore_physics_simulator_state(state_id: int) -> None
      :abstractmethod:

      Restores the objects and environment state in the physics simulator according to
       the given state using the unique state id.

      :param state_id: The unique id representing the state.


   .. py:method:: get_images_for_target(target_pose: pycram.datastructures.pose.Pose, cam_pose: pycram.datastructures.pose.Pose, size: typing_extensions.Optional[int] = 256) -> typing_extensions.List[numpy.ndarray]

      Calculates the view and projection Matrix and returns 3 images:

      1. An RGB image
      2. A depth image
      3. A segmentation Mask, the segmentation mask indicates for every pixel the visible Object

      :param target_pose: The pose to which the camera should point.
      :param cam_pose: The pose of the camera.
      :param size: The height and width of the images in pixels.
      :return: A list containing an RGB and depth image as well as a segmentation mask, in this order.


   .. py:method:: register_two_objects_collision_callbacks(object_a: pycram.world_concepts.world_object.Object, object_b: pycram.world_concepts.world_object.Object, on_collision_callback: typing_extensions.Callable, on_collision_removal_callback: typing_extensions.Optional[typing_extensions.Callable] = None) -> None

      Registers callback methods for contact between two Objects. There can be a callback for when the two Objects
      get in contact and, optionally, for when they are not in contact anymore.

      :param object_a: An object in the World
      :param object_b: Another object in the World
      :param on_collision_callback: A function that should be called if the objects are in contact
      :param on_collision_removal_callback: A function that should be called if the objects are not in contact


   .. py:method:: add_resource_path(path: str) -> None
      :classmethod:

      Adds a resource path in which the World will search for files. This resource directory is searched if an
      Object is spawned only with a filename.

      :param path: A path in the filesystem in which to search for files.


   .. py:method:: get_prospection_object_for_object(obj: pycram.world_concepts.world_object.Object) -> pycram.world_concepts.world_object.Object

      Returns the corresponding object from the prospection world for a given object in the main world.
       If the given Object is already in the prospection world, it is returned.

      :param obj: The object for which the corresponding object in the prospection World should be found.
      :return: The corresponding object in the prospection world.


   .. py:method:: get_object_for_prospection_object(prospection_object: pycram.world_concepts.world_object.Object) -> pycram.world_concepts.world_object.Object

      Returns the corresponding object from the main World for a given
      object in the prospection world. If the  given object is not in the prospection
      world an error will be raised.

      :param prospection_object: The object for which the corresponding object in the main World should be found.
      :return: The object in the main World.


   .. py:method:: reset_world(remove_saved_states=True) -> None

      Resets the World to the state it was first spawned in.
      All attached objects will be detached, all joints will be set to the
      default position of 0 and all objects will be set to the position and
      orientation in which they were spawned.

      :param remove_saved_states: If the saved states should be removed.


   .. py:method:: remove_saved_states() -> None

      Removes all saved states of the World.


   .. py:method:: update_transforms_for_objects_in_current_world() -> None

      Updates transformations for all objects that are currently in :py:attr:`~pycram.world.World.current_world`.


   .. py:method:: ray_test(from_position: typing_extensions.List[float], to_position: typing_extensions.List[float]) -> int
      :abstractmethod:

      Cast a ray and return the first object hit, if any.

      :param from_position: The starting position of the ray in Cartesian world coordinates.
      :param to_position: The ending position of the ray in Cartesian world coordinates.
      :return: The object id of the first object hit, or -1 if no object was hit.


   .. py:method:: ray_test_batch(from_positions: typing_extensions.List[typing_extensions.List[float]], to_positions: typing_extensions.List[typing_extensions.List[float]], num_threads: int = 1) -> typing_extensions.List[int]
      :abstractmethod:

      Cast a batch of rays and return the result for each of the rays (first object hit, if any. or -1)
       Takes optional argument num_threads to specify the number of threads to use
         to compute the ray intersections for the batch. Specify 0 to let simulator decide, 1 (default) for single
          core execution, 2 or more to select the number of threads to use.

      :param from_positions: The starting positions of the rays in Cartesian world coordinates.
      :param to_positions: The ending positions of the rays in Cartesian world coordinates.
      :param num_threads: The number of threads to use to compute the ray intersections for the batch.


   .. py:method:: create_visual_shape(visual_shape: pycram.datastructures.dataclasses.VisualShape) -> int
      :abstractmethod:

      Creates a visual shape in the physics simulator and returns the unique id of the created shape.

      :param visual_shape: The visual shape to be created, uses the VisualShape dataclass defined in world_dataclasses
      :return: The unique id of the created shape.


   .. py:method:: create_multi_body_from_visual_shapes(visual_shape_ids: typing_extensions.List[int], pose: pycram.datastructures.pose.Pose) -> int

      Creates a multi body from visual shapes in the physics simulator and returns the unique id of the created
      multi body.

      :param visual_shape_ids: The ids of the visual shapes that should be used to create the multi body.
      :param pose: The pose of the origin of the multi body relative to the world frame.
      :return: The unique id of the created multi body.


   .. py:method:: create_multi_body(multi_body: pycram.datastructures.dataclasses.MultiBody) -> int
      :abstractmethod:

      Creates a multi body in the physics simulator and returns the unique id of the created multi body. The multibody
      is created by joining multiple links/shapes together with joints.

      :param multi_body: The multi body to be created, uses the MultiBody dataclass defined in world_dataclasses.
      :return: The unique id of the created multi body.


   .. py:method:: create_box_visual_shape(shape_data: pycram.datastructures.dataclasses.BoxVisualShape) -> int
      :abstractmethod:

      Creates a box visual shape in the physics simulator and returns the unique id of the created shape.

      :param shape_data: The parameters that define the box visual shape to be created, uses the BoxVisualShape dataclass defined in world_dataclasses.
      :return: The unique id of the created shape.


   .. py:method:: create_cylinder_visual_shape(shape_data: pycram.datastructures.dataclasses.CylinderVisualShape) -> int
      :abstractmethod:

      Creates a cylinder visual shape in the physics simulator and returns the unique id of the created shape.

      :param shape_data: The parameters that define the cylinder visual shape to be created, uses the CylinderVisualShape dataclass defined in world_dataclasses.
      :return: The unique id of the created shape.


   .. py:method:: create_sphere_visual_shape(shape_data: pycram.datastructures.dataclasses.SphereVisualShape) -> int
      :abstractmethod:

      Creates a sphere visual shape in the physics simulator and returns the unique id of the created shape.

      :param shape_data: The parameters that define the sphere visual shape to be created, uses the SphereVisualShape dataclass defined in world_dataclasses.
      :return: The unique id of the created shape.


   .. py:method:: create_capsule_visual_shape(shape_data: pycram.datastructures.dataclasses.CapsuleVisualShape) -> int
      :abstractmethod:

      Creates a capsule visual shape in the physics simulator and returns the unique id of the created shape.

      :param shape_data: The parameters that define the capsule visual shape to be created, uses the CapsuleVisualShape dataclass defined in world_dataclasses.
      :return: The unique id of the created shape.


   .. py:method:: create_plane_visual_shape(shape_data: pycram.datastructures.dataclasses.PlaneVisualShape) -> int
      :abstractmethod:

      Creates a plane visual shape in the physics simulator and returns the unique id of the created shape.

      :param shape_data: The parameters that define the plane visual shape to be created, uses the PlaneVisualShape dataclass defined in world_dataclasses.
      :return: The unique id of the created shape.


   .. py:method:: create_mesh_visual_shape(shape_data: pycram.datastructures.dataclasses.MeshVisualShape) -> int
      :abstractmethod:

      Creates a mesh visual shape in the physics simulator and returns the unique id of the created shape.

      :param shape_data: The parameters that define the mesh visual shape to be created,
      uses the MeshVisualShape dataclass defined in world_dataclasses.
      :return: The unique id of the created shape.


   .. py:method:: add_text(text: str, position: typing_extensions.List[float], orientation: typing_extensions.Optional[typing_extensions.List[float]] = None, size: float = 0.1, color: typing_extensions.Optional[pycram.datastructures.dataclasses.Color] = Color(), life_time: typing_extensions.Optional[float] = 0, parent_object_id: typing_extensions.Optional[int] = None, parent_link_id: typing_extensions.Optional[int] = None) -> int
      :abstractmethod:

      Adds text to the world.

      :param text: The text to be added.
      :param position: The position of the text in the world.
      :param orientation: By default, debug text will always face the camera, automatically rotation. By specifying a text orientation (quaternion), the orientation will be fixed in world space or local space (when parent is specified).
      :param size: The size of the text.
      :param color: The color of the text.
      :param life_time: The lifetime in seconds of the text to remain in the world, if 0 the text will remain in the world until it is removed manually.
      :param parent_object_id: The id of the object to which the text should be attached.
      :param parent_link_id: The id of the link to which the text should be attached.
      :return: The id of the added text.


   .. py:method:: remove_text(text_id: typing_extensions.Optional[int] = None) -> None
      :abstractmethod:

      Removes text from the world using the given id. if no id is given all text will be removed.

      :param text_id: The id of the text to be removed.


   .. py:method:: enable_joint_force_torque_sensor(obj: pycram.world_concepts.world_object.Object, fts_joint_idx: int) -> None
      :abstractmethod:

      You can enable a joint force/torque sensor in each joint. Once enabled, if you perform
      a simulation step, the get_joint_reaction_force_torque will report the joint reaction forces in
      the fixed degrees of freedom: a fixed joint will measure all 6DOF joint forces/torques.
      A revolute/hinge joint force/torque sensor will measure 5DOF reaction forces along all axis except
      the hinge axis. The applied force by a joint motor is available through get_applied_joint_motor_torque.

      :param obj: The object in which the joint is located.
      :param fts_joint_idx: The index of the joint for which the force torque sensor should be enabled.


   .. py:method:: disable_joint_force_torque_sensor(obj: pycram.world_concepts.world_object.Object, joint_id: int) -> None
      :abstractmethod:

      Disables the force torque sensor of a joint.

      :param obj: The object in which the joint is located.
      :param joint_id: The id of the joint for which the force torque sensor should be disabled.


   .. py:method:: get_joint_reaction_force_torque(obj: pycram.world_concepts.world_object.Object, joint_id: int) -> typing_extensions.List[float]
      :abstractmethod:

      Returns the joint reaction forces and torques of the specified joint.

      :param obj: The object in which the joint is located.
      :param joint_id: The id of the joint for which the force torque should be returned.
      :return: The joint reaction forces and torques of the specified joint.


   .. py:method:: get_applied_joint_motor_torque(obj: pycram.world_concepts.world_object.Object, joint_id: int) -> float
      :abstractmethod:

      Returns the applied torque by a joint motor.

      :param obj: The object in which the joint is located.
      :param joint_id: The id of the joint for which the applied motor torque should be returned.
      :return: The applied torque by a joint motor.


   .. py:method:: __del__()



.. py:class:: UseProspectionWorld


   An environment for using the prospection world, while in this environment the :py:attr:`~World.current_world`
   variable will point to the prospection world.

   .. rubric:: Example

   with UseProspectionWorld():
       NavigateAction.Action([[1, 0, 0], [0, 0, 0, 1]]).perform()

   .. py:attribute:: WAIT_TIME_FOR_ADDING_QUEUE
      :value: 20

      The time in seconds to wait for the adding queue to be ready.

   .. py:method:: __enter__()

      This method is called when entering the with block, it will set the current world to the prospection world


   .. py:method:: __exit__(*args)

      This method is called when exiting the with block, it will restore the previous world to be the current world.



.. py:class:: WorldSync(world: World, prospection_world: World)


   Bases: :py:obj:`threading.Thread`

   Synchronizes the state between the World and its prospection world.
   Meaning the cartesian and joint position of everything in the prospection world will be
   synchronized with the main World.
   Adding and removing objects is done via queues, such that loading times of objects
   in the prospection world does not affect the World.
   The class provides the possibility to pause the synchronization, this can be used
   if reasoning should be done in the prospection world.

   This constructor should always be called with keyword arguments. Arguments are:

   *group* should be None; reserved for future extension when a ThreadGroup
   class is implemented.

   *target* is the callable object to be invoked by the run()
   method. Defaults to None, meaning nothing is called.

   *name* is the thread name. By default, a unique name is constructed of
   the form "Thread-N" where N is a small decimal number.

   *args* is the argument tuple for the target invocation. Defaults to ().

   *kwargs* is a dictionary of keyword arguments for the target
   invocation. Defaults to {}.

   If a subclass overrides the constructor, it must make sure to invoke
   the base class constructor (Thread.__init__()) before doing anything
   else to the thread.


   .. py:method:: run(wait_time_as_n_simulation_steps: typing_extensions.Optional[int] = 1)

      Main method of the synchronization, this thread runs in a loop until the
      terminate flag is set.
      While this loop runs it continuously checks the cartesian and joint position of
      every object in the World and updates the corresponding object in the
      prospection world. When there are entries in the adding or removing queue the corresponding objects will
      be added or removed in the same iteration.

      :param wait_time_as_n_simulation_steps: The time in simulation steps to wait between each iteration of
       the syncing loop.


   .. py:method:: check_for_pause() -> None

      Checks if :py:attr:`~self.pause_sync` is true and sleeps this thread until it isn't anymore.


   .. py:method:: check_for_equal() -> bool

      Checks if both Worlds have the same state, meaning all objects are in the same position.
      This is currently not used, but might be used in the future if synchronization issues worsen.

      :return: True if both Worlds have the same state, False otherwise.



