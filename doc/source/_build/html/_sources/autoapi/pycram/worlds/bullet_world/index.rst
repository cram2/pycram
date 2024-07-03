:py:mod:`pycram.worlds.bullet_world`
====================================

.. py:module:: pycram.worlds.bullet_world


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.worlds.bullet_world.BulletWorld
   pycram.worlds.bullet_world.Gui




Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.worlds.bullet_world.Link
   pycram.worlds.bullet_world.RootLink
   pycram.worlds.bullet_world.Joint


.. py:data:: Link

   

.. py:data:: RootLink

   

.. py:data:: Joint

   

.. py:class:: BulletWorld(mode: pycram.datastructures.enums.WorldMode = WorldMode.DIRECT, is_prospection_world: bool = False, sim_frequency=240)


   Bases: :py:obj:`pycram.datastructures.world.World`

   This class represents a BulletWorld, which is a simulation environment that uses the Bullet Physics Engine. This
   class is the main interface to the Bullet Physics Engine and should be used to spawn Objects, simulate Physic and
   manipulate the Bullet World.

   Creates a new simulation, the type decides of the simulation should be a rendered window or just run in the
   background. There can only be one rendered simulation.
   The BulletWorld object also initializes the Events for attachment, detachment and for manipulating the world.

   :param mode: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default is "GUI"
   :param is_prospection_world: For internal usage, decides if this BulletWorld should be used as a shadow world.

   .. py:attribute:: extension
      :type: str

      

   .. py:attribute:: get_object_number_of_joints

      

   .. py:method:: _init_world(mode: pycram.datastructures.enums.WorldMode)


   .. py:method:: load_object_and_get_id(path: typing_extensions.Optional[str] = None, pose: typing_extensions.Optional[pycram.datastructures.pose.Pose] = None) -> int


   .. py:method:: _load_object_and_get_id(path: str, pose: pycram.datastructures.pose.Pose) -> int


   .. py:method:: remove_object_from_simulator(obj: pycram.world_concepts.world_object.Object) -> None


   .. py:method:: remove_object_by_id(obj_id: int) -> None


   .. py:method:: add_constraint(constraint: pycram.world_concepts.constraints.Constraint) -> int


   .. py:method:: remove_constraint(constraint_id)


   .. py:method:: get_joint_position(joint: pycram.object_descriptors.urdf.ObjectDescription.Joint) -> float


   .. py:method:: get_object_joint_names(obj: pycram.world_concepts.world_object.Object) -> typing_extensions.List[str]


   .. py:method:: get_link_pose(link: pycram.object_descriptors.urdf.ObjectDescription.Link) -> pycram.datastructures.pose.Pose


   .. py:method:: get_object_link_names(obj: pycram.world_concepts.world_object.Object) -> typing_extensions.List[str]


   .. py:method:: get_object_number_of_links(obj: pycram.world_concepts.world_object.Object) -> int


   .. py:method:: perform_collision_detection() -> None


   .. py:method:: get_object_contact_points(obj: pycram.world_concepts.world_object.Object) -> typing_extensions.List

      For a more detailed explanation of the
       returned list please look at:
       `PyBullet Doc <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#>`_


   .. py:method:: get_contact_points_between_two_objects(obj1: pycram.world_concepts.world_object.Object, obj2: pycram.world_concepts.world_object.Object) -> typing_extensions.List


   .. py:method:: reset_joint_position(joint: pycram.object_descriptors.urdf.ObjectDescription.Joint, joint_position: str) -> None


   .. py:method:: reset_object_base_pose(obj: pycram.world_concepts.world_object.Object, pose: pycram.datastructures.pose.Pose) -> None


   .. py:method:: step()


   .. py:method:: get_object_pose(obj: pycram.world_concepts.world_object.Object) -> pycram.datastructures.pose.Pose


   .. py:method:: set_link_color(link: pycram.object_descriptors.urdf.ObjectDescription.Link, rgba_color: pycram.datastructures.dataclasses.Color)


   .. py:method:: get_link_color(link: pycram.object_descriptors.urdf.ObjectDescription.Link) -> pycram.datastructures.dataclasses.Color


   .. py:method:: get_colors_of_object_links(obj: pycram.world_concepts.world_object.Object) -> typing_extensions.Dict[str, pycram.datastructures.dataclasses.Color]


   .. py:method:: get_object_axis_aligned_bounding_box(obj: pycram.world_concepts.world_object.Object) -> pycram.datastructures.dataclasses.AxisAlignedBoundingBox


   .. py:method:: get_link_axis_aligned_bounding_box(link: pycram.object_descriptors.urdf.ObjectDescription.Link) -> pycram.datastructures.dataclasses.AxisAlignedBoundingBox


   .. py:method:: set_realtime(real_time: bool) -> None


   .. py:method:: set_gravity(gravity_vector: typing_extensions.List[float]) -> None


   .. py:method:: disconnect_from_physics_server()


   .. py:method:: join_threads()

      Joins the GUI thread if it exists.


   .. py:method:: join_gui_thread_if_exists()


   .. py:method:: save_physics_simulator_state() -> int


   .. py:method:: restore_physics_simulator_state(state_id)


   .. py:method:: remove_physics_simulator_state(state_id: int)


   .. py:method:: add_vis_axis(pose: pycram.datastructures.pose.Pose, length: typing_extensions.Optional[float] = 0.2) -> None

      Creates a Visual object which represents the coordinate frame at the given
      position and orientation. There can be an unlimited amount of vis axis objects.

      :param pose: The pose at which the axis should be spawned
      :param length: Optional parameter to configure the length of the axes


   .. py:method:: remove_vis_axis() -> None

      Removes all spawned vis axis objects that are currently in this BulletWorld.


   .. py:method:: ray_test(from_position: typing_extensions.List[float], to_position: typing_extensions.List[float]) -> int


   .. py:method:: ray_test_batch(from_positions: typing_extensions.List[typing_extensions.List[float]], to_positions: typing_extensions.List[typing_extensions.List[float]], num_threads: int = 1) -> typing_extensions.List[int]


   .. py:method:: create_visual_shape(visual_shape: pycram.datastructures.dataclasses.VisualShape) -> int


   .. py:method:: create_multi_body(multi_body: pycram.datastructures.dataclasses.MultiBody) -> int


   .. py:method:: get_images_for_target(target_pose: pycram.datastructures.pose.Pose, cam_pose: pycram.datastructures.pose.Pose, size: typing_extensions.Optional[int] = 256) -> typing_extensions.List[numpy.ndarray]


   .. py:method:: add_text(text: str, position: typing_extensions.List[float], orientation: typing_extensions.Optional[typing_extensions.List[float]] = None, size: typing_extensions.Optional[float] = None, color: typing_extensions.Optional[pycram.datastructures.dataclasses.Color] = Color(), life_time: typing_extensions.Optional[float] = 0, parent_object_id: typing_extensions.Optional[int] = None, parent_link_id: typing_extensions.Optional[int] = None) -> int


   .. py:method:: remove_text(text_id: typing_extensions.Optional[int] = None) -> None


   .. py:method:: enable_joint_force_torque_sensor(obj: pycram.world_concepts.world_object.Object, fts_joint_idx: int) -> None


   .. py:method:: disable_joint_force_torque_sensor(obj: pycram.world_concepts.world_object.Object, joint_id: int) -> None


   .. py:method:: get_joint_reaction_force_torque(obj: pycram.world_concepts.world_object.Object, joint_id: int) -> typing_extensions.List[float]


   .. py:method:: get_applied_joint_motor_torque(obj: pycram.world_concepts.world_object.Object, joint_id: int) -> float



.. py:class:: Gui(world: pycram.datastructures.world.World, mode: pycram.datastructures.enums.WorldMode)


   Bases: :py:obj:`threading.Thread`

   For internal use only. Creates a new thread for the physics simulation that is active until closed by
   :func:`~World.exit`
   Also contains the code for controlling the camera.

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


   .. py:method:: run()

      Initializes the new simulation and checks in an endless loop
      if it is still active. If it is the thread will be suspended for 1/80 seconds, if it is not the method and
      thus the thread terminates. The loop also checks for mouse and keyboard inputs to control the camera.



