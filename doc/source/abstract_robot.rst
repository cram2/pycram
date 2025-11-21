Astract Robot Overview
======================
To define and manage semantic information about robots PyCRAM uses the  ``AbstractRobot`` class of the semantic
digital twin. Specific instances of the ``AbstractRobot`` class are part of the Context that is passed to the Plan on
creation.


The ``AbstractRobot`` class defines a semantic, high‑level model of a robot as it appears in a world description. Rather
than focusing on actuation details or low‑level control, it organizes the robot’s physical and functional structure into
coherent parts such as kinematic chains, manipulators, sensors, and the torso. This abstraction lets downstream components
reason about what the robot is, what it has, and how its parts relate, without needing to know how individual joints or links are implemented.

The Problem It Solves
---------------------
Robot software often mixes structural knowledge (which bodies and joints form an arm, where the gripper’s tool frame is,
what cameras are available) with control or task logic. That coupling complicates reuse and makes it hard to write generic
algorithms that operate across different platforms. ``AbstractRobot`` solves this by providing a uniform, semantic view of
a robot that is reconstructible from a ``World`` description. The result is a clean separation: semantic structure and
capabilities are captured once, while planners, controllers, and task logic can query that structure in a device‑agnostic way.

Core Concepts and Terminology
-----------------------------
A robot is a ``RootedSemanticAnnotation`` with a ``root`` body typically representing its base. From this root, the robot
is composed of structured semantic annotations of its parts. A ``KinematicChain`` is a contiguous sequence of ``KinematicStructureEntity``
objects from a ``root`` body to a ``tip`` body. A chain may carry a ``Manipulator`` such as a gripper and it may carry one or more ``Sensor``
instances such as cameras. When a chain contains both, it participates in both the manipulator and sensor perspectives of the robot.
A ``Torso`` is a special kinematic chain that connects the base to other chains, often without a manipulator. A ``Neck`` is a
specialized kinematic chain that connects the head and its sensors and can optionally identify specific pitch and yaw bodies.

A ``Manipulator`` is a semantic description of an end effector that always has a ``tool_frame``. A ``ParallelGripper`` is a
concrete manipulator that holds exactly one ``Finger`` and one ``thumb`` and defines a ``front_facing_orientation`` and ``front_facing_axis``
for tasks such as approach planning. A ``Sensor`` is any perceptual device; ``Camera`` is a concrete sensor that adds a
``forward_facing_axis``, a ``FieldOfView``, and typical operating height bounds. All of these are semantic annotations that
point back to the shared world model through ``_world`` so they can derive bodies, connections, and regions.

How Parts Fit Together
----------------------
An ``AbstractRobot`` instance can hold at most one ``torso``, zero or more ``Manipulator`` objects, and zero or more ``Sensor``
objects. It also keeps two orthogonal sets of chains: ``manipulator_chains`` for any kinematic chains that include a
manipulator and ``sensor_chains`` for any chains that include one or more sensors. A single chain can appear in both sets
if it carries both a manipulator and sensors, which reflects real robots where tools and cameras share the same arm.

Assignment is explicit and safe by design. Each semantic sub‑component implements ``assign_to_robot`` and refuses to be
attached to more than one robot at a time. High‑level methods on the robot—``add_manipulator``, ``add_sensor``, ``add_torso``,
and ``add_kinematic_chain``—add components to the robot, register them as semantic annotations in the world, and delegate
to ``assign_to_robot``. When adding a kinematic chain, if it neither contains a manipulator nor sensors, a warning is emitted
to prevent accidental creation of structurally empty chains. These safeguards make interfaces hard to misuse and keep the semantic model consistent.

Within a chain, bodies, connections, and regions are derived from the world graph between the chain’s ``root`` and ``tip``.
The world provides utilities like ``compute_chain_of_kinematic_structure_entities`` and ``compute_chain_of_connections``,
which the chain exposes as ``kinematic_structure_entities``, ``bodies``, ``regions``, and ``connections``. This approach ensures
a single source of truth: the chain is a semantic view over the underlying kinematic structure defined in the world.

Interaction with the World and Motion Control
---------------------------------------------
Because ``AbstractRobot`` is rooted in the world model, it can expose cross‑cutting capabilities in a uniform way. The
``controlled_connections`` property returns the intersection of the world’s controlled connections and the robot’s own
connections. The ``drive`` property exposes an ``OmniDrive`` connection if the robot’s base is connected that way, allowing
higher‑level code to discover how to command base motion without hard‑coding link names.

For safety and performance tuning, the robot provides a convenience method to tighten velocity limits of all one
degree‑of‑freedom active connections. The method ``tighten_dof_velocity_limits_of_1dof_connections`` accepts a mapping
from ``ActiveConnection1DOF`` to a scalar limit and overwrites both lower and upper velocity bounds via ``DerivativeMap``.
The default collision checking configuration is available via ``default_collision_config``, and individual robots may load
additional collision constraints from SRDF using ``load_srdf``.

Construction and Extensibility
------------------------------
Robots are usually created via the ``from_world`` class method, which constructs the semantic structure by looking up
bodies and connections in a ``World``. This factory method encapsulates all name lookups and wiring so that user code can
simply request a robot by type. Subclasses are responsible for implementing ``from_world`` and may also implement ``load_srdf``
to bring in collision pairs or other model tweaks. Additional semantic elements can be added by defining new subclasses of
``SemanticRobotAnnotation`` that follow the same assignment and world‑integration patterns used by ``KinematicChain``, ``Manipulator``, and ``Sensor``.

Concrete Examples: PR2 and HSRB
-------------------------------
The ``PR2`` subclass illustrates a dual‑arm mobile manipulator. It names its base ``root`` as ``base_footprint``. Each arm is a
``KinematicChain`` from the torso lift link to the wrist, and each arm carries a ``ParallelGripper`` manipulator with one
``Finger`` and one ``thumb``. The gripper’s tool frame and front‑facing axis are specified so that grasp planners have a
consistent convention. The PR2’s head is represented as a ``Neck`` carrying a single ``Camera``, complete with field of view
and plausible operating heights. It defines a ``Torso`` and loads an SRDF to configure collision checking. Finally, the
example tightens selected joint velocity limits for safety and realism.

The ``HSRB`` subclass models a single‑arm service robot. Its arm kinematic chain includes both a ``ParallelGripper`` and
an additional ``Camera`` mounted on the hand, demonstrating how one chain can be both a manipulator chain and a sensor chain.
The head is represented by a ``Neck`` that carries multiple cameras: a center camera, stereo cameras, and an RGB‑D sensor.
The torso connects the base to the lift link. As with PR2, the HSRB subclass is built entirely by ``from_world`` using
URDF‑consistent link names to ensure it matches the parsed world model.

Design Notes
------------
The design emphasizes strong ownership and clear interfaces. Components attach to exactly one robot and register themselves
as semantic annotations in the shared world, which aids discoverability and avoids duplication. By expressing pose
conventions explicitly—like ``tool_frame`` for manipulators and forward‑facing axes for cameras—the model minimizes ambiguity
that would otherwise leak into algorithms. The clear separation between ``Manipulator``, ``Sensor``, and ``KinematicChain``
aligns with SOLID principles and keeps concerns well factored. Where possible, common operations such as discovering controlled
connections or tightening limits are encapsulated as small, intention‑revealing methods.

Key takeaways
-------------
- ``AbstractRobot`` is a semantic, world‑backed description of a robot’s structure and capabilities.
- Parts are composed from ``KinematicChain``, ``Manipulator``, ``Sensor``, and ``Torso``, each with explicit ownership and world integration.
- Chains can simultaneously be manipulator chains and sensor chains, reflecting real hardware layouts.
- The API provides safe attachment, discoverability of drives and controlled joints, and helpers for collision and limit configuration.
- PR2 and HSRB show how diverse robots fit the same abstraction, enabling reusable, robot‑agnostic logic.