:py:mod:`pycram.robot_description`
==================================

.. py:module:: pycram.robot_description


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.robot_description.RobotDescriptionManager
   pycram.robot_description.RobotDescription
   pycram.robot_description.KinematicChainDescription
   pycram.robot_description.CameraDescription
   pycram.robot_description.EndEffectorDescription




.. py:class:: RobotDescriptionManager


   Singleton class to manage multiple robot descriptions. Stores all robot descriptions and loads a robot description
   according to the name of the loaded robot.

   Initialize the RobotDescriptionManager, if no instance exists a new instance is created.

   .. py:attribute:: _instance

      

   .. py:method:: load_description(name: str)

      Loads a robot description according to the name of the robot. This required that a robot description with the
      corresponding name is registered.

      :param name: Name of the robot to which the description should be loaded.
      :return: The loaded robot description.


   .. py:method:: register_description(description: RobotDescription)

      Register a robot description to the RobotDescriptionManager. The description is stored with the name of the
      description as key. This will later be used to load the description.

      :param description: RobotDescription to register.



.. py:class:: RobotDescription(name: str, base_link: str, torso_link: str, torso_joint: str, urdf_path: str)


   Base class of a robot description. Contains all necessary information about a robot, like the URDF, the base link,
   the torso link and joint, the kinematic chains and cameras.

   Initialize the RobotDescription. The URDF is loaded from the given path and used as basis for the kinematic
   chains.

   :param name: Name of the robot
   :param base_link: Base link of the robot, meaning the first link in the URDF
   :param torso_link: Torso link of the robot
   :param torso_joint: Torso joint of the robot, this is the joint that moves the torso upwards if there is one
   :param urdf_path: Path to the URDF file of the robot

   .. py:attribute:: current_robot_description
      :type: RobotDescription

      

   .. py:method:: add_kinematic_chain_description(chain: KinematicChainDescription)

      Adds a KinematicChainDescription object to the RobotDescription. The chain is stored with the name of the chain
      as key.

      :param chain: KinematicChainDescription object to add


   .. py:method:: add_kinematic_chain(name: str, start_link: str, end_link: str)

      Creates and adds a KinematicChainDescription object to the RobotDescription.

      :param name: Name of the KinematicChainDescription object
      :param start_link: First link of the chain
      :param end_link: Last link of the chain


   .. py:method:: add_camera_description(camera: CameraDescription)

      Adds a CameraDescription object to the RobotDescription. The camera is stored with the name of the camera as key.
      :param camera: The CameraDescription object to add


   .. py:method:: add_camera(name: str, camera_link: str, minimal_height: float, maximal_height: float)

      Creates and adds a CameraDescription object to the RobotDescription. Minimal and maximal height of the camera are
      relevant if the robot has a moveable torso or the camera is mounted on a moveable part of the robot. Otherwise
      both values can be the same.

      :param name: Name of the CameraDescription object
      :param camera_link: Link of the camera in the URDF
      :param minimal_height: Minimal height of the camera
      :param maximal_height: Maximal height of the camera
      :return:


   .. py:method:: add_grasp_orientation(grasp: pycram.datastructures.enums.Grasp, orientation: typing_extensions.List[float])

      Adds a grasp orientation to the robot description. This is used to define the orientation of the end effector
      when grasping an object.

      :param grasp: Gasp from the Grasp enum which should be added
      :param orientation: List of floats representing the orientation


   .. py:method:: add_grasp_orientations(orientations: typing_extensions.Dict[pycram.datastructures.enums.Grasp, typing_extensions.List[float]])

      Adds multiple grasp orientations to the robot description. This is used to define the orientation of the end effector
      when grasping an object.

      :param orientations: Dictionary of grasp orientations


   .. py:method:: get_manipulator_chains() -> typing_extensions.List[KinematicChainDescription]

      Returns a list of all manipulator chains of the robot which posses an end effector.

      :return: A list of KinematicChainDescription objects


   .. py:method:: get_camera_frame() -> str

      Quick method to get the name of a link of a camera. Uses the first camera in the list of cameras.

      :return: A name of the link of a camera


   .. py:method:: get_default_camera() -> CameraDescription

      Returns the first camera in the list of cameras.

      :return: A CameraDescription object


   .. py:method:: get_static_joint_chain(kinematic_chain_name: str, configuration_name: str)

      Returns the static joint states of a kinematic chain for a specific configuration. When trying to access one of
      the robot arms the function `:func: get_arm_chain` should be used.

      :param kinematic_chain_name:
      :param configuration_name:
      :return:


   .. py:method:: get_parent(name: str) -> str

      Returns the parent of a link or joint in the URDF. Always returns the imeadiate parent, for a link this is a joint
      and vice versa.

      :param name: Name of the link or joint in the URDF
      :return: Name of the parent link or joint


   .. py:method:: get_child(name: str, return_multiple_children: bool = False) -> typing_extensions.Union[str, typing_extensions.List[str]]

      Returns the child of a link or joint in the URDF. Always returns the immediate child, for a link this is a joint
      and vice versa. Since a link can have multiple children, the return_multiple_children parameter can be set to
      True to get a list of all children.

      :param name: Name of the link or joint in the URDF
      :param return_multiple_children: If True, a list of all children is returned
      :return: Name of the child link or joint or a list of all children


   .. py:method:: get_arm_chain(arm: pycram.datastructures.enums.Arms) -> KinematicChainDescription

      Returns the kinematic chain of a specific arm.

      :param arm: Arm for which the chain should be returned
      :return: KinematicChainDescription object of the arm



.. py:class:: KinematicChainDescription(name: str, start_link: str, end_link: str, urdf_object: urdf_parser_py.urdf.URDF, arm_type: pycram.datastructures.enums.Arms = None, include_fixed_joints=False)


   Represents a kinematic chain of a robot. A Kinematic chain is a chain of links and joints that are connected to each
   other and can be moved.

   This class contains all necessary information about the chain, like the start and end
   link, the URDF object and the joints of the chain.

   Initialize the KinematicChainDescription object.

   :param name: Name of the chain
   :param start_link: First link of the chain
   :param end_link: Last link of the chain
   :param urdf_object: URDF object of the robot which is used to get the chain
   :param arm_type: Type of the arm, if the chain is an arm
   :param include_fixed_joints: If True, fixed joints are included in the chain

   .. py:property:: links
      :type: typing_extensions.List[str]

      Property to get the links of the chain.

      :return: List of link names

   .. py:property:: joints
      :type: typing_extensions.List[str]

      Property to get the joints of the chain.

      :return: List of joint names

   .. py:method:: _init_links()

      Initializes the links of the chain by getting the chain from the URDF object.


   .. py:method:: _init_joints()

      Initializes the joints of the chain by getting the chain from the URDF object.


   .. py:method:: get_joints() -> typing_extensions.List[str]

      Returns a list of all joints of the chain.

      :return: List of joint names


   .. py:method:: get_links() -> typing_extensions.List[str]

      Returns a list of all links of the chain.

      :return: List of link names


   .. py:method:: add_static_joint_states(name: str, states: dict)

      Adds static joint states to the chain. These define a specific configuration of the chain.

      :param name: Name of the static joint states
      :param states: Dictionary of joint names and their values


   .. py:method:: get_static_joint_states(name: str) -> typing_extensions.Dict[str, float]

      Returns the dictionary of static joint states for a given name of the static joint states.

      :param name: Name of the static joint states
      :return: Dictionary of joint names and their values


   .. py:method:: get_tool_frame() -> str

      Returns the name of the tool frame of the end effector of this chain, if it has an end effector.

      :return: The name of the link of the tool frame in the URDF.


   .. py:method:: get_static_gripper_state(state: pycram.datastructures.enums.GripperState) -> typing_extensions.Dict[str, float]

      Returns the static joint states for the gripper of the chain.

      :param state: Name of the static joint states
      :return: Dictionary of joint names and their values



.. py:class:: CameraDescription(name: str, link_name: str, minimal_height: float, maximal_height: float, horizontal_angle: float = 20, vertical_angle: float = 20, front_facing_axis: typing_extensions.List[float] = None)


   Represents a camera mounted on a robot. Contains all necessary information about the camera, like the link name,
   minimal and maximal height, horizontal and vertical angle and the front facing axis.

   Initialize the CameraDescription object.

   :param name: Name of the camera
   :param link_name: Name of the link in the URDF
   :param minimal_height: Minimal height the camera can be at
   :param maximal_height: Maximal height the camera can be at
   :param horizontal_angle: Horizontal opening angle of the camera
   :param vertical_angle: Vertical opening angle of the camera
   :param front_facing_axis: Axis along which the camera taking the image


.. py:class:: EndEffectorDescription(name: str, start_link: str, tool_frame: str, urdf_object: urdf_parser_py.urdf.URDF)


   Describes an end effector of robot. Contains all necessary information about the end effector, like the
   base link, the tool frame, the URDF object and the static joint states.

   Initialize the EndEffectorDescription object.

   :param name: Name of the end effector
   :param start_link: Root link of the end effector, every link below this link in the URDF is part of the end effector
   :param tool_frame: Name of the tool frame link in the URDf
   :param urdf_object: URDF object of the robot

   .. py:property:: links
      :type: typing_extensions.List[str]

      Property to get the links of the chain.

      :return: List of link names

   .. py:property:: joints
      :type: typing_extensions.List[str]

      Property to get the joints of the chain.

      :return: List of joint names

   .. py:method:: _init_links_joints()

      Traverses the URDF object to get all links and joints of the end effector below the start link.1


   .. py:method:: add_static_joint_states(name: pycram.datastructures.enums.GripperState, states: dict)

      Adds static joint states to the end effector. These define a specific configuration of the end effector. Like
      open and close configurations of a gripper.

      :param name: Name of the static joint states
      :param states: Dictionary of joint names and their values



