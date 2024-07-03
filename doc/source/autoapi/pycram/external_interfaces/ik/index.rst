:py:mod:`pycram.external_interfaces.ik`
=======================================

.. py:module:: pycram.external_interfaces.ik


Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   pycram.external_interfaces.ik._make_request_msg
   pycram.external_interfaces.ik.call_ik
   pycram.external_interfaces.ik.try_to_reach_with_grasp
   pycram.external_interfaces.ik.apply_grasp_orientation_to_pose
   pycram.external_interfaces.ik.try_to_reach
   pycram.external_interfaces.ik.request_ik
   pycram.external_interfaces.ik.request_kdl_ik
   pycram.external_interfaces.ik.request_giskard_ik



.. py:function:: _make_request_msg(root_link: str, tip_link: str, target_pose: pycram.datastructures.pose.Pose, robot_object: pycram.world_concepts.world_object.Object, joints: typing_extensions.List[str]) -> moveit_msgs.msg.PositionIKRequest

   Generates an ik request message for the kdl_ik_service. The message is
   of the type moveit_msgs/PositionIKRequest and contains all information
   contained in the parameter.

   :param root_link: The first link of the chain of joints to be altered
   :param tip_link: The last link of the chain of joints to be altered
   :param target_pose: Target pose for which the message should be created
   :param robot_object: The robot for which the ik should be generated
   :param joints: A list of joint names between the root_link and tip_link that should be altered.
   :return: A moveit_msgs/PositionIKRequest message containing all the information from the parameter


.. py:function:: call_ik(root_link: str, tip_link: str, target_pose: pycram.datastructures.pose.Pose, robot_object: pycram.world_concepts.world_object.Object, joints: typing_extensions.List[str]) -> typing_extensions.List[float]

   Sends a request to the kdl_ik_service and returns the solution.
   Note that the robot in robot_object should be identical to the robot description
   uploaded to the parameter server. Furthermore, note that the root_link and
   tip_link are the links attached to the first and last joints in the joints list.

   :param root_link: The first link of the chain of joints to be altered
   :param tip_link: The last link in the chain of joints to be altered
   :param target_pose: The target pose in frame of root link second is the orientation as quaternion in world coordinate frame
   :param robot_object: The robot object for which the ik solution should be generated
   :param joints: A list of joint name that should be altered
   :return: The solution that was generated as a list of joint values corresponding to the order of joints given


.. py:function:: try_to_reach_with_grasp(pose_or_object: typing_extensions.Union[pycram.datastructures.pose.Pose, pycram.world_concepts.world_object.Object], prospection_robot: pycram.world_concepts.world_object.Object, gripper_name: str, grasp: str) -> typing_extensions.Union[pycram.datastructures.pose.Pose, None]

   Checks if the robot can reach a given position with a specific grasp orientation.
   To determine this the inverse kinematics are calculated and applied.

   :param pose_or_object: The position and rotation or Object for which reachability should be checked or an Object
   :param prospection_robot: The robot that should reach for the position
   :param gripper_name: The name of the end effector
   :param grasp: The grasp type with which the object should be grasped


.. py:function:: apply_grasp_orientation_to_pose(grasp: str, pose: pycram.datastructures.pose.Pose) -> pycram.datastructures.pose.Pose

   Applies the orientation of a grasp to a given pose. This is done by using the grasp orientation
   of the given grasp and applying it to the given pose.

   :param grasp: The name of the grasp
   :param pose: The pose to which the grasp orientation should be applied


.. py:function:: try_to_reach(pose_or_object: typing_extensions.Union[pycram.datastructures.pose.Pose, pycram.world_concepts.world_object.Object], prospection_robot: pycram.world_concepts.world_object.Object, gripper_name: str) -> typing_extensions.Union[pycram.datastructures.pose.Pose, None]

   Tries to reach a given position with a given robot. This is done by calculating the inverse kinematics.

   :param pose_or_object: The position and rotation or Object for which reachability should be checked.
   :param prospection_robot: The robot that should be used to check for reachability, should be the one in the prospection world
   :param gripper_name: Name of the gripper tool frame
   :return: The pose at which the robot should stand or None if the target is not reachable


.. py:function:: request_ik(target_pose: pycram.datastructures.pose.Pose, robot: pycram.world_concepts.world_object.Object, joints: typing_extensions.List[str], gripper: str) -> typing_extensions.Tuple[pycram.datastructures.pose.Pose, typing_extensions.Dict[str, float]]

   Top-level method to request ik solution for a given pose. This method will check if the giskard node is running
   and if so will call the giskard service. If the giskard node is not running the kdl_ik_service will be called.

   :param target_pose: Pose of the end-effector for which an ik solution should be found
   :param robot: The robot object which should be used
   :param joints: A list of joints that should be used in computation, this is only relevant for the kdl_ik_service
   :param gripper: Name of the tool frame which should grasp, this should be at the end of the given joint chain
   :return: A Pose at which the robt should stand as well as a dictionary of joint values


.. py:function:: request_kdl_ik(target_pose: pycram.datastructures.pose.Pose, robot: pycram.world_concepts.world_object.Object, joints: typing_extensions.List[str], gripper: str) -> typing_extensions.Dict[str, float]

   Top-level method to request ik solution for a given pose. Before calling the ik service the links directly before
   and after the joint chain will be queried and the target_pose will be transformed into the frame of the root_link.
   Afterward, the offset between the tip_link and end effector will be calculated and taken into account. Lastly the
   ik service is called and the result returned

   :param target_pose: Pose for which an ik solution should be found
   :param robot: Robot object which should be used
   :param joints: List of joints that should be used in computation
   :param gripper: Name of the gripper which should grasp, this should be at the end of the given joint chain
   :return: A list of joint values


.. py:function:: request_giskard_ik(target_pose: pycram.datastructures.pose.Pose, robot: pycram.world_concepts.world_object.Object, gripper: str) -> typing_extensions.Tuple[pycram.datastructures.pose.Pose, typing_extensions.Dict[str, float]]

   Calls giskard in projection mode and queries the ik solution for a full body ik solution. This method will
   try to drive the robot directly to a pose from which the target_pose is reachable for the end effector. If there
   are obstacles in the way this method will fail. In this case please use the GiskardLocation designator.

   :param target_pose: Pose at which the end effector should be moved.
   :param robot: Robot object which should be used.
   :param gripper: Name of the tool frame which should grasp, this should be at the end of the given joint chain.
   :return: A list of joint values.


