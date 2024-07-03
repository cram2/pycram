:py:mod:`pycram.world_reasoning`
================================

.. py:module:: pycram.world_reasoning


Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   pycram.world_reasoning.stable
   pycram.world_reasoning.contact
   pycram.world_reasoning.get_visible_objects
   pycram.world_reasoning.visible
   pycram.world_reasoning.occluding
   pycram.world_reasoning.reachable
   pycram.world_reasoning.blocking
   pycram.world_reasoning.supporting
   pycram.world_reasoning.link_pose_for_joint_config



.. py:function:: stable(obj: pycram.world_concepts.world_object.Object) -> bool

   Checks if an object is stable in the world. Stable meaning that it's position will not change after simulating
   physics in the World. This will be done by simulating the world for 10 seconds and compare
   the previous coordinates with the coordinates after the simulation.

   :param obj: The object which should be checked
   :return: True if the given object is stable in the world False else


.. py:function:: contact(object1: pycram.world_concepts.world_object.Object, object2: pycram.world_concepts.world_object.Object, return_links: bool = False) -> typing_extensions.Union[bool, typing_extensions.Tuple[bool, typing_extensions.List]]

   Checks if two objects are in contact or not. If the links should be returned then the output will also contain a
   list of tuples where the first element is the link name of 'object1' and the second element is the link name of
   'object2'.

   :param object1: The first object
   :param object2: The second object
   :param return_links: If the respective links on the objects that are in contact should be returned.
   :return: True if the two objects are in contact False else. If links should be returned a list of links in contact


.. py:function:: get_visible_objects(camera_pose: pycram.datastructures.pose.Pose, front_facing_axis: typing_extensions.Optional[typing_extensions.List[float]] = None) -> typing_extensions.Tuple[numpy.ndarray, pycram.datastructures.pose.Pose]

   Returns a segmentation mask of the objects that are visible from the given camera pose and the front facing axis.

   :param camera_pose: The pose of the camera in world coordinate frame.
   :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
   :return: A segmentation mask of the objects that are visible and the pose of the point at exactly 2 meters in front of the camera in the direction of the front facing axis with respect to the world coordinate frame.


.. py:function:: visible(obj: pycram.world_concepts.world_object.Object, camera_pose: pycram.datastructures.pose.Pose, front_facing_axis: typing_extensions.Optional[typing_extensions.List[float]] = None, threshold: float = 0.8) -> bool

   Checks if an object is visible from a given position. This will be achieved by rendering the object
   alone and counting the visible pixel, then rendering the complete scene and compare the visible pixels with the
   absolut count of pixels.

   :param obj: The object for which the visibility should be checked
   :param camera_pose: The pose of the camera in map frame
   :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
   :param threshold: The minimum percentage of the object that needs to be visible for this method to return true.
   :return: True if the object is visible from the camera_position False if not


.. py:function:: occluding(obj: pycram.world_concepts.world_object.Object, camera_pose: pycram.datastructures.pose.Pose, front_facing_axis: typing_extensions.Optional[typing_extensions.List[float]] = None) -> typing_extensions.List[pycram.world_concepts.world_object.Object]

   Lists all objects which are occluding the given object. This works similar to 'visible'.
   First the object alone will be rendered and the position of the pixels of the object in the picture will be saved.
   After that the complete scene will be rendered and the previous saved pixel positions will be compared to the
   actual pixels, if in one pixel another object is visible ot will be saved as occluding.

   :param obj: The object for which occlusion should be checked
   :param camera_pose: The pose of the camera in world coordinate frame
   :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
   :return: A list of occluding objects


.. py:function:: reachable(pose_or_object: typing_extensions.Union[pycram.world_concepts.world_object.Object, pycram.datastructures.pose.Pose], robot: pycram.world_concepts.world_object.Object, gripper_name: str, threshold: float = 0.01) -> bool

   Checks if the robot can reach a given position. To determine this the inverse kinematics are
   calculated and applied. Afterward the distance between the position and the given end effector is calculated, if
   it is smaller than the threshold the reasoning query returns True, if not it returns False.

   :param pose_or_object: The position and rotation or Object for which reachability should be checked or an Object
   :param robot: The robot that should reach for the position
   :param gripper_name: The name of the end effector
   :param threshold: The threshold between the end effector and the position.
   :return: True if the end effector is closer than the threshold to the target position, False in every other case


.. py:function:: blocking(pose_or_object: typing_extensions.Union[pycram.world_concepts.world_object.Object, pycram.datastructures.pose.Pose], robot: pycram.world_concepts.world_object.Object, gripper_name: str, grasp: str = None) -> typing_extensions.Union[typing_extensions.List[pycram.world_concepts.world_object.Object], None]

   Checks if any objects are blocking another object when a robot tries to pick it. This works
   similar to the reachable predicate. First the inverse kinematics between the robot and the object will be
   calculated and applied. Then it will be checked if the robot is in contact with any object except the given one.
   If the given pose or Object is not reachable None will be returned

   :param pose_or_object: The object or pose for which blocking objects should be found
   :param robot: The robot Object who reaches for the object
   :param gripper_name: The name of the end effector of the robot
   :param grasp: The grasp type with which the object should be grasped
   :return: A list of objects the robot is in collision with when reaching for the specified object or None if the pose or object is not reachable.


.. py:function:: supporting(object1: pycram.world_concepts.world_object.Object, object2: pycram.world_concepts.world_object.Object) -> bool

   Checks if one object is supporting another object. An object supports another object if they are in
   contact and the second object is above the first one. (e.g. a Bottle will be supported by a table)

   :param object1: Object that is supported
   :param object2: Object that supports the first object
   :return: True if the second object is in contact with the first one and the second is above the first else False


.. py:function:: link_pose_for_joint_config(obj: pycram.world_concepts.world_object.Object, joint_config: typing_extensions.Dict[str, float], link_name: str) -> pycram.datastructures.pose.Pose

   Returns the pose a link would be in if the given joint configuration would be applied to the object.
   This is done by using the respective object in the prospection world and applying the joint configuration
   to this one. After applying the joint configuration the link position is taken from there.

   :param obj: Object of which the link is a part
   :param joint_config: Dict with the goal joint configuration
   :param link_name: Name of the link for which the pose should be returned
   :return: The pose of the link after applying the joint configuration


