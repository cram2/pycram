:py:mod:`pycram.pose_generator_and_validator`
=============================================

.. py:module:: pycram.pose_generator_and_validator


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.pose_generator_and_validator.PoseGenerator



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.pose_generator_and_validator.visibility_validator
   pycram.pose_generator_and_validator._in_contact
   pycram.pose_generator_and_validator.reachability_validator
   pycram.pose_generator_and_validator.collision_check



.. py:class:: PoseGenerator(costmap: pycram.costmaps.Costmap, number_of_samples=100, orientation_generator=None)


   Crates pose candidates from a given costmap. The generator
   selects the highest values, amount is given by number_of_sample, and returns the corresponding positions.
   Orientations are calculated such that the Robot faces the center of the costmap.

   :param costmap: The costmap from which poses should be sampled.
   :param number_of_samples: The number of samples from the costmap that should be returned at max
   :param orientation_generator: function that generates an orientation given a position and the origin of the costmap

   .. py:attribute:: current_orientation_generator

      If no orientation generator is given, this generator is used to generate the orientation of the robot.

   .. py:attribute:: override_orientation_generator

      Override the orientation generator with a custom generator, which will be used regardless of the current_orientation_generator.

   .. py:method:: __iter__() -> typing_extensions.Iterable

      A generator that crates pose candidates from a given costmap. The generator
      selects the highest 100 values and returns the corresponding positions.
      Orientations are calculated such that the Robot faces the center of the costmap.

      :Yield: A tuple of position and orientation


   .. py:method:: height_generator() -> float
      :staticmethod:


   .. py:method:: generate_orientation(position: typing_extensions.List[float], origin: pycram.datastructures.pose.Pose) -> typing_extensions.List[float]
      :staticmethod:

      This method generates the orientation for a given position in a costmap. The
      orientation is calculated such that the robot faces the origin of the costmap.
      This generation is done by simply calculating the arctan between the position,
      in the costmap, and the origin of the costmap.

      :param position: The position in the costmap. This position is already converted to the world coordinate frame.
      :param origin: The origin of the costmap. This is also the point which the robot should face.
      :return: A quaternion of the calculated orientation



.. py:function:: visibility_validator(pose: pycram.datastructures.pose.Pose, robot: pycram.world_concepts.world_object.Object, object_or_pose: typing_extensions.Union[pycram.world_concepts.world_object.Object, pycram.datastructures.pose.Pose], world: pycram.datastructures.world.World) -> bool

   This method validates if the robot can see the target position from a given
   pose candidate. The target position can either be a position, in world coordinate
   system, or an object in the World. The validation is done by shooting a
   ray from the camera to the target position and checking that it does not collide
   with anything else.

   :param pose: The pose candidate that should be validated
   :param robot: The robot object for which this should be validated
   :param object_or_pose: The target position or object for which the pose candidate should be validated.
   :param world: The World instance in which this should be validated.
   :return: True if the target is visible for the robot, None in any other case.


.. py:function:: _in_contact(robot: pycram.world_concepts.world_object.Object, obj: pycram.world_concepts.world_object.Object, allowed_collision: typing_extensions.Dict[pycram.world_concepts.world_object.Object, typing_extensions.List[str]], allowed_robot_links: typing_extensions.List[str]) -> bool

   This method checks if a given robot is in contact with a given object.

   :param robot: The robot object that should be checked for contact.
   :param obj: The object that should be checked for contact with the robot.
   :param allowed_collision: A dictionary that contains the allowed collisions for links of each object in the world.
   :param allowed_robot_links: A list of links of the robot that are allowed to be in contact with the object.
   :return: True if the robot is in contact with the object and False otherwise.


.. py:function:: reachability_validator(pose: pycram.datastructures.pose.Pose, robot: pycram.world_concepts.world_object.Object, target: typing_extensions.Union[pycram.world_concepts.world_object.Object, pycram.datastructures.pose.Pose], allowed_collision: typing_extensions.Dict[pycram.world_concepts.world_object.Object, typing_extensions.List] = None) -> typing_extensions.Tuple[bool, typing_extensions.List]

   This method validates if a target position is reachable for a pose candidate.
   This is done by asking the ik solver if there is a valid solution if the
   robot stands at the position of the pose candidate. if there is a solution
   the validator returns True and False in any other case.

   :param pose: The pose candidate for which the reachability should be validated
   :param robot: The robot object in the World for which the reachability should be validated.
   :param target: The target position or object instance which should be the target for reachability.
   :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates to a list of links of which this object consists
   :return: True if the target is reachable for the robot and False in any other case.


.. py:function:: collision_check(robot: pycram.world_concepts.world_object.Object, allowed_collision: typing_extensions.Dict[pycram.world_concepts.world_object.Object, typing_extensions.List])

   This method checks if a given robot collides with any object within the world
   which it is not allowed to collide with.
   This is done checking iterating over every object within the world and checking
   if the robot collides with it. Careful the floor will be ignored.
   If there is a collision with an object that was not within the allowed collision
   list the function returns True else it will return False

   :param robot: The robot object in the (Bullet)World where it should be checked if it collides with something
   :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates to a list of links of which this object consists
   :return: True if the target is reachable for the robot and False in any other case.


