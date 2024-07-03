:py:mod:`pycram.designators.location_designator`
================================================

.. py:module:: pycram.designators.location_designator


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.designators.location_designator.Location
   pycram.designators.location_designator.ObjectRelativeLocation
   pycram.designators.location_designator.CostmapLocation
   pycram.designators.location_designator.AccessingLocation
   pycram.designators.location_designator.SemanticCostmapLocation




.. py:class:: Location(pose: pycram.datastructures.pose.Pose, resolver=None)


   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`

   Default location designator which only wraps a pose.

   Basic location designator that represents a single pose.

   :param pose: The pose that should be represented by this location designator
   :param resolver: An alternative specialized_designators that returns a resolved location

   .. py:class:: Location


      Bases: :py:obj:`pycram.designator.LocationDesignatorDescription.Location`

      Resolved location that represents a specific point in the world which satisfies the constraints of the location
      designator description.


   .. py:method:: ground() -> Location

      Default specialized_designators which returns a resolved designator which contains the pose given in init.

      :return: A resolved designator



.. py:class:: ObjectRelativeLocation(relative_pose: pycram.datastructures.pose.Pose = None, reference_object: pycram.designators.object_designator.ObjectDesignatorDescription = None, resolver=None)


   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`

   Location relative to an object

   Location designator representing a location relative to a given object.

   :param relative_pose: Pose that should be relative, in world coordinate frame
   :param reference_object: Object to which the pose should be relative
   :param resolver: An alternative specialized_designators that returns a resolved location for the input parameter

   .. py:class:: Location


      Bases: :py:obj:`pycram.designator.LocationDesignatorDescription.Location`

      Resolved location that represents a specific point in the world which satisfies the constraints of the location
      designator description.

      .. py:attribute:: relative_pose
         :type: pycram.datastructures.pose.Pose

         Pose relative to the object

      .. py:attribute:: reference_object
         :type: pycram.designators.object_designator.ObjectDesignatorDescription.Object

         Object to which the pose is relative


   .. py:method:: ground() -> Location

      Default specialized_designators which returns a resolved location for description input. Resolved location is the first result
      of the iteration of this instance.

      :return: A resolved location


   .. py:method:: __iter__() -> typing_extensions.Iterable[Location]

      Iterates over all possible solutions for a resolved location that is relative to the given object.

      :yield: An instance of ObjectRelativeLocation.Location with the relative pose



.. py:class:: CostmapLocation(target: typing_extensions.Union[pycram.datastructures.pose.Pose, pycram.designators.object_designator.ObjectDesignatorDescription.Object], reachable_for: typing_extensions.Optional[pycram.designators.object_designator.ObjectDesignatorDescription.Object] = None, visible_for: typing_extensions.Optional[pycram.designators.object_designator.ObjectDesignatorDescription.Object] = None, reachable_arm: typing_extensions.Optional[pycram.datastructures.enums.Arms] = None, resolver: typing_extensions.Optional[typing_extensions.Callable] = None)


   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`

   Uses Costmaps to create locations for complex constrains

   Location designator that uses costmaps as base to calculate locations for complex constrains like reachable or
   visible. In case of reachable the resolved location contains a list of arms with which the location is reachable.

   :param target: Location for which visibility or reachability should be calculated
   :param reachable_for: Object for which the reachability should be calculated, usually a robot
   :param visible_for: Object for which the visibility should be calculated, usually a robot
   :param reachable_arm: An optional arm with which the target should be reached
   :param resolver: An alternative specialized_designators that returns a resolved location for the given input of this description

   .. py:class:: Location


      Bases: :py:obj:`pycram.designator.LocationDesignatorDescription.Location`

      Resolved location that represents a specific point in the world which satisfies the constraints of the location
      designator description.

      .. py:attribute:: reachable_arms
         :type: typing_extensions.List[pycram.datastructures.enums.Arms]

         List of arms with which the pose can be reached, is only used when the 'rechable_for' parameter is used


   .. py:method:: ground() -> Location

      Default specialized_designators which returns the first result from the iterator of this instance.

      :return: A resolved location


   .. py:method:: __iter__()

      Generates positions for a given set of constrains from a costmap and returns
      them. The generation is based of a costmap which itself is the product of
      merging costmaps, each for a different purpose. In any case an occupancy costmap
      is used as the base, then according to the given constrains a visibility or
      gaussian costmap is also merged with this. Once the costmaps are merged,
      a generator generates pose candidates from the costmap. Each pose candidate
      is then validated against the constraints given by the designator if all validators
      pass the pose is considered valid and yielded.

      :yield: An instance of CostmapLocation.Location with a valid position that satisfies the given constraints



.. py:class:: AccessingLocation(handle_desig: pycram.designators.object_designator.ObjectPart.Object, robot_desig: pycram.designators.object_designator.ObjectDesignatorDescription.Object, resolver=None)


   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`

   Location designator which describes poses used for opening drawers

   Describes a position from where a drawer can be opened. For now this position should be calculated before the
   drawer will be opened. Calculating the pose while the drawer is open could lead to problems.

   :param handle_desig: ObjectPart designator for handle of the drawer
   :param robot: Object designator for the robot which should open the drawer
   :param resolver: An alternative specialized_designators to create the location

   .. py:class:: Location


      Bases: :py:obj:`pycram.designator.LocationDesignatorDescription.Location`

      Resolved location that represents a specific point in the world which satisfies the constraints of the location
      designator description.

      .. py:attribute:: arms
         :type: typing_extensions.List[pycram.datastructures.enums.Arms]

         List of arms that can be used to for accessing from this pose


   .. py:method:: ground() -> Location

      Default specialized_designators for this location designator, just returns the first element from the iteration

      :return: A location designator for a pose from which the drawer can be opened


   .. py:method:: __iter__() -> Location

      Creates poses from which the robot can open the drawer specified by the ObjectPart designator describing the
      handle. Poses are validated by checking if the robot can grasp the handle while the drawer is closed and if
      the handle can be grasped if the drawer is open.

      :yield: A location designator containing the pose and the arms that can be used.



.. py:class:: SemanticCostmapLocation(urdf_link_name, part_of, for_object=None, resolver=None)


   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription`

   Locations over semantic entities, like a table surface

   Creates a distribution over a urdf link to sample poses which are on this link. Can be used, for example, to find
   poses that are on a table. Optionally an object can be given for which poses should be calculated, in that case
   the poses are calculated such that the bottom of the object is on the link.

   :param urdf_link_name: Name of the urdf link for which a distribution should be calculated
   :param part_of: Object of which the urdf link is a part
   :param for_object: Optional object that should be placed at the found location
   :param resolver: An alternative specialized_designators that creates a resolved location for the input parameter of this description

   .. py:class:: Location


      Bases: :py:obj:`pycram.designator.LocationDesignatorDescription.Location`

      Resolved location that represents a specific point in the world which satisfies the constraints of the location
      designator description.


   .. py:method:: ground() -> Location

      Default specialized_designators which returns the first element of the iterator of this instance.

      :return: A resolved location


   .. py:method:: __iter__()

      Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
      which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
      is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

      :yield: An instance of SemanticCostmapLocation.Location with the found valid position of the Costmap.



