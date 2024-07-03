:py:mod:`pycram.designators.specialized_designators.location.giskard_location`
==============================================================================

.. py:module:: pycram.designators.specialized_designators.location.giskard_location


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.designators.specialized_designators.location.giskard_location.GiskardLocation




.. py:class:: GiskardLocation(target: typing_extensions.Union[pycram.datastructures.pose.Pose, pycram.designators.object_designator.ObjectDesignatorDescription.Object], reachable_for: typing_extensions.Optional[pycram.designators.object_designator.ObjectDesignatorDescription.Object] = None, visible_for: typing_extensions.Optional[pycram.designators.object_designator.ObjectDesignatorDescription.Object] = None, reachable_arm: typing_extensions.Optional[pycram.datastructures.enums.Arms] = None, resolver: typing_extensions.Optional[typing_extensions.Callable] = None)


   Bases: :py:obj:`pycram.designators.location_designator.CostmapLocation`

   '
   Specialization version of the CostmapLocation which uses Giskard to solve for a full-body IK solution. This
   designator is especially useful for robots which lack a degree of freedom and therefore need to use the base to
   manipulate the environment effectively.

   Location designator that uses costmaps as base to calculate locations for complex constrains like reachable or
   visible. In case of reachable the resolved location contains a list of arms with which the location is reachable.

   :param target: Location for which visibility or reachability should be calculated
   :param reachable_for: Object for which the reachability should be calculated, usually a robot
   :param visible_for: Object for which the visibility should be calculated, usually a robot
   :param reachable_arm: An optional arm with which the target should be reached
   :param resolver: An alternative specialized_designators that returns a resolved location for the given input of this description

   .. py:method:: __iter__() -> pycram.designators.location_designator.CostmapLocation.Location

      Uses Giskard to perform full body ik solving to get the pose of a robot at which it is able to reach a certain point.

      :yield: An instance of CostmapLocation.Location with a pose from which the robot can reach the target



