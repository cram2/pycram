:py:mod:`pycram.designators.specialized_designators.location.database_location`
===============================================================================

.. py:module:: pycram.designators.specialized_designators.location.database_location


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.designators.specialized_designators.location.database_location.Location
   pycram.designators.specialized_designators.location.database_location.AbstractCostmapLocation
   pycram.designators.specialized_designators.location.database_location.DatabaseCostmapLocation




.. py:class:: Location


   Bases: :py:obj:`pycram.designator.LocationDesignatorDescription.Location`

   A location that is described by a pose, a reachable arm, a torso height and a grasp.

   .. py:attribute:: pose
      :type: pycram.datastructures.pose.Pose

      

   .. py:attribute:: reachable_arm
      :type: str

      

   .. py:attribute:: torso_height
      :type: float

      

   .. py:attribute:: grasp
      :type: str

      


.. py:class:: AbstractCostmapLocation(target, reachable_for=None, reachable_arm=None)


   Bases: :py:obj:`pycram.designators.location_designator.CostmapLocation`

   Abstract Class for JPT and Database costmaps.

   Create a new AbstractCostmapLocation instance.
   :param target: The target object
   :param reachable_for:
   :param reachable_arm:

   .. py:method:: create_occupancy_rectangles() -> typing_extensions.List[pycram.costmaps.Rectangle]

      :return: A list of rectangles that represent the occupied space of the target object.



.. py:class:: DatabaseCostmapLocation(target, session: sqlalchemy.orm.Session = None, reachable_for=None, reachable_arm=None)


   Bases: :py:obj:`AbstractCostmapLocation`

   Class that represents costmap locations from a given Database.
   The database has to have a schema that is compatible with the pycram.orm package.

   Create a Database Costmap

   :param target: The target object
   :param session: A session that can be used to execute queries
   :param reachable_for: The robot to grab the object with
   :param reachable_arm: The arm to use


   .. py:method:: select_statement(view: typing_extensions.Type[pycram.orm.views.PickUpWithContextView]) -> sqlalchemy.Select
      :staticmethod:


   .. py:method:: create_query_from_occupancy_costmap() -> sqlalchemy.Select

      Create a query that queries all relative robot positions from an object that are not occluded using an
      OccupancyCostmap.


   .. py:method:: sample_to_location(sample: sqlalchemy.engine.row.Row) -> Location

      Convert a database row to a costmap location.

      :param sample: The database row.
      :return: The costmap location


   .. py:method:: __iter__() -> Location

      Generates positions for a given set of constrains from a costmap and returns
      them. The generation is based of a costmap which itself is the product of
      merging costmaps, each for a different purpose. In any case an occupancy costmap
      is used as the base, then according to the given constrains a visibility or
      gaussian costmap is also merged with this. Once the costmaps are merged,
      a generator generates pose candidates from the costmap. Each pose candidate
      is then validated against the constraints given by the designator if all validators
      pass the pose is considered valid and yielded.

      :yield: An instance of CostmapLocation.Location with a valid position that satisfies the given constraints



