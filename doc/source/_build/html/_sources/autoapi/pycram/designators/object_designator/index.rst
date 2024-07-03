:py:mod:`pycram.designators.object_designator`
==============================================

.. py:module:: pycram.designators.object_designator


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.designators.object_designator.BelieveObject
   pycram.designators.object_designator.ObjectPart
   pycram.designators.object_designator.LocatedObject
   pycram.designators.object_designator.RealObject




.. py:class:: BelieveObject(names: typing_extensions.Optional[typing_extensions.List[str]] = None, types: typing_extensions.Optional[typing_extensions.List[pycram.datastructures.enums.ObjectType]] = None, resolver: typing_extensions.Optional[typing_extensions.Callable] = None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ObjectDesignatorDescription`

   Description for Objects that are only believed in.

   Base of all object designator descriptions. Every object designator has the name and type of the object.

   :param names: A list of names that could describe the object
   :param types: A list of types that could represent the object
   :param resolver: An alternative specialized_designators that returns an object designator for the list of names and types
   :param ontology_concept_holders: A list of ontology concepts that the object is categorized as or associated with

   .. py:class:: Object


      Bases: :py:obj:`pycram.designator.ObjectDesignatorDescription.Object`

      Concrete object that is believed in.

      .. py:method:: to_sql() -> pycram.orm.object_designator.BelieveObject

         Create an ORM object that corresponds to this description.

         :return: The created ORM object.


      .. py:method:: insert(session: sqlalchemy.orm.session.Session) -> pycram.orm.object_designator.BelieveObject

         Add and commit this and all related objects to the session.
         Auto-Incrementing primary keys and foreign keys have to be filled by this method.

         :param session: Session with a database that is used to add and commit the objects
         :return: The completely instanced ORM object




.. py:class:: ObjectPart(names: typing_extensions.List[str], part_of: pycram.designator.ObjectDesignatorDescription.Object, type: typing_extensions.Optional[str] = None, resolver: typing_extensions.Optional[typing_extensions.Callable] = None)


   Bases: :py:obj:`pycram.designator.ObjectDesignatorDescription`

   Object Designator Descriptions for Objects that are part of some other object.

   Describing the relationship between an object and a specific part of it.

   :param names: Possible names for the part
   :param part_of: Parent object of which the part should be described
   :param type: Type of the part
   :param resolver: An alternative specialized_designators to resolve the input parameter to an object designator
   :param ontology_concept_holders: A list of ontology concepts that the object part is categorized as or associated with

   .. py:class:: Object


      Bases: :py:obj:`pycram.designator.ObjectDesignatorDescription.Object`

      A single element that fits the description.

      .. py:attribute:: part_pose
         :type: pycram.datastructures.pose.Pose

         

      .. py:method:: to_sql() -> pycram.orm.object_designator.ObjectPart

         Create an ORM object that corresponds to this description.

         :return: The created ORM object.


      .. py:method:: insert(session: sqlalchemy.orm.session.Session) -> pycram.orm.object_designator.ObjectPart

         Add and commit this and all related objects to the session.
         Auto-Incrementing primary keys and foreign keys have to be filled by this method.

         :param session: Session with a database that is used to add and commit the objects
         :return: The completely instanced ORM object



   .. py:method:: ground() -> Object

      Default specialized_designators, returns the first result of the iterator of this instance.

      :return: A resolved object designator


   .. py:method:: __iter__()

      Iterates through every possible solution for the given input parameter.

      :yield: A resolved Object designator



.. py:class:: LocatedObject(names: typing_extensions.List[str], types: typing_extensions.List[str], reference_frames: typing_extensions.List[str], timestamps: typing_extensions.List[float], resolver: typing_extensions.Optional[typing_extensions.Callable] = None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`pycram.designator.ObjectDesignatorDescription`

   Description for KnowRob located objects.
   **Currently has no specialized_designators**

   Describing an object resolved through knowrob.

   :param names: List of possible names describing the object
   :param types: List of possible types describing the object
   :param reference_frames: Frame of reference in which the object position should be
   :param timestamps: Timestamps for which positions should be returned
   :param resolver: An alternative specialized_designators that resolves the input parameter to an object designator.
   :param ontology_concept_holders: A list of ontology concepts that the object is categorized as

   .. py:class:: Object


      Bases: :py:obj:`pycram.designator.ObjectDesignatorDescription.Object`

      A single element that fits the description.

      .. py:attribute:: reference_frame
         :type: str

         Reference frame in which the position is given

      .. py:attribute:: timestamp
         :type: float

         Timestamp at which the position was valid



.. py:class:: RealObject(names: typing_extensions.Optional[typing_extensions.List[str]] = None, types: typing_extensions.Optional[typing_extensions.List[str]] = None, world_object: pycram.world_concepts.world_object.Object = None, resolver: typing_extensions.Optional[typing_extensions.Callable] = None)


   Bases: :py:obj:`pycram.designator.ObjectDesignatorDescription`

   Object designator representing an object in the real world, when resolving this object designator description ]
   RoboKudo is queried to perceive an object fitting the given criteria. Afterward the specialized_designators tries to match
   the found object to an Object in the World.

   :param names:
   :param types:
   :param world_object:
   :param resolver:

   .. py:class:: Object


      Bases: :py:obj:`pycram.designator.ObjectDesignatorDescription.Object`

      A single element that fits the description.

      .. py:attribute:: pose
         :type: pycram.datastructures.pose.Pose

         Pose of the perceived object


   .. py:method:: __iter__()

      Queries RoboKudo for objects that fit the description and then iterates over all World objects that have
      the same type to match a World object to the real object.

      :yield: A resolved object designator with reference world object



