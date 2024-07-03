:py:mod:`pycram.designator`
===========================

.. py:module:: pycram.designator


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.designator.Designator
   pycram.designator.DesignatorDescription
   pycram.designator.ActionDesignatorDescription
   pycram.designator.LocationDesignatorDescription
   pycram.designator.ObjectDesignatorDescription




Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.designator.owlready2
   pycram.designator.SPECIAL_KNOWLEDGE


.. py:data:: owlready2

   

.. py:exception:: DesignatorError(*args, **kwargs)


   Bases: :py:obj:`Exception`

   Implementation of designator errors.

   Create a new designator error.


.. py:exception:: ResolutionError(missing_properties: typing_extensions.List[str], wrong_type: typing_extensions.Dict, current_type: typing_extensions.Any, designator: Designator)


   Bases: :py:obj:`Exception`

   Common base class for all non-exit exceptions.

   Initialize self.  See help(type(self)) for accurate signature.


.. py:class:: Designator(description: DesignatorDescription, parent: typing_extensions.Optional[Designator] = None)


   Bases: :py:obj:`abc.ABC`

   Implementation of designators. DEPRECTAED SINCE DESIGNATOR DESCRIPTIONS ARE USED AS BASE CLASS

   Designators are objects containing sequences of key-value pairs. They can be resolved which means to generate real
   parameters for executing performables from these pairs of key and value.

   :ivar timestamp: The timestamp of creation of reference or None if still not referencing an object.

   Create a new desginator.

   Arguments:
   :param description: A list of tuples (key-value pairs) describing this designator.
   :param parent: The parent to equate with (default is None).

   .. py:attribute:: resolvers

      List of all designator resolvers. Designator resolvers are functions which take a designator as
      argument and return a list of solutions. A solution can also be a generator.

   .. py:method:: equate(parent: Designator) -> None

      Equate the designator with the given parent.

      Arguments:
      parent -- the parent to equate with.


   .. py:method:: equal(other: Designator) -> bool

      Check if the designator describes the same entity as another designator, i.e. if they are equated.

      Arguments:
      other -- the other designator.


   .. py:method:: first() -> Designator

      Return the first ancestor in the chain of equated designators.


   .. py:method:: current() -> Designator

      Return the newest designator, i.e. that one that has been equated last to the designator or one of its
      equated designators.


   .. py:method:: _reference() -> typing_extensions.Any

      This is a helper method for internal usage only.

      This method is to be overwritten instead of the reference method.


   .. py:method:: reference() -> typing_extensions.Any

      Try to dereference the designator and return its data object or raise DesignatorError if it is not an
      effective designator.


   .. py:method:: next_solution()
      :abstractmethod:

      Return another solution for the effective designator or None if none exists. The next solution is a newly
      constructed designator with identical properties that is equated to the designator since it describes the same
      entity.


   .. py:method:: solutions(from_root: typing_extensions.Optional[Designator] = None)

      Return a generator for all solutions of the designator.

      Arguments:
      from_root -- if not None, the generator for all solutions beginning from with the original designator is returned (default is None).


   .. py:method:: copy(new_properties: typing_extensions.Optional[typing_extensions.List] = None) -> Designator

      Construct a new designator with the same properties as this one. If new properties are specified, these will
      be merged with the old ones while the new properties are dominant in this relation.

      Arguments:
      new_properties -- a list of new properties to merge into the old ones (default is None).


   .. py:method:: make_effective(properties: typing_extensions.Optional[typing_extensions.List] = None, data: typing_extensions.Optional[typing_extensions.Any] = None, timestamp: typing_extensions.Optional[float] = None) -> Designator

      Create a new effective designator of the same type as this one. If no properties are specified, this ones are used.

      Arguments:
      new_properties -- a list of properties (default is None).
      data -- the low-level data structure the new designator describes (default is None).
      timestamp -- the timestamp of creation of reference (default is the current).


   .. py:method:: newest_effective() -> Designator

      Return the newest effective designator.


   .. py:method:: prop_value(key: str) -> typing_extensions.Any

      Return the first value matching the specified property key.

      Arguments:
      key -- the key to return the value of.


   .. py:method:: check_constraints(properties: typing_extensions.List) -> bool

      Return True if all the given properties match, False otherwise.

      Arguments:
      properties -- the properties which have to match. A property can be a tuple in which case its first value is the
      key of a property which must equal the second value. Otherwise it's simply the key of a property which must be
      not None.


   .. py:method:: make_dictionary(properties: typing_extensions.List) -> typing_extensions.Dict

      DEPRECATED, Moved to the description. Function only keept because of
      backward compatability.
      Return the given properties as dictionary.

      Arguments:
      properties -- the properties to create a dictionary of. A property can be a tuple in which case its first value
      is the dictionary key and the second value is the dictionary value. Otherwise it's simply the dictionary key
      and the key of a property which is the dictionary value.


   .. py:method:: rename_prop(old: str, new: str) -> Designator



.. py:class:: DesignatorDescription(resolver: typing_extensions.Optional[typing_extensions.Callable] = None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[pycram.ontology.ontology_common.OntologyConceptHolder]] = None)


   Bases: :py:obj:`abc.ABC`

   :ivar resolve: The specialized_designators function to use for this designator, defaults to self.ground

   Create a Designator description.

   :param resolver: The grounding method used for the description. The grounding method creates a location instance that matches the description.
   :param ontology_concept_holders: A list of holders of ontology concepts that the designator is categorized as or associated with

   .. py:method:: make_dictionary(properties: typing_extensions.List[str])

      Creates a dictionary of this description with only the given properties
      included.

      :param properties: A list of properties that should be included in the dictionary.
                          The given properties have to be an attribute of this description.
      :return: A dictionary with the properties as keys.


   .. py:method:: ground() -> typing_extensions.Any

      Should be overwritten with an actual grounding function which infers missing properties.


   .. py:method:: get_slots() -> typing_extensions.List[str]

      Returns a list of all slots of this description. Can be used for inspecting different descriptions and debugging.

      :return: A list of all slots.


   .. py:method:: copy() -> DesignatorDescription


   .. py:method:: get_default_ontology_concept() -> owlready2.Thing | None

      Returns the first element of ontology_concept_holders if there is, else None



.. py:class:: ActionDesignatorDescription(resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[pycram.ontology.ontology_common.OntologyConceptHolder]] = None)


   Bases: :py:obj:`DesignatorDescription`, :py:obj:`pycram.language.Language`

   Abstract class for action designator descriptions.
   Descriptions hold possible parameter ranges for action designators.

   Base of all action designator descriptions.

   :param resolver: An alternative resolver that returns an action designator
   :param ontology_concept_holders: A list of ontology concepts that the action is categorized as or associated with

   .. py:class:: Action


      The performable designator with a single element for each list of possible parameter.

      .. py:attribute:: robot_position
         :type: pycram.datastructures.pose.Pose

         The position of the robot at the start of the action.

      .. py:attribute:: robot_torso_height
         :type: float

         The torso height of the robot at the start of the action.

      .. py:attribute:: robot_type
         :type: pycram.datastructures.enums.ObjectType

         The type of the robot at the start of the action.

      .. py:method:: __post_init__()


      .. py:method:: perform() -> typing_extensions.Any
         :abstractmethod:

         Executes the action with the single parameters from the description.


      .. py:method:: to_sql() -> pycram.orm.action_designator.Action
         :abstractmethod:

         Create an ORM object that corresponds to this description.

         :return: The created ORM object.


      .. py:method:: insert(session: sqlalchemy.orm.session.Session, *args, **kwargs) -> pycram.orm.action_designator.Action

         Add and commit this and all related objects to the session.
         Auto-Incrementing primary keys and foreign keys have to be filled by this method.

         :param session: Session with a database that is used to add and commit the objects
         :param args: Possible extra arguments
         :param kwargs: Possible extra keyword arguments
         :return: The completely instanced ORM object



   .. py:method:: ground() -> Action
      :abstractmethod:

      Fill all missing parameters and chose plan to execute.


   .. py:method:: init_ontology_concepts(ontology_concept_classes: typing_extensions.Dict[str, typing_extensions.Type[owlready2.Thing]])

      Initialize the ontology concept holders for this action designator

      :param ontology_concept_classes: The ontology concept classes that the action is categorized as or associated with
      :param ontology_concept_name: The name of the ontology concept instance to be created


   .. py:method:: __iter__()

      Iterate through all possible performables fitting this description

      :yield: A resolved action designator



.. py:class:: LocationDesignatorDescription(resolver=None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`DesignatorDescription`

   Parent class of location designator descriptions.

   Create a Designator description.

   :param resolver: The grounding method used for the description. The grounding method creates a location instance that matches the description.
   :param ontology_concept_holders: A list of holders of ontology concepts that the designator is categorized as or associated with

   .. py:class:: Location


      Resolved location that represents a specific point in the world which satisfies the constraints of the location
      designator description.

      .. py:attribute:: pose
         :type: pycram.datastructures.pose.Pose

         The resolved pose of the location designator. Pose is inherited by all location designator.


   .. py:method:: ground() -> Location
      :abstractmethod:

      Find a location that satisfies all constrains.



.. py:data:: SPECIAL_KNOWLEDGE

   

.. py:class:: ObjectDesignatorDescription(names: typing_extensions.Optional[typing_extensions.List[str]] = None, types: typing_extensions.Optional[typing_extensions.List[pycram.datastructures.enums.ObjectType]] = None, resolver: typing_extensions.Optional[typing_extensions.Callable] = None, ontology_concept_holders: typing_extensions.Optional[typing_extensions.List[owlready2.Thing]] = None)


   Bases: :py:obj:`DesignatorDescription`

   Class for object designator descriptions.
   Descriptions hold possible parameter ranges for object designators.

   Base of all object designator descriptions. Every object designator has the name and type of the object.

   :param names: A list of names that could describe the object
   :param types: A list of types that could represent the object
   :param resolver: An alternative specialized_designators that returns an object designator for the list of names and types
   :param ontology_concept_holders: A list of ontology concepts that the object is categorized as or associated with

   .. py:class:: Object


      A single element that fits the description.

      .. py:property:: pose

         Property of the current position and orientation of the object.
         Evaluate the _pose function.

         :return: Position and orientation

      .. py:attribute:: name
         :type: str

         Name of the object

      .. py:attribute:: obj_type
         :type: pycram.datastructures.enums.ObjectType

         Type of the object

      .. py:attribute:: world_object
         :type: typing_extensions.Optional[pycram.world_concepts.world_object.Object]

         Reference to the World object

      .. py:attribute:: _pose
         :type: typing_extensions.Optional[typing_extensions.Callable]

         A callable returning the pose of this object. The _pose member is used overwritten for data copies
         which will not update when the original world_object is moved.

      .. py:method:: __post_init__()


      .. py:method:: to_sql() -> pycram.orm.object_designator.Object

         Create an ORM object that corresponds to this description.

         :return: The created ORM object.


      .. py:method:: insert(session: sqlalchemy.orm.session.Session) -> pycram.orm.object_designator.Object

         Add and commit this and all related objects to the session.
         Auto-Incrementing primary keys and foreign keys have to be filled by this method.

         :param session: Session with a database that is used to add and commit the objects
         :return: The completely instanced ORM object


      .. py:method:: frozen_copy() -> ObjectDesignatorDescription

         Returns a copy of this designator containing only the fields.

         :return: A copy containing only the fields of this class. The WorldObject attached to this pycram object is not copied. The _pose gets set to a method that statically returns the pose of the object when this method was called.


      .. py:method:: __repr__()

         Return repr(self).


      .. py:method:: special_knowledge_adjustment_pose(grasp: str, pose: pycram.datastructures.pose.Pose) -> pycram.datastructures.pose.Pose

         Returns the adjusted target pose based on special knowledge for "grasp front".

         :param grasp: From which side the object should be grasped
         :param pose: Pose at which the object should be grasped, before adjustment
         :return: The adjusted grasp pose



   .. py:method:: ground() -> typing_extensions.Union[Object, bool]

      Return the first object from the world that fits the description.

      :return: A resolved object designator


   .. py:method:: __iter__() -> typing_extensions.Iterable[Object]

      Iterate through all possible objects fitting this description

      :yield: A resolved object designator



