:py:mod:`pycram.ontology.ontology`
==================================

.. py:module:: pycram.ontology.ontology


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.ontology.ontology.OntologyManager




Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.ontology.ontology.SOMA_HOME_ONTOLOGY_IRI
   pycram.ontology.ontology.SOMA_ONTOLOGY_IRI
   pycram.ontology.ontology.SOMA_ONTOLOGY_NAMESPACE
   pycram.ontology.ontology.DUL_ONTOLOGY_NAMESPACE


.. py:data:: SOMA_HOME_ONTOLOGY_IRI
   :value: 'http://www.ease-crc.org/ont/SOMA-HOME.owl'

   

.. py:data:: SOMA_ONTOLOGY_IRI
   :value: 'http://www.ease-crc.org/ont/SOMA.owl'

   

.. py:data:: SOMA_ONTOLOGY_NAMESPACE
   :value: 'SOMA'

   

.. py:data:: DUL_ONTOLOGY_NAMESPACE
   :value: 'DUL'

   

.. py:class:: OntologyManager(main_ontology_iri: Optional[str] = None, ontology_search_path: Optional[str] = None)


   Bases: :py:obj:`object`

   Singleton class as the adapter accessing data of an OWL ontology, largely based on owlready2.

   Create the singleton object of OntologyManager class

   :param main_ontology_iri: Ontology IRI (Internationalized Resource Identifier), either a URL to a remote OWL file or the full name path of a local one
   :param ontology_search_path: directory path from which a possibly existing ontology is searched. This is appended to `owlready2.onto_path`, a global variable containing a list of directories for searching local copies of ontologies (similarly to python `sys.path` for modules/packages). If not specified, the path is "$HOME/ontologies"

   .. py:method:: print_ontology_class(ontology_class: Type[owlready2.Thing])
      :staticmethod:

      Print information (ancestors, super classes, subclasses, properties, etc.) of an ontology class

      :param ontology_class: An ontology class


   .. py:method:: load_ontology(ontology_iri: str) -> tuple[owlready2.Ontology, owlready2.Namespace]

      Load an ontology from an IRI

      :param ontology_iri: An ontology IRI
      :return: A tuple including an ontology instance & its namespace


   .. py:method:: initialized() -> bool

      Check if the main ontology has been loaded

      :return: True if loaded, otherwise False


   .. py:method:: browse_ontologies(ontology: owlready2.Ontology, condition: Optional[Callable] = None, func: Optional[Callable] = None, **kwargs) -> None
      :staticmethod:

      Browse the loaded ontologies (including the main and imported ones), doing operations based on a condition.

      :param ontology: An ontology instance as the result of ontology loading
      :param condition: a Callable condition that if not None needs to be passed before doing operations, otherwise just always carry the operations
      :param func: a Callable specifying the operations to perform on all the loaded ontologies if condition is None, otherwise only the first ontology which meets the condition


   .. py:method:: save(target_filename: str = '', overwrite: bool = False) -> bool

      Save the current ontology to disk

      :param target_filename: full name path of a file which the ontologies are saved into.
      :param overwrite: overwrite an existing file if it exists. If empty, they are saved to the same original OWL file from which the main ontology was loaded, or a file at the same folder with ontology search path specified at constructor if it was loaded from a remote IRI.
      :return: True if the ontology was successfully saved, False otherwise


   .. py:method:: create_ontology_concept_class(class_name: str, ontology_parent_concept_class: Optional[owlready2.Thing] = None) -> Type[owlready2.Thing]

      Create a new concept class in ontology

      :param class_name: A given name to the new class
      :param ontology_parent_concept_class: An optional parent ontology class of the new class
      :return: The created ontology class


   .. py:method:: create_ontology_property_class(class_name: str, ontology_parent_property_class: Optional[Type[owlready2.Property]] = None) -> Optional[Type[owlready2.Property]]
      :staticmethod:

      Create a new property class in ontology

      :param class_name: A given name to the new class
      :param ontology_parent_property_class: An optional parent ontology property class of the new class
      :return: The created ontology class


   .. py:method:: get_ontology_classes_by_condition(condition: Callable, first_match_only=False, **kwargs) -> List[Type[owlready2.Thing]]

      Get an ontology class by a given condition

      :param condition: condition of searching
      :param first_match_only: whether to only fetch the first class matching the given condition
      :return: The ontology class satisfying the given condition if found else None


   .. py:method:: get_ontology_class_by_ontology(ontology: owlready2.Ontology, class_name: str) -> Optional[Type[owlready2.Thing]]
      :staticmethod:

      Get an ontology class if it exists in a given ontology

      :param ontology: an ontology instance
      :return: The ontology class if it exists under the namespace of the given ontology, None otherwise


   .. py:method:: get_ontology_class(class_name: str) -> Optional[Type[owlready2.Thing]]

      Get an ontology class by name

      :param class_name: name of the searched-for ontology class
      :return: The ontology class of the given name if existing else None


   .. py:method:: get_ontology_classes_by_namespace(ontology_namespace: str) -> List[Type[owlready2.Thing]]

      Get all ontologies classes by namespace

      :param ontology_namespace: namespace of the searched-for ontology classes
      :return: A list of the ontology classes under the given namespace


   .. py:method:: get_ontology_classes_by_subname(class_subname: str) -> List[Type[owlready2.Thing]]

      Get all ontologies classes by subname

      :param class_subname: a string as part of the full names of the searched-for ontology classes
      :return: A list of the ontology classes of which the name contains the given subname


   .. py:method:: get_ontology_descendant_classes(ancestor_class: Type[owlready2.Thing], class_subname: str = '') -> List[Type[owlready2.Thing]]

      Get ontology descendant classes of an ancestor class given descendant class subname

      :param class_subname: a string as part of the ancestor class full name
      :return: A list of the ontology descendant classes


   .. py:method:: create_ontology_triple_classes(subject_class_name: str, object_class_name: str, predicate_name: str, inverse_predicate_name: str, ontology_subject_parent_class: Optional[Type[owlready2.Thing]] = None, ontology_object_parent_class: Optional[Type[owlready2.Thing]] = None, ontology_property_parent_class: Optional[Type[owlready2.Property]] = ObjectProperty, ontology_inverse_property_parent_class: Optional[Type[owlready2.Property]] = ObjectProperty) -> None

      Dynamically create ontology triple classes under same namespace with the main ontology,
      as known as {subject, predicate, object}, with the relations among them

      :param subject_class_name: name of the subject class
      :param object_class_name: name of the object class
      :param predicate_name: name of predicate class, also used as a Python attribute of the subject class to query object instances
      :param inverse_predicate_name: name of inverse predicate
      :param ontology_subject_parent_class: a parent class of the subject class
      :param ontology_object_parent_class: a parent class of the object class
      :param ontology_property_parent_class: a parent ontology property class, default: owlready2.ObjectProperty
      :param ontology_inverse_property_parent_class: a parent ontology inverse property class, default: owlready2.ObjectProperty


   .. py:method:: create_ontology_linked_designator(designator_class: Type[pycram.designator.DesignatorDescription], ontology_concept_name: str, object_name: Optional[str] = '', ontology_parent_class: Optional[Type[owlready2.Thing]] = None) -> Optional[pycram.designator.DesignatorDescription]

      Create a designator linked to a given ontology concept

      :param designator_class: A given designator class
      :param ontology_concept_name: Ontology concept name
      :param object_name: Name of object in case of the designator to be created is an Object Designator
      :param ontology_parent_class: Parent ontology class from which the class of designator inherits
      :return: A designator associated with an ontology concept


   .. py:method:: create_ontology_linked_designator_by_concept(designator_class: Type[pycram.designator.DesignatorDescription], ontology_concept_class: Type[owlready2.Thing], object_name: Optional[str] = '') -> Optional[pycram.designator.DesignatorDescription]

      Create a designator that belongs to a given ontology concept class

      :param designator_class: A given designator class
      :param ontology_concept_class: An ontology concept class with which the output designator is associated
      :param object_name: Name of object in case of the designator to be created is an Object Designator
      :return: An object designator associated with the given ontology concept class if created successfully (not already exists), None otherwise


   .. py:method:: set_ontology_concept_designator_connection(designator: pycram.designator.DesignatorDescription, ontology_concept_holder: pycram.ontology.ontology_common.OntologyConceptHolder) -> None
      :staticmethod:

      Set two-way connection between a designator and an ontology concept

      :param designator: Designator
      :param ontology_concept_holder: Ontology concept holder


   .. py:method:: set_ontology_relation(subject_designator: pycram.designator.DesignatorDescription, object_designator: pycram.designator.DesignatorDescription, predicate_name: str) -> bool
      :staticmethod:

      Set ontology relation between subject and object designators

      :param subject_designator: An object designator as the ontology subject
      :param object_designator: An object designator as the ontology object
      :param predicate_name: Name of the predicate
      :return: True if the relation is set, False otherwise


   .. py:method:: get_designators_by_subject_predicate(subject: pycram.designator.DesignatorDescription, predicate_name: str) -> List[pycram.designator.DesignatorDescription]
      :staticmethod:

      Get list of designators of an ontology-object concept given a subject designator and predicate

      :param subject: The ontology-subject designator
      :param predicate_name: The ontology-predicate name of the relation
      :return: List of object designators


   .. py:method:: create_ontology_object_designator_from_type(object_type: pycram.datastructures.enums.ObjectType, ontology_concept_class: Type[owlready2.Thing]) -> Optional[pycram.designator.ObjectDesignatorDescription]

      Create an object designator associated with an ontology concept class from a given object type

      :param object_type: An enumerated type of object
      :param ontology_concept_class: An ontology concept class
      :return: An object designator if created successfully (if not already existing), otherwise None


   .. py:method:: destroy_ontology_class(ontology_class, destroy_instances: bool = True)
      :staticmethod:

      Destroy all classes of an ontology

      :param ontology_class: The ontology class to be destroyed
      :param destroy_instances: Whether to destroy instances of those ontology classes



