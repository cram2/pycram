:py:mod:`pycram.ontology.ontology_common`
=========================================

.. py:module:: pycram.ontology.ontology_common


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.ontology.ontology_common.OntologyConceptHolderStore
   pycram.ontology.ontology_common.OntologyConceptHolder




.. py:class:: OntologyConceptHolderStore


   Bases: :py:obj:`object`

   Singleton class storing all instances of `OntologyConceptHolder`

   Initialize the OntologyConceptHolderStore

   .. py:method:: add_ontology_concept_holder(ontology_concept_name: str, ontology_concept_holder: OntologyConceptHolder) -> bool

      Add an ontology concept to the store

      :param ontology_concept_name: Name of the ontology concept to be removed
      :return: True if the ontology concept can be added into the concept store (if not already existing), otherwise False


   .. py:method:: remove_ontology_concept(ontology_concept_name: str) -> bool

      Remove an ontology concept from the store

      :param ontology_concept_name: Name of the ontology concept to be removed
      :return: True if the ontology concept can be removed from the concept store (if existing), otherwise False


   .. py:method:: get_ontology_concepts_by_class(ontology_concept_class: Type[owlready2.Thing]) -> List[owlready2.Thing]

      Get a list of ontology concepts for a given class

      :param ontology_concept_class: An ontology concept class
      :return: A list of ontology concepts of which the type is either the given class or its subclass


   .. py:method:: get_ontology_concept_by_name(ontology_concept_name: str) -> Optional[owlready2.Thing]

      Get the ontology concept of a given name if exists, otherwise None

      :param ontology_concept_name: Name of an ontology concept
      :return: The ontology concept of a given name if exists or None otherwise


   .. py:method:: get_ontology_concept_holders_by_class(ontology_concept_class: Type[owlready2.Thing]) -> List[OntologyConceptHolder]

      Get a list of ontology concept holders for a given ontology concept class

      :param ontology_concept_class: An ontology concept class
      :return: A list of ontology concept holders as instances of a given ontology concept class


   .. py:method:: get_ontology_concept_holder_by_name(ontology_concept_name: str) -> Optional[OntologyConceptHolder]

      Get the ontology concept holder for one of a given name if exists, otherwise None

      :param ontology_concept_name: Name of an ontology concept
      :return: The ontology concept holder for one of a given name if exists, otherwise None


   .. py:method:: get_ontology_concepts_of_designator(designator: pycram.designator.DesignatorDescription) -> List[owlready2.Thing]
      :staticmethod:

      Get the corresponding ontology concepts for a given designator

      :param designator: A designator associated with an ontology concept
      :return: A list of ontology concepts corresponding with a given designator


   .. py:method:: get_designators_of_ontology_concept(ontology_concept_name: str) -> List[pycram.designator.DesignatorDescription]

      Get the corresponding designators associated with a given ontology concept

      :param ontology_concept_name: An ontology concept name
      :return: A list of designators corresponding to a given ontology concept



.. py:class:: OntologyConceptHolder(ontology_concept: owlready2.Thing)


   Bases: :py:obj:`object`

   Wrapper of an ontology concept that is either dynamically created or loaded from an ontology.
   NOTE: Since an ontology concept class, after being saved into an ontology file, must be reusable in the next time
   the ontology is loaded, there must be no other attributes of it that should be created aside from ones inherited from `owlready2.Thing`!

   :ivar ontology_concept: An ontology concept, either dynamically created, or loaded from an ontology

   Initialize a holder of a given ontology concept instance

   :param ontology_concept: An ontology concept instance

   .. py:property:: name
      :type: str

      Get name of the ontology concept owned by this holder

      :return: Ontology concept name

   .. py:method:: get_default_designator() -> Optional[pycram.designator.DesignatorDescription]

      Get the first element of designators if there is, else None

      :return: The first designator associated with the ontology concept held by this holder if exists or None


   .. py:method:: has_designator(designator) -> bool

      Check whether this ontology concept holder has a given designator registered with its ontology concept

      :return: True if a given designator was registered by this ontology concept holder, either by itself or under another of the same name


   .. py:method:: __eq__(other: OntologyConceptHolder) -> bool

      Equality check based on name of the ontology concept

      :param other: Other ontology concept instance to check against
      :return: True if the ontology concept of the other holder has the same name with the current one, otherwise False



