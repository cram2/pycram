import logging
from typing import Optional, List, Type
import rospy

try:
    import owlready2
    from owlready2 import *
except ImportError:
    owlready2 = None
    logging.warn("Could not import owlready2")


class OntologyConceptHolder(object):
    """
    Wrapper of an ontology concept that is either dynamically created or loaded from an ontology.
    NOTE: Since an ontology concept class, after being saved into an ontology file, must be reusable in the next time
    the ontology is loaded, there must be no other attributes should be created for it aside from ones inherited from `owlready2.Thing`!

    Attributes
    ----------
    designators: List[DesignatorDescription]
        List of designators associated with this ontology concept
    resolve: Callable
        A callable used to resolve the designators to whatever of interest, like designators or their resolving results
    ontology_concept: owlready2.Thing
        An ontology concept, either dynamically created, or loaded from an ontology
    """
    __all_ontology_concept_holders = {}

    def __init__(self, ontology_concept: owlready2.Thing):
        """
        :param ontology_concept: An ontology concept instance
        """
        self.ontology_concept = ontology_concept
        self.designators = []
        self.resolve = None
        if ontology_concept.name in self.__all_ontology_concept_holders:
            rospy.logerr(f"OntologyConceptHolder for [{ontology_concept.name}] was already created!")
        self.__all_ontology_concept_holders.setdefault(ontology_concept.name, self)

    @property
    def name(self):
        """
        :return: Ontology concept name
        """
        return self.ontology_concept.name if self.ontology_concept else ""

    @classmethod
    def remove_ontology_concept(cls, ontology_concept_name: str):
        """
        Remove an ontology concept from `__all_ontology_concept_holders`
        """
        if ontology_concept_name in cls.__all_ontology_concept_holders:
            del cls.__all_ontology_concept_holders[ontology_concept_name]

    def __eq__(self, other):
        """
        Equality check based on name of the ontology concept
        :param other: Other ontology concept instance to check against
        """
        return ((self.ontology_concept == other.ontology_concept) or
                (self.ontology_concept.name == other.ontology_concept.name))

    def get_default_designator(self):
        """
        :return: The first element of designators if there is, else None
        """
        return self.designators[0] if len(self.designators) > 0 else None

    @classmethod
    def get_ontology_concepts_by_class(cls, ontology_concept_class: Type[owlready2.Thing]) -> List[owlready2.Thing]:
        """
        :ontology_concept_class: An ontology concept class
        :return: A list of ontology concepts for a given class
        """
        return list(itertools.chain(
            *[concept_holder.ontology_concept
              for concept_holder in cls.__all_ontology_concept_holders.values()
              if owlready2.issubclass(concept_holder.ontology_concept, ontology_concept_class)]))

    @classmethod
    def get_ontology_concept_by_name(cls, ontology_concept_name: str) -> owlready2.Thing:
        """
        :ontology_concept_name: Name of an ontology concept
        Return the ontology concept holder for one of a given name if exists, otherwise None
        """
        concept_holder = cls.__all_ontology_concept_holders.get(ontology_concept_name)
        return concept_holder.ontology_concept if concept_holder else None

    @classmethod
    def get_ontology_concept_holders_by_class(cls, ontology_concept_class: Type[owlready2.Thing]):
        """
        :ontology_concept_class: An ontology concept class
        Return a list of ontology concept holders for the given ontology concept class
        """
        return [concept_holder for concept_holder in cls.__all_ontology_concept_holders.values()
                if isinstance(concept_holder.ontology_concept, ontology_concept_class)]

    @classmethod
    def get_ontology_concept_holder_by_name(cls, ontology_concept_name: str):
        """
        :ontology_concept_name: Name of an ontology concept
        Return the ontology concept holder for one of a given name if exists, otherwise None
        """
        return cls.__all_ontology_concept_holders.get(ontology_concept_name)

    @classmethod
    def get_ontology_concept_of_designator(cls, designator):
        """
        :param designator: A designator associated with an ontology concept
        :return: The corresponding ontology concept for a given designator
        """
        for ontology_concept_holder in cls.__all_ontology_concept_holders.values():
            if designator in ontology_concept_holder.designators:
                return ontology_concept_holder.ontology_concept
        return None

    @classmethod
    def get_designators_of_ontology_concept(cls, ontology_concept_name: str):
        """
        :param ontology_concept_name: An ontology concept name
        :return: The corresponding designators associated with the given ontology concept
        """
        return cls.__all_ontology_concept_holders[ontology_concept_name].designators \
            if ontology_concept_name in cls.__all_ontology_concept_holders else []

    def has_designator(self, designator):
        """
        :return: True if a given designator was registered by this ontology concept holder, either by itself or under
        another of the same name
        """
        if designator in self.designators:
            return True
        if not hasattr(designator, "name"):
            return False
        for our_designator in self.designators:
            if hasattr(our_designator, "name") and (getattr(our_designator, "name") == getattr(designator, "name")):
                return True
        return False
