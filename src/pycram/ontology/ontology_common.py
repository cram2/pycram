from __future__ import annotations

import itertools
from typing import Callable, Dict, List, Optional, Type, TYPE_CHECKING
import rospy

from ..helper import Singleton
if TYPE_CHECKING:
    from ..designator import DesignatorDescription

from owlready2 import issubclass, Thing


class OntologyConceptHolderStore(object, metaclass=Singleton):
    """
    Singleton class storing all instances of `OntologyConceptHolder`
    """

    def __init__(self):
        """
        Initialize the OntologyConceptHolderStore
        """
        # Dictionary of all ontology concept holders, keyed by concept names
        self.__all_ontology_concept_holders: Dict[str, OntologyConceptHolder] = {}

    def add_ontology_concept_holder(self, ontology_concept_name: str, ontology_concept_holder: OntologyConceptHolder)\
            -> bool:
        """
        Add an ontology concept to the store

        :param ontology_concept_name: Name of the ontology concept to be removed
        :return: True if the ontology concept can be added into the concept store (if not already existing), otherwise False
        """
        if ontology_concept_name in self.__all_ontology_concept_holders:
            rospy.logerr(f"OntologyConceptHolder for `{ontology_concept_name}` was already created!")
            return False
        else:
            self.__all_ontology_concept_holders.setdefault(ontology_concept_name, ontology_concept_holder)
            return True

    def remove_ontology_concept(self, ontology_concept_name: str) -> bool:
        """
        Remove an ontology concept from the store

        :param ontology_concept_name: Name of the ontology concept to be removed
        :return: True if the ontology concept can be removed from the concept store (if existing), otherwise False
        """
        if ontology_concept_name in self.__all_ontology_concept_holders:
            del self.__all_ontology_concept_holders[ontology_concept_name]
            return True
        return False

    def get_ontology_concepts_by_class(self, ontology_concept_class: Type[Thing]) -> List[Thing]:
        """
        Get a list of ontology concepts for a given class

        :param ontology_concept_class: An ontology concept class
        :return: A list of ontology concepts of which the type is either the given class or its subclass
        """
        return list(itertools.chain(
            *[concept_holder.ontology_concept
              for concept_holder in self.__all_ontology_concept_holders.values()
              if issubclass(concept_holder.ontology_concept, ontology_concept_class)]))

    def get_ontology_concept_by_name(self, ontology_concept_name: str) -> Optional[Thing]:
        """
        Get the ontology concept of a given name if exists, otherwise None

        :param ontology_concept_name: Name of an ontology concept
        :return: The ontology concept of a given name if exists or None otherwise
        """
        concept_holder = self.__all_ontology_concept_holders.get(ontology_concept_name)
        return concept_holder.ontology_concept if concept_holder else None

    def get_ontology_concept_holders_by_class(self, ontology_concept_class: Type[Thing]) \
            -> List[OntologyConceptHolder]:
        """
        Get a list of ontology concept holders for a given ontology concept class

        :param ontology_concept_class: An ontology concept class
        :return: A list of ontology concept holders as instances of a given ontology concept class
        """
        return [concept_holder for concept_holder in self.__all_ontology_concept_holders.values()
                if isinstance(concept_holder.ontology_concept, ontology_concept_class)]

    def get_ontology_concept_holder_by_name(self, ontology_concept_name: str) -> Optional[OntologyConceptHolder]:
        """
        Get the ontology concept holder for one of a given name if exists, otherwise None

        :param ontology_concept_name: Name of an ontology concept
        :return: The ontology concept holder for one of a given name if exists, otherwise None
        """
        return self.__all_ontology_concept_holders.get(ontology_concept_name)

    @staticmethod
    def get_ontology_concepts_of_designator(designator: DesignatorDescription) -> List[Thing]:
        """
        Get the corresponding ontology concepts for a given designator

        :param designator: A designator associated with an ontology concept
        :return: A list of ontology concepts corresponding with a given designator
        """
        return [concept_holder.ontology_concept for concept_holder in designator.ontology_concept_holders]

    def get_designators_of_ontology_concept(self, ontology_concept_name: str) -> List[DesignatorDescription]:
        """
        Get the corresponding designators associated with a given ontology concept

        :param ontology_concept_name: An ontology concept name
        :return: A list of designators corresponding to a given ontology concept
        """
        return self.__all_ontology_concept_holders[ontology_concept_name].designators \
            if ontology_concept_name in self.__all_ontology_concept_holders else []


class OntologyConceptHolder(object):
    """
    Wrapper of an ontology concept that is either dynamically created or loaded from an ontology.
    NOTE: Since an ontology concept class, after being saved into an ontology file, must be reusable in the next time
    the ontology is loaded, there must be no other attributes of it that should be created aside from ones inherited from `owlready2.Thing`!

    :ivar ontology_concept: An ontology concept, either dynamically created, or loaded from an ontology
    """

    def __init__(self, ontology_concept: Thing):
        """
        Initialize a holder of a given ontology concept instance

        :param ontology_concept: An ontology concept instance
        """

        #: An ontology concept, either dynamically created, or loaded from an ontology
        self.ontology_concept: Thing = ontology_concept
        #: List of designators associated with this ontology concept
        self.designators: List[DesignatorDescription] = []
        # A callable used to resolve the designators to whatever of interest, like designators or their resolving results
        self.resolve: Optional[Callable] = None

        #: The store for all OntologyConceptHolder instances
        self.concept_holder_store: OntologyConceptHolderStore = OntologyConceptHolderStore()
        self.concept_holder_store.add_ontology_concept_holder(ontology_concept.name, self)

    @property
    def name(self) -> str:
        """
        Get name of the ontology concept owned by this holder

        :return: Ontology concept name
        """
        return self.ontology_concept.name if self.ontology_concept else ""

    def get_default_designator(self) -> Optional[DesignatorDescription]:
        """
        Get the first element of designators if there is, else None

        :return: The first designator associated with the ontology concept held by this holder if exists or None
        """
        return self.designators[0] if len(self.designators) > 0 else None

    def has_designator(self, designator) -> bool:
        """
        Check whether this ontology concept holder has a given designator registered with its ontology concept

        :return: True if a given designator was registered by this ontology concept holder, either by itself or under another of the same name
        """
        if designator in self.designators:
            return True
        if not hasattr(designator, "name"):
            return False
        for our_designator in self.designators:
            if hasattr(our_designator, "name") and (getattr(our_designator, "name") == getattr(designator, "name")):
                return True
        return False

    def __eq__(self, other: OntologyConceptHolder) -> bool:
        """
        Equality check based on name of the ontology concept

        :param other: Other ontology concept instance to check against
        :return: True if the ontology concept of the other holder has the same name with the current one, otherwise False
        """
        return ((self.ontology_concept == other.ontology_concept) or
                (self.ontology_concept.name == other.ontology_concept.name))
