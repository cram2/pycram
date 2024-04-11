from __future__ import annotations

from typing import Callable, Dict, List, Optional, Type, TYPE_CHECKING
import rospy

from pycram.helper import Singleton
if TYPE_CHECKING:
    from pycram.designator import DesignatorDescription

try:
    import owlready2
    from owlready2 import *
except ImportError:
    owlready2 = None
    rospy.logwarn("Could not import owlready2, OntologyConceptHolder will not be available!")


class OntologyConceptHolderStore(object, metaclass=Singleton):
    """
    Singleton class storing all instances of `OntologyConceptHolder`

    Attributes
    ----------
    __all_ontology_concept_holders: Dict[str, OntologyConceptHolder]
        Dictionary of all ontology concept holders, keyed by concept names
    """

    def __init__(self):
        """
        Initialize the OntologyConceptHolderStore
        """
        if owlready2 is None:
            return
        self.__all_ontology_concept_holders: Dict[str, OntologyConceptHolder] = {}

    def add_ontology_concept_holder(self, ontology_concept_name: str, ontology_concept_holder: OntologyConceptHolder)\
            -> bool:
        if ontology_concept_name in self.__all_ontology_concept_holders:
            rospy.logerr(f"OntologyConceptHolder for `{ontology_concept_name}` was already created!")
            return False
        else:
            self.__all_ontology_concept_holders.setdefault(ontology_concept_name, ontology_concept_holder)
            return True

    def remove_ontology_concept(self, ontology_concept_name: str) -> bool:
        """
        Remove an ontology concept from the store
        """
        if ontology_concept_name in self.__all_ontology_concept_holders:
            del self.__all_ontology_concept_holders[ontology_concept_name]
            return True
        return False

    def get_ontology_concepts_by_class(self, ontology_concept_class: Type[owlready2.Thing]) -> List[owlready2.Thing]:
        """
        Get a list of ontology concepts for a given class
        :ontology_concept_class: An ontology concept class
        """
        return list(itertools.chain(
            *[concept_holder.ontology_concept
              for concept_holder in self.__all_ontology_concept_holders.values()
              if owlready2.issubclass(concept_holder.ontology_concept, ontology_concept_class)]))

    def get_ontology_concept_by_name(self, ontology_concept_name: str) -> owlready2.Thing | None:
        """
        Get the ontology concept holder for one of a given name if exists, otherwise None
        :ontology_concept_name: Name of an ontology concept
        """
        concept_holder = self.__all_ontology_concept_holders.get(ontology_concept_name)
        return concept_holder.ontology_concept if concept_holder else None

    def get_ontology_concept_holders_by_class(self, ontology_concept_class: Type[owlready2.Thing]) \
            -> List[OntologyConceptHolder]:
        """
        Get a list of ontology concept holders for the given ontology concept class
        :ontology_concept_class: An ontology concept class
        """
        return [concept_holder for concept_holder in self.__all_ontology_concept_holders.values()
                if isinstance(concept_holder.ontology_concept, ontology_concept_class)]

    def get_ontology_concept_holder_by_name(self, ontology_concept_name: str) -> OntologyConceptHolder | None:
        """
        Get the ontology concept holder for one of a given name if exists, otherwise None
        :ontology_concept_name: Name of an ontology concept
        """
        return self.__all_ontology_concept_holders.get(ontology_concept_name)

    @staticmethod
    def get_ontology_concepts_of_designator(designator: DesignatorDescription) -> List[owlready2.Thing]:
        """
        Get the corresponding ontology concepts for a given designator
        :param designator: A designator associated with an ontology concept
        """
        return [concept_holder.ontology_concept for concept_holder in designator.ontology_concept_holders]

    def get_designators_of_ontology_concept(self, ontology_concept_name: str) -> List[DesignatorDescription]:
        """
        Get the corresponding designators associated with the given ontology concept
        :param ontology_concept_name: An ontology concept name
        """
        return self.__all_ontology_concept_holders[ontology_concept_name].designators \
            if ontology_concept_name in self.__all_ontology_concept_holders else []


class OntologyConceptHolder(object):
    """
    Wrapper of an ontology concept that is either dynamically created or loaded from an ontology.
    NOTE: Since an ontology concept class, after being saved into an ontology file, must be reusable in the next time
    the ontology is loaded, there must be no other attributes should be created for it aside from ones inherited from `owlready2.Thing`!

    Attributes
    ----------
    ontology_concept: owlready2.Thing
        An ontology concept, either dynamically created, or loaded from an ontology
    designators: List[DesignatorDescription]
        List of designators associated with this ontology concept
    resolve: Callable
        A callable used to resolve the designators to whatever of interest, like designators or their resolving results
    concept_holder_store: OntologyConceptHolder
        The store for all OntologyConceptHolder instances
    """

    def __init__(self, ontology_concept: owlready2.Thing):
        """
        :param ontology_concept: An ontology concept instance
        """
        if owlready2 is None:
            return

        self.ontology_concept: owlready2.Thing = ontology_concept
        self.designators: List[DesignatorDescription] = []
        self.resolve: Optional[Callable] = None

        self.concept_holder_store: OntologyConceptHolderStore = OntologyConceptHolderStore()
        self.concept_holder_store.add_ontology_concept_holder(ontology_concept.name, self)

    @property
    def name(self) -> str:
        """
        :return: Ontology concept name
        """
        return self.ontology_concept.name if self.ontology_concept else ""

    def get_default_designator(self) -> DesignatorDescription | None:
        """
        Get the first element of designators if there is, else None
        """
        return self.designators[0] if len(self.designators) > 0 else None

    def has_designator(self, designator) -> bool:
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

    def __eq__(self, other: OntologyConceptHolder) -> bool:
        """
        Equality check based on name of the ontology concept
        :param other: Other ontology concept instance to check against
        """
        return ((self.ontology_concept == other.ontology_concept) or
                (self.ontology_concept.name == other.ontology_concept.name))
