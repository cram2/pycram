from __future__ import annotations

import inspect
import logging
from pathlib import Path
from typing import Callable, Dict, List, Optional, Type

import rospy

try:
    import owlready2
    from owlready2 import *
except ImportError:
    owlready2 = None
    rospy.logwarn("Could not import owlready2, OntologyManager could not be initialized!")

from pycram.datastructures.enums import ObjectType
from pycram.helper import Singleton
from pycram.designator import DesignatorDescription, ObjectDesignatorDescription

from pycram.ontology.ontology_common import OntologyConceptHolderStore, OntologyConceptHolder

SOMA_HOME_ONTOLOGY_IRI = "http://www.ease-crc.org/ont/SOMA-HOME.owl"
SOMA_ONTOLOGY_IRI = "http://www.ease-crc.org/ont/SOMA.owl"
SOMA_ONTOLOGY_NAMESPACE = "SOMA"
DUL_ONTOLOGY_NAMESPACE = "DUL"


class OntologyManager(object, metaclass=Singleton):
    """
    Singleton class as the adapter accessing data of an OWL ontology, largely based on owlready2.

    Attributes
    ----------
    ontologies: Dict[str, owlready2.Ontology]
        A dictionary of OWL ontologies, keyed by ontology name (same as its namespace name), eg. 'SOMA'

    main_ontology: owlready2.Ontology
        The main ontology instance as the result of an ontology loading operation

    main_ontology_iri: str
        Ontology IRI (Internationalized Resource Identifier), either a URL to a remote OWL file or the full name path of a local one

    main_ontology_namespace: owlready2.Namespace
        Namespace of the main ontology

    soma: owlready2.Ontology
        The SOMA ontology instance, referencing :attr:`ontology` in case of ontology loading from SOMA.owl
        Ref: http://www.ease-crc.org/ont/SOMA.owl

    dul: owlready2.Ontology
        The DUL ontology instance, referencing :attr:`ontology` in case of ontology loading from DUL.owl
        Ref: http://www.ease-crc.org/ont/DUL.owl

    ontology_world:
        Ontology world, the placeholder of triples stored by owlready2.
        Ref: https://owlready2.readthedocs.io/en/latest/world.html
    """

    def __init__(self, main_ontology_iri: str = "", ontology_search_path: str = f"{Path.home()}/ontologies"):
        """
        Create the singleton object of OntologyManager class

        :param main_ontology_iri: Ontology IRI (Internationalized Resource Identifier), either a URL to a remote OWL file
        or the full name path of a local one
        :param ontology_search_path: directory path from which a possibly existing ontology is searched. This is appended
        to `owlready2.onto_path`, a global variable containing a list of directories for searching local copies of
        ontologies (similarly to python `sys.path` for modules/packages).
        """
        if owlready2:
            Path(ontology_search_path).mkdir(parents=True, exist_ok=True)
            owlready2.onto_path.append(ontology_search_path)
        else:
            return

        self.ontologies: Dict[str, owlready2.Ontology] = {}
        self.main_ontology: owlready2.Ontology = None
        self.soma: owlready2.Ontology = None
        self.dul: owlready2.Ontology = None

        self.ontology_world: owlready2.World = None
        self.main_ontology_iri: str = main_ontology_iri
        self.main_ontology_namespace: owlready2.Namespace = None

        # Create an ontology world with parallelized file parsing enabled
        self.ontology_world = World(filename=f"{ontology_search_path}/{Path(main_ontology_iri).stem}.sqlite3",
                                    exclusive=False, enable_thread_parallelism=True)

        self.main_ontology, self.main_ontology_namespace = self.load_ontology(main_ontology_iri)
        if self.main_ontology.loaded:
            self.soma = self.ontologies[SOMA_ONTOLOGY_NAMESPACE]
            self.dul = self.ontologies[DUL_ONTOLOGY_NAMESPACE]

    @staticmethod
    def print_ontology_class(ontology_class):
        """
        Print information (ancestors, super classes, subclasses, properties, etc.) of an ontology class
        """
        if ontology_class is None:
            return
        rospy.loginfo("-------------------")
        rospy.loginfo(f"{ontology_class} {type(ontology_class)}")
        rospy.loginfo(f"Super classes: {ontology_class.is_a}")
        rospy.loginfo(f"Ancestors: {ontology_class.ancestors()}")
        rospy.loginfo(f"Subclasses: {list(ontology_class.subclasses())}")
        rospy.loginfo(f"Properties: {list(ontology_class.get_class_properties())}")
        rospy.loginfo(f"Instances: {list(ontology_class.instances())}")
        rospy.loginfo(f"Direct Instances: {list(ontology_class.direct_instances())}")
        rospy.loginfo(f"Inverse Restrictions: {list(ontology_class.inverse_restrictions())}")

    def load_ontology(self, ontology_iri) -> tuple[owlready2.Ontology, owlready2.Namespace] | None:
        """
        Load an ontology from an IRI
        :param ontology_iri: An ontology IRI
        :return: A tuple including an ontology instance & its namespace
        """
        ontology = self.ontology_world.get_ontology(ontology_iri).load(reload_if_newer=True)
        ontology_namespace = owlready2.get_namespace(ontology_iri)
        if ontology.loaded:
            rospy.loginfo(
                f'Ontology [{ontology.base_iri}]\'s name: {ontology.name} has been loaded')
            rospy.loginfo(f'- main namespace: {ontology_namespace.name}')
            rospy.loginfo(f'- loaded ontologies:')

            def fetch_ontology(ontology__):
                self.ontologies[ontology__.name] = ontology__
                rospy.loginfo(ontology__.base_iri)

            self.browse_ontologies(ontology, condition=None, func=lambda ontology__: fetch_ontology(ontology__))
            return ontology, ontology_namespace
        else:
            rospy.logerr(f"Ontology [{ontology.base_iri}]\'s name: {ontology.name} failed being loaded")
            return None

    def initialized(self) -> bool:
        return hasattr(self, "main_ontology") and self.main_ontology.loaded

    @staticmethod
    def browse_ontologies(ontology: owlready2.Ontology,
                          condition: Optional[Callable] = None, func: Optional[Callable] = None, **kwargs) -> None:
        """
        Browse the loaded ontologies (including the main and imported ones), doing operations based on a condition.

        :param ontology: An ontology instance as the result of ontology loading
        :param condition: a Callable condition that if not None needs to be passed before doing operations, otherwise just
        always carry the operations
        :param func: a Callable specifying the operations to perform on all the loaded ontologies if condition is None,
        otherwise only the first ontology which meets the condition
        """
        if ontology is None:
            rospy.logerr(f"Ontology {ontology=} is None!")
            return
        elif not ontology.loaded:
            rospy.logerr(f"Ontology {ontology} was not loaded!")
            return

        will_do_func = func is not None
        # No condition: Do func for all ontologies
        if condition is None:
            if will_do_func:
                func(ontology, **kwargs)
                for sub_onto in ontology.get_imported_ontologies():
                    func(sub_onto, **kwargs)
        # Else: Only do func for the first ontology which meets the condition
        elif condition(ontology, **kwargs):
            if will_do_func: func(ontology, **kwargs)
        else:
            for sub_onto in ontology.get_imported_ontologies():
                if condition(sub_onto, **kwargs) and will_do_func:
                    func(sub_onto, **kwargs)
                    break

    def save(self, target_filename: str = "", overwrite: bool = False) -> bool:
        """
        Save the current ontology to disk
        :param target_filename: full name path of a file which the ontologies are saved into.
        :param overwrite: overwrite an existing file if it exists.
        If empty, they are saved to the same original OWL file from which the main ontology was loaded, or
        a file at the same folder with ontology search path specified at constructor if it was loaded from a remote IRI.
        :return: True if the ontology was successfully saved, False otherwise
        """

        # Commit the whole graph data of the current ontology world, saving it into SQLite3, to be reused the next time
        # the ontologies are loaded
        self.ontology_world.save()

        # Save ontologies to OWL
        is_current_ontology_local = Path(self.main_ontology_iri).exists()
        current_ontology_filename = self.main_ontology_iri if is_current_ontology_local \
            else f"{Path(self.ontology_world.filename).parent.absolute()}/{Path(self.main_ontology_iri).stem}.owl"
        save_to_same_file = is_current_ontology_local and (target_filename == current_ontology_filename)
        if save_to_same_file and not overwrite:
            rospy.logerr(f"Ontologies cannot be saved to the originally loaded [{target_filename}] if not by overwriting")
            return False
        else:
            save_filename = target_filename if target_filename else current_ontology_filename
            self.main_ontology.save(save_filename)
            if save_to_same_file and overwrite:
                rospy.logwarn(f"Ontologies have been overwritten to {save_filename}")
            else:
                rospy.loginfo(f"Ontologies have been saved to {save_filename}")
            return True

    def create_ontology_concept_class(self, class_name: str,
                                      ontology_parent_concept_class: Optional[owlready2.Thing] = None) \
            -> Type[owlready2.Thing]:
        """
        Create a new concept class in ontology

        :param class_name: A given name to the new class
        :param ontology_parent_concept_class: An optional parent ontology class of the new class
        :return: The created ontology class
        """
        ontology_concept_class = self.get_ontology_class_by_ontology(self.main_ontology, class_name)
        if ontology_concept_class:
            return ontology_concept_class

        with self.main_ontology:
            return types.new_class(class_name, (owlready2.Thing, ontology_parent_concept_class,)
                   if inspect.isclass(ontology_parent_concept_class) else (owlready2.Thing,))

    @staticmethod
    def create_ontology_property_class(class_name: str,
                                       ontology_parent_property_class: Optional[Type[owlready2.Property]] = None) \
            -> Type[owlready2.Property] | None:
        """
        Create a new property class in ontology

        :param class_name: A given name to the new class
        :param ontology_parent_property_class: An optional parent ontology property class of the new class
        :return: The created ontology class
        """
        parent_class = ontology_parent_property_class if (ontology_parent_property_class and
                                                          issubclass(ontology_parent_property_class,
                                                                     owlready2.Property)) \
            else None
        return types.new_class(class_name, (parent_class,) if parent_class else (owlready2.Property,))

    def get_ontology_classes_by_condition(self, condition: Callable, first_match_only=False, **kwargs) \
            -> List[Type[owlready2.Thing]]:
        """
        Get an ontology class by a given condition

        :param condition: condition of searching
        :param first_match_only: whether to only fetch the first class matching the given condition
        :return: The ontology class satisfying the given condition if found else None
        """
        out_classes = []
        for ontology_class in list(self.main_ontology.classes()):
            if condition(ontology_class, **kwargs):
                out_classes.append(ontology_class)
                if first_match_only:
                    return out_classes

        for sub_onto in self.main_ontology.get_imported_ontologies():
            for sub_ontology_class in list(sub_onto.classes()):
                if condition(sub_ontology_class, **kwargs):
                    out_classes.append(sub_ontology_class)
                    if first_match_only:
                        return out_classes

        if not out_classes:
            rospy.loginfo(f"No class with {kwargs} is found in the ontology {self.main_ontology}")
        return out_classes

    @staticmethod
    def get_ontology_class_by_ontology(ontology: owlready2.Ontology, class_name: str) -> Type[owlready2.Thing] | None:
        """
        Get an ontology class if it exists in a given ontology

        :param ontology: an ontology instance
        :return: The ontology class if it exists under the namespace of the given ontology, None otherwise
        """
        return getattr(ontology, class_name) if ontology and hasattr(ontology, class_name) else None

    def get_ontology_class(self, class_name: str) -> Type[owlready2.Thing] | None:
        """
        Get an ontology class by name

        :param class_name: name of the searched-for ontology class
        :return: The ontology class of the given name if existing else None
        """

        def is_matching_class_name(ontology_class: Type[owlready2.Thing], ontology_class_name: str):
            return ontology_class.name == ontology_class_name

        found_classes = self.get_ontology_classes_by_condition(condition=is_matching_class_name,
                                                               ontology_class_name=class_name,
                                                               first_match_only=True)
        return found_classes[0] if len(found_classes) > 0 else None

    def get_ontology_classes_by_namespace(self, ontology_namespace: str) -> List[
        Type[owlready2.Thing]]:
        """
        Get all ontologies classes by namespace

        :param ontology_namespace: namespace of the searched-for ontology classes
        :return: A list of the ontology classes under the given namespace
        """

        def is_matching_ontology_namespace(ontology_class: Type[owlready2.Thing], ontology_namespace_: str):
            return ontology_class.namespace.name == ontology_namespace_

        return self.get_ontology_classes_by_condition(condition=is_matching_ontology_namespace,
                                                      ontology_namespace_=ontology_namespace)

    def get_ontology_classes_by_subname(self, class_subname: str) -> List[Type[owlready2.Thing]]:
        """
        Get all ontologies classes by subname

        :param class_subname: a string as part of the full names of the searched-for ontology classes
        :return: A list of the ontology classes of which the name contains the given subname
        """

        def is_matching_class_subname(ontology_class: Type[owlready2.Thing], ontology_class_subname: str):
            return ontology_class_subname.lower() in ontology_class.name.lower()

        return self.get_ontology_classes_by_condition(condition=is_matching_class_subname,
                                                      ontology_class_subname=class_subname)

    def get_ontology_descendant_classes(self, ancestor_class: Type[owlready2.Thing], class_subname: str = "") \
            -> List[Type[owlready2.Thing]]:
        """
        Get ontology descendant classes of an ancestor class given descendant class subname

        :param class_subname: a string as part of the ancestor class full name
        :return: A list of the ontology descendant classes
        """
        return [ontology_class for ontology_class in self.main_ontology.classes()
                if (class_subname.lower() in ontology_class.name.lower()) and
                (ancestor_class in ontology_class.ancestors())]

    def create_ontology_triple_classes(self, subject_class_name: str, object_class_name: str,
                                       predicate_name: str, inverse_predicate_name: str,
                                       ontology_subject_parent_class: Optional[Type[owlready2.Thing]] = None,
                                       ontology_object_parent_class: Optional[Type[owlready2.Thing]] = None,
                                       ontology_property_parent_class: Optional[Type[
                                           owlready2.Property]] = owlready2.ObjectProperty if owlready2 else None,
                                       ontology_inverse_property_parent_class: Optional[Type[
                                           owlready2.Property]] = owlready2.ObjectProperty if owlready2 else None) -> None:
        """
        Dynamically create ontology triple classes under same namespace with the main ontology,
        as known as {subject, predicate, object}, with the relations among them

        :param subject_class_name: name of the subject class
        :param object_class_name: name of the object class
        :param predicate_name: name of predicate class, also used as a Python attribute of the subject class to
        query object instances
        :param inverse_predicate_name: name of inverse predicate
        :param ontology_subject_parent_class: a parent class of the subject class
        :param ontology_object_parent_class: a parent class of the object class
        :param ontology_property_parent_class: a parent ontology property class, default: owlready2.ObjectProperty
        :param ontology_inverse_property_parent_class: a parent ontology inverse property class, default: owlready2.ObjectProperty
        """

        # This context manager ensures all classes created here-in share the same namepsace with `self.main_ontology`
        with self.main_ontology:
            # Subject
            ontology_subject_class = self.create_ontology_concept_class(subject_class_name,
                                                                        ontology_subject_parent_class)

            # Object
            ontology_object_class = self.create_ontology_concept_class(object_class_name, ontology_object_parent_class)

            # Predicate
            ontology_predicate_class = self.create_ontology_property_class("OntologyPredicate",
                                                                           ontology_property_parent_class)
            ontology_predicate_class.domain = [ontology_subject_class]
            ontology_predicate_class.range = [ontology_object_class]
            ontology_predicate_class.python_name = predicate_name

            # Inverse Predicate
            ontology_inverse_predicate = self.create_ontology_property_class("OntologyInversePredicate",
                                                                             ontology_inverse_property_parent_class)
            ontology_inverse_predicate.inverse_property = ontology_predicate_class
            ontology_inverse_predicate.python_name = inverse_predicate_name

    def create_ontology_linked_designator(self, designator_name: str, designator_class: Type[DesignatorDescription],
                                          ontology_concept_name: str,
                                          ontology_parent_class: Optional[Type[owlready2.Thing]] = None) \
            -> DesignatorDescription | None:
        """
        Create an object designator linked to a given ontology concept

        :param designator_name: Designator name
        :param designator_class: Designator class
        :param ontology_concept_name: Ontology concept name
        :param ontology_parent_class: Parent ontology class from which the class of designator inherits
        :return: An object designator associated with an ontology concept
        """
        ontology_concept_class = self.create_ontology_concept_class(ontology_concept_name, ontology_parent_class)
        return self.create_ontology_linked_designator_by_concept(designator_name, designator_class,
                                                                 ontology_concept_class)

    def create_ontology_linked_designator_by_concept(self, designator_name: str,
                                                     designator_class: Type[DesignatorDescription],
                                                     ontology_concept_class: Type[owlready2.Thing]) \
            -> DesignatorDescription | None:
        """
        Create a designator that belongs to a given ontology concept class

        :param designator_name: Designator name
        :param designator_class: Designator class
        :param ontology_concept_class: An ontology concept class which the output designator is associated with
        :return: An object designator associated with the given ontology concept class
        """
        ontology_concept_name = f'{designator_name}_concept'
        if len(OntologyConceptHolderStore().get_designators_of_ontology_concept(ontology_concept_name)) > 0:
            rospy.logerr(f"A designator named [{designator_name}] is already created for ontology concept [{ontology_concept_name}]")
            return None

        # Create a designator of `designator_class`
        designator = designator_class(names=[designator_name]) if issubclass(designator_class, ObjectDesignatorDescription) \
                                                               else designator_class()

        # Link designator with an ontology concept of `ontology_concept_class`
        ontology_concept_holder = OntologyConceptHolderStore().get_ontology_concept_holder_by_name(ontology_concept_name)
        if ontology_concept_holder is None:
            ontology_concept_holder = OntologyConceptHolder(ontology_concept_class(name=ontology_concept_name,
                                                                                   namespace=self.main_ontology))
        self.set_ontology_concept_designator_connection(designator, ontology_concept_holder)
        return designator

    @staticmethod
    def set_ontology_concept_designator_connection(designator: DesignatorDescription,
                                                   ontology_concept_holder: OntologyConceptHolder) -> None:
        """
        Set two-way connection between a designator and an ontology concept

        :param designator: Designator
        :param ontology_concept_holder: Ontology concept holder
        """
        if ontology_concept_holder not in designator.ontology_concept_holders:
            designator.ontology_concept_holders.append(ontology_concept_holder)

        if not ontology_concept_holder.has_designator(designator):
            ontology_concept_holder.designators.append(designator)

    @staticmethod
    def set_ontology_relation(subject_designator: DesignatorDescription,
                              object_designator: DesignatorDescription,
                              predicate_name: str) -> bool:
        """
        Set ontology relation between subject and object designators

        :param subject_designator: An object designator as the ontology subject
        :param object_designator: An object designator as the ontology object
        :param predicate_name: Name of the predicate
        :return: True if the relation is set, False otherwise
        """
        for subject_concept_holder in subject_designator.ontology_concept_holders:
            subject_concept = subject_concept_holder.ontology_concept
            if hasattr(subject_concept, predicate_name):
                object_concepts_list = getattr(subject_concept, predicate_name)
                object_concepts_names = [concept.name for concept in object_concepts_list]
                for holder in object_designator.ontology_concept_holders:
                    if holder.ontology_concept.name not in object_concepts_names:
                        object_concepts_list.append(holder.ontology_concept)
                return True
            else:
                rospy.logerr(f"Ontology concept [{subject_concept.name}] has no predicate named [{predicate_name}]")
                return False

    @staticmethod
    def get_designators_by_subject_predicate(subject: DesignatorDescription,
                                             predicate_name: str) -> List[DesignatorDescription]:
        """
        Get list of designators of an ontology-object concept given a subject designator and predicate

        :param subject: The ontology-subject designator
        :param predicate_name: The ontology-predicate name of the relation
        :return: List of object designators
        """
        return list(itertools.chain(
            *[OntologyConceptHolderStore().get_designators_of_ontology_concept(object_concept.name)
              for subject_concept_holder in subject.ontology_concept_holders
              for object_concept in getattr(subject_concept_holder.ontology_concept, predicate_name)
              if hasattr(subject_concept_holder.ontology_concept, predicate_name)]))

    def create_ontology_object_designator_from_type(self, object_type: ObjectType,
                                                    ontology_concept_class: Type[owlready2.Thing]) \
            -> ObjectDesignatorDescription:
        object_type_name = object_type.name.lower()
        object_designator = \
            self.create_ontology_linked_designator_by_concept(object_type_name,
                                                              ObjectDesignatorDescription,
                                                              ontology_concept_class)
        object_designator.types = [object_type_name]
        return object_designator

    @staticmethod
    def destroy_ontology_class(ontology_class, destroy_instances: bool = True):
        """
        Destroy all classes of an ontology
        :param ontology_class: The ontology class to be destroyed
        :param destroy_instances: Whether to destroy instances of those ontology classes
        """
        if destroy_instances:
            for ontology_individual in ontology_class.instances():
                destroy_entity(ontology_individual)
            OntologyConceptHolderStore().remove_ontology_concept(ontology_class.name)
        destroy_entity(ontology_class)
