from __future__ import annotations

import inspect
import itertools
import logging
import os.path
import sqlite3

from pathlib import Path
from typing import Callable, Dict, List, Optional, Type, Tuple, Union

from owlready2 import (Namespace, Ontology, World as OntologyWorld, Thing, EntityClass, Imp,
                       Property, ObjectProperty, OwlReadyError, types,
                       onto_path, default_world, get_namespace, get_ontology, destroy_entity,
                       sync_reasoner_pellet, sync_reasoner_hermit,
                       OwlReadyOntologyParsingError)
from owlready2.class_construct import GeneralClassAxiom

from ..datastructures.enums import ObjectType
from ..helper import Singleton
from ..designator import DesignatorDescription, ObjectDesignatorDescription

from ..ontology.ontology_common import (OntologyConceptHolderStore, OntologyConceptHolder,
                                        ONTOLOGY_SQL_BACKEND_FILE_EXTENSION,
                                        ONTOLOGY_SQL_IN_MEMORY_BACKEND)
from ..ros.logging import loginfo, logerr, logwarn

SOMA_HOME_ONTOLOGY_IRI = "http://www.ease-crc.org/ont/SOMA-HOME.owl"
SOMA_ONTOLOGY_IRI = "http://www.ease-crc.org/ont/SOMA.owl"
SOMA_ONTOLOGY_NAMESPACE = "SOMA"
DUL_ONTOLOGY_NAMESPACE = "DUL"


class OntologyManager(object, metaclass=Singleton):
    """
    Singleton class as the adapter accessing data of an OWL ontology, largely based on owlready2.
    """

    def __init__(self, main_ontology_iri: Optional[str] = None, main_sql_backend_filename: Optional[str] = None,
                 ontology_search_path: Optional[str] = None,
                 use_global_default_world: bool = True):
        """
        Create the singleton object of OntologyManager class

        :param main_ontology_iri: Ontology IRI (Internationalized Resource Identifier), either a URL to a remote OWL file or the full name path of a local one
        :param main_sql_backend_filename: a full file path (no need to already exist) being used as SQL backend for the ontology world. If None, in-memory is used instead
        :param ontology_search_path: directory path from which a possibly existing ontology is searched. This is appended to `owlready2.onto_path`, a global variable containing a list of directories for searching local copies of ontologies (similarly to python `sys.path` for modules/packages). If not specified, the path is "$HOME/ontologies"
        :param use_global_default_world: whether or not using the owlready2-provided global default persistent world
        """
        if not ontology_search_path:
            ontology_search_path = f"{Path.home()}/ontologies"
        Path(ontology_search_path).mkdir(parents=True, exist_ok=True)
        onto_path.append(ontology_search_path)

        #: A dictionary of OWL ontologies, keyed by ontology name (same as its namespace name), eg. 'SOMA'
        self.ontologies: Dict[str, Ontology] = {}

        #: The main ontology instance created by Ontology Manager at initialization as the result of loading from `main_ontology_iri`
        self.main_ontology: Optional[Ontology] = None

        #: The SOMA ontology instance, referencing :attr:`main_ontology` in case of ontology loading from `SOMA.owl`.
        # Ref: http://www.ease-crc.org/ont/SOMA.owl
        self.soma: Optional[Ontology] = None

        #: The DUL ontology instance, referencing :attr:`main_ontology` in case of ontology loading from `DUL.owl`.
        # Ref: http://www.ease-crc.org/ont/DUL.owl
        self.dul: Optional[Ontology] = None

        #: The main ontology world, the placeholder of triples created in :attr:`main_ontology`.
        # Ref: https://owlready2.readthedocs.io/en/latest/world.html
        self.main_ontology_world: Optional[OntologyWorld] = None

        #: Ontology IRI (Internationalized Resource Identifier), either a URL to a remote OWL file or the full name path of a local one
        # Ref: https://owlready2.readthedocs.io/en/latest/onto.html
        self.main_ontology_iri: str = main_ontology_iri if main_ontology_iri else SOMA_HOME_ONTOLOGY_IRI

        #: Namespace of the main ontology
        self.main_ontology_namespace: Optional[Namespace] = None

        #: SQL backend for :attr:`main_ontology_world`, being either "memory" or a full file path (no need to already exist)
        self.main_ontology_sql_backend = main_sql_backend_filename if main_sql_backend_filename else ONTOLOGY_SQL_IN_MEMORY_BACKEND

        # Create the main ontology world holding triples
        self.create_main_ontology_world(use_global_default_world=use_global_default_world)

        # Create the main ontology & its namespace, fetching :attr:`soma`, :attr:`dul` if loading from SOMA ontology
        self.create_main_ontology()

    @staticmethod
    def print_ontology_class(ontology_class: Type[Thing]):
        """
        Print information (ancestors, super classes, subclasses, properties, etc.) of an ontology class

        :param ontology_class: An ontology class
        """
        if ontology_class is None:
            return
        loginfo(f"{ontology_class} {type(ontology_class)}")
        loginfo(f"Defined class: {ontology_class.get_defined_class()}")
        loginfo(f"Super classes: {ontology_class.is_a}")
        loginfo(f"Equivalent to: {EntityClass.get_equivalent_to(ontology_class)}")
        loginfo(f"Indirectly equivalent to: {ontology_class.get_indirect_equivalent_to()}")
        loginfo(f"Ancestors: {list(ontology_class.ancestors())}")
        loginfo(f"Subclasses: {list(ontology_class.subclasses())}")
        loginfo(f"Disjoint unions: {ontology_class.get_disjoint_unions()}")
        loginfo(f"Properties: {list(ontology_class.get_class_properties())}")
        loginfo(f"Indirect Properties: {list(ontology_class.INDIRECT_get_class_properties())}")
        loginfo(f"Instances: {list(ontology_class.instances())}")
        loginfo(f"Direct Instances: {list(ontology_class.direct_instances())}")
        loginfo(f"Inverse Restrictions: {list(ontology_class.inverse_restrictions())}")
        loginfo("-------------------")

    @staticmethod
    def print_ontology_property(ontology_property: Property):
        """
        Print information (subjects, objects, relations, etc.) of an ontology property

        :param ontology_property: An ontology property
        """
        if ontology_property is None:
            return
        property_class = type(ontology_property)
        loginfo(f"{ontology_property} {property_class}")
        loginfo(f"Relations: {list(ontology_property.get_relations())}")
        loginfo(f"Domain: {ontology_property.get_domain()}")
        loginfo(f"Range: {ontology_property.get_range()}")
        if hasattr(property_class, "_equivalent_to"):
            loginfo(f"Equivalent classes: {EntityClass.get_equivalent_to(property_class)}")
        if hasattr(property_class, "_indirect"):
            loginfo(f"Indirectly equivalent classes: {EntityClass.get_indirect_equivalent_to(property_class)}")
        loginfo(f"Property chain: {ontology_property.get_property_chain()}")
        loginfo(f"Class property type: {ontology_property.get_class_property_type()}")
        loginfo("-------------------")

    @staticmethod
    def get_default_ontology_search_path() -> Optional[str]:
        """
        Get the first ontology search path from owlready2.onto_path

        :return: the path to the ontology search path if existing, otherwise None
        """
        if onto_path:
            return onto_path[0]
        else:
            logerr("No ontology search path has been configured!")
            return None

    def get_main_ontology_dir(self) -> Optional[str]:
        """
        Get path to the directory of :attr:`main_ontology_iri` if it is a local absolute path,
        otherwise path to the default ontology search directory

        :return: the path to the directory of the main ontology IRI
        """
        return os.path.dirname(self.main_ontology_iri) if os.path.isabs(
            self.main_ontology_iri) else self.get_default_ontology_search_path()

    def is_main_ontology_sql_backend_in_memory(self) -> bool:
        """
        Whether the main ontology's SQL backend is in-memory

        :return: true if the main ontology's SQL backend is in-memory
        """
        return self.main_ontology_sql_backend == ONTOLOGY_SQL_IN_MEMORY_BACKEND

    def create_main_ontology_world(self, use_global_default_world: bool = True) -> None:
        """
        Create the main ontology world, either reusing the owlready2-provided global default ontology world or create a new one
        A backend sqlite3 file of same name with `main_ontology` is also created at the same folder with :attr:`main_ontology_iri`
        (if it is a local absolute path). The file is automatically registered as cache for the main ontology world.

        :param use_global_default_world: whether or not using the owlready2-provided global default persistent world
        :param sql_backend_filename: a full file path (no need to already exist) being used as SQL backend for the ontology world. If None, memory is used instead
        """
        self.main_ontology_world = self.create_ontology_world(
            sql_backend_filename=self.main_ontology_sql_backend,
            use_global_default_world=use_global_default_world)

    @staticmethod
    def create_ontology_world(use_global_default_world: bool = False,
                              sql_backend_filename: Optional[str] = None) -> OntologyWorld:
        """
        Either reuse the owlready2-provided global default ontology world or create a new one.

        :param use_global_default_world: whether or not using the owlready2-provided global default persistent world
        :param sql_backend_filename: an absolute file path (no need to already exist) being used as SQL backend for the ontology world. If it is None or non-absolute path, in-memory is used instead
        :return: owlready2-provided global default ontology world or a newly created ontology world
        """
        world = default_world
        sql_backend_path_absolute = (sql_backend_filename and os.path.isabs(sql_backend_filename))
        if sql_backend_filename and (sql_backend_filename != ONTOLOGY_SQL_IN_MEMORY_BACKEND):
            if not sql_backend_path_absolute:
                logerr(f"For ontology world accessing, either f{ONTOLOGY_SQL_IN_MEMORY_BACKEND}"
                              f"or an absolute path to its SQL file backend is expected: {sql_backend_filename}")
                return default_world
            elif not sql_backend_filename.endswith(ONTOLOGY_SQL_BACKEND_FILE_EXTENSION):
                logerr(
                    f"Ontology world SQL backend file path, {sql_backend_filename},"
                    f"is expected to be of extension {ONTOLOGY_SQL_BACKEND_FILE_EXTENSION}!")
                return default_world

        sql_backend_path_valid = sql_backend_path_absolute
        sql_backend_name = sql_backend_filename if sql_backend_path_valid else ONTOLOGY_SQL_IN_MEMORY_BACKEND
        try:
            if use_global_default_world:
                # Reuse default world
                if sql_backend_path_valid:
                    world.set_backend(filename=sql_backend_filename, exclusive=False, enable_thread_parallelism=True)
                else:
                    world.set_backend(exclusive=False, enable_thread_parallelism=True)
                loginfo(f"Using global default ontology world with SQL backend: {sql_backend_name}")
            else:
                # Create a new world with parallelized file parsing enabled
                if sql_backend_path_valid:
                    world = OntologyWorld(filename=sql_backend_filename, exclusive=False, enable_thread_parallelism=True)
                else:
                    world = OntologyWorld(exclusive=False, enable_thread_parallelism=True)
                loginfo(f"Created a new ontology world with SQL backend: {sql_backend_name}")
        except sqlite3.Error as e:
            logerr(f"Failed accessing the SQL backend of ontology world: {sql_backend_name}",
                          e.sqlite_errorcode, e.sqlite_errorname)
        return world

    def create_main_ontology(self) -> bool:
        """
        Load ontologies from :attr:`main_ontology_iri` to :attr:`main_ontology_world`
        If `main_ontology_iri` is a remote URL, Owlready2 first searches for a local copy of the OWL file (from `onto_path`),
        if not found, tries to download it from the Internet.

        :return: True if loading succeeds
        """
        ontology_info = self.load_ontology(self.main_ontology_iri)
        if ontology_info:
            self.main_ontology, self.main_ontology_namespace = ontology_info
            if self.main_ontology and self.main_ontology.loaded:
                self.soma = self.ontologies.get(SOMA_ONTOLOGY_NAMESPACE)
                self.dul = self.ontologies.get(DUL_ONTOLOGY_NAMESPACE)
        return ontology_info is not None

    def load_ontology(self, ontology_iri: str) -> Optional[Tuple[Ontology, Namespace]]:
        """
        Load an ontology from an IRI

        :param ontology_iri: An ontology IRI
        :return: A tuple including an ontology instance & its namespace
        """
        if not ontology_iri:
            logerr("Ontology IRI is empty")
            return None

        is_local_ontology_iri = not (ontology_iri.startswith("http:") or ontology_iri.startswith("https:"))

        # If `ontology_iri` is a local path
        if is_local_ontology_iri and not Path(ontology_iri).exists():
            # -> Create an empty ontology file if not existing
            ontology_path = ontology_iri if os.path.isabs(ontology_iri) else (
                os.path.join(self.get_main_ontology_dir(), ontology_iri))
            with open(ontology_path, 'w'):
                pass

        # Load ontology from `ontology_iri`
        ontology = None
        try:
            if self.main_ontology_world:
                ontology = self.main_ontology_world.get_ontology(ontology_iri).load(reload_if_newer=True)
            else:
                ontology = get_ontology(ontology_iri).load(reload_if_newer=True)
        except OwlReadyOntologyParsingError as error:
            logwarn(error)
            if is_local_ontology_iri:
                logerr(f"Main ontology failed being loaded from {ontology_iri}")
            else:
                logwarn(f"Main ontology failed being downloaded from the remote {ontology_iri}")
            return None

        # Browse loaded `ontology`, fetching sub-ontologies
        ontology_namespace = get_namespace(ontology_iri)
        if ontology and ontology.loaded:
            loginfo(
                f'Ontology [{ontology.base_iri}]\'s name: {ontology.name} has been loaded')
            loginfo(f'- main namespace: {ontology_namespace.name}')
            loginfo(f'- loaded ontologies:')

            def fetch_ontology(ontology__):
                self.ontologies[ontology__.name] = ontology__
                loginfo(ontology__.base_iri)

            self.browse_ontologies(ontology, condition=None, func=lambda ontology__: fetch_ontology(ontology__))
        else:
            logerr(f"Ontology [{ontology.base_iri}]\'s name: {ontology.name} failed being loaded")
        return ontology, ontology_namespace

    def initialized(self) -> bool:
        """
        Check if the main ontology has been loaded

        :return: True if loaded, otherwise False
        """
        return hasattr(self, "main_ontology") and self.main_ontology and self.main_ontology.loaded

    @staticmethod
    def browse_ontologies(ontology: Ontology,
                          condition: Optional[Callable] = None, func: Optional[Callable] = None, **kwargs) -> None:
        """
        Browse the loaded ontologies (including the main and imported ones), doing operations based on a condition.

        :param ontology: An ontology instance as the result of ontology loading
        :param condition: a Callable condition that if not None needs to be passed before doing operations, otherwise just always carry the operations
        :param func: a Callable specifying the operations to perform on all the loaded ontologies if condition is None, otherwise only the first ontology which meets the condition
        """
        if ontology is None:
            logerr(f"Ontology {ontology=} is None!")
            return
        elif not ontology.loaded:
            logerr(f"Ontology {ontology} was not loaded!")
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

    def save(self, target_filename: Optional[str] = None, overwrite: bool = False) -> bool:
        """
        Save :attr:`main_ontology` to a file on disk, also caching :attr:`main_ontology_world` to a sqlite3 file

        :param target_filename: full name path of a file which the ontologies are saved into.
        :param overwrite: overwrite an existing file if it exists. If empty, they are saved to the same original OWL file from which the main ontology was loaded, or a file at the same folder with ontology search path specified at constructor if it was loaded from a remote IRI.
        :return: True if the ontology was successfully saved, False otherwise
        """

        # Save ontologies to OWL
        is_current_ontology_local = os.path.isfile(self.main_ontology_iri)
        current_ontology_filename = self.main_ontology_iri if is_current_ontology_local \
            else f"{self.get_main_ontology_dir()}/{Path(self.main_ontology_iri).name}"
        save_to_same_file = is_current_ontology_local and (target_filename == current_ontology_filename)
        if save_to_same_file and not overwrite:
            logerr(
                f"Ontologies cannot be saved to the originally loaded [{target_filename}] if not by overwriting")
            return False
        else:
            save_filename = target_filename if target_filename else current_ontology_filename
            self.main_ontology.save(save_filename)
            if save_to_same_file and overwrite:
                logwarn(f"Main ontology {self.main_ontology.name} has been overwritten to {save_filename}")
            else:
                loginfo(f"Main ontology {self.main_ontology.name} has been saved to {save_filename}")

            # Commit the whole graph data of the current ontology world, saving it into SQLite3, to be reused the next time
            # the ontologies are loaded
            main_ontology_sql_filename = self.main_ontology_world.filename
            self.main_ontology_world.save()
            if os.path.isfile(main_ontology_sql_filename):
                loginfo(
                    f"Main ontology world for {self.main_ontology.name} has been cached and saved to SQL: {main_ontology_sql_filename}")
            #else: it could be using memory cache as SQL backend
            return True

    def create_ontology_concept_class(self, class_name: str,
                                      ontology_parent_concept_class: Optional[Thing] = None,
                                      ontology: Optional[Ontology] = None) \
            -> Optional[Type[Thing]]:
        """
        Create a new concept class in a given ontology

        :param class_name: A given name to the new class
        :param ontology_parent_concept_class: An optional parent ontology class of the new class
        :param ontology: an owlready2.Ontology in which the concept class is created
        :return: The created ontology class
        """
        ontology = ontology if ontology else self.main_ontology
        ontology_concept_class = self.get_ontology_class_by_ontology(ontology, class_name)
        if ontology_concept_class:
            return ontology_concept_class

        if getattr(ontology, class_name, None):
            logerr(f"Ontology concept class {ontology.name}.{class_name} already exists")
            return None

        with ontology:
            return types.new_class(class_name, (Thing, ontology_parent_concept_class,)
            if inspect.isclass(ontology_parent_concept_class) else (Thing,))

    def create_ontology_property_class(self, class_name: str,
                                       ontology_parent_property_class: Optional[Type[Property]] = None,
                                       ontology: Optional[Ontology] = None) \
            -> Optional[Type[Property]]:
        """
        Create a new property class in a given ontology

        :param class_name: A given name to the new class
        :param ontology_parent_property_class: An optional parent ontology property class of the new class
        :param ontology: an owlready2.Ontology in which the concept class is created
        :return: The created ontology class
        """
        ontology = ontology if ontology else self.main_ontology
        parent_class = ontology_parent_property_class if (ontology_parent_property_class and
                                                          issubclass(ontology_parent_property_class,
                                                                     Property)) \
            else None

        if getattr(ontology, class_name, None):
            logerr(f"Ontology property class {ontology.name}.{class_name} already exists")
            return None

        with ontology:
            return types.new_class(class_name, (parent_class,) if parent_class else (Property,))

    def get_ontology_classes_by_condition(self, condition: Callable, first_match_only=False, **kwargs) \
            -> List[Type[Thing]]:
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
            loginfo(f"No class with {kwargs} is found in the ontology {self.main_ontology}")
        return out_classes

    @staticmethod
    def get_ontology_class_by_ontology(ontology: Ontology, class_name: str) -> Optional[Type[Thing]]:
        """
        Get an ontology class if it exists in a given ontology

        :param ontology: an ontology instance
        :param class_name: name of the searched-for ontology class
        :return: The ontology class if it exists under the namespace of the given ontology, None otherwise
        """
        return getattr(ontology, class_name, None) if ontology else None

    def get_ontology_class(self, class_name: str) -> Optional[Type[Thing]]:
        """
        Get an ontology class by name

        :param class_name: name of the searched-for ontology class
        :return: The ontology class of the given name if existing else None
        """

        def is_matching_class_name(ontology_class: Type[Thing], ontology_class_name: str):
            return ontology_class.name == ontology_class_name

        found_classes = self.get_ontology_classes_by_condition(condition=is_matching_class_name,
                                                               ontology_class_name=class_name,
                                                               first_match_only=True)
        return found_classes[0] if len(found_classes) > 0 else None

    def get_ontology_classes_by_namespace(self, ontology_namespace: str) -> List[Type[Thing]]:
        """
        Get all ontologies classes by namespace

        :param ontology_namespace: namespace of the searched-for ontology classes
        :return: A list of the ontology classes under the given namespace
        """

        def is_matching_ontology_namespace(ontology_class: Type[Thing], ontology_namespace_: str):
            return ontology_class.namespace.name == ontology_namespace_

        return self.get_ontology_classes_by_condition(condition=is_matching_ontology_namespace,
                                                      ontology_namespace_=ontology_namespace)

    def get_ontology_classes_by_subname(self, class_subname: str) -> List[Type[Thing]]:
        """
        Get all ontologies classes by subname

        :param class_subname: a string as part of the full names of the searched-for ontology classes
        :return: A list of the ontology classes of which the name contains the given subname
        """

        def is_matching_class_subname(ontology_class: Type[Thing], ontology_class_subname: str):
            return ontology_class_subname.lower() in ontology_class.name.lower()

        return self.get_ontology_classes_by_condition(condition=is_matching_class_subname,
                                                      ontology_class_subname=class_subname)

    def get_ontology_descendant_classes(self, ancestor_class: Type[Thing], class_subname: str = "") \
            -> List[Type[Thing]]:
        """
        Get ontology descendant classes of an ancestor class given descendant class subname

        :param class_subname: a string as part of the ancestor class full name
        :return: A list of the ontology descendant classes
        """
        return [ontology_class for ontology_class in self.main_ontology.classes()
                if (class_subname.lower() in ontology_class.name.lower()) and
                (ancestor_class in ontology_class.ancestors())]

    def get_ontology_general_class_axioms(self, ontology: Optional[Ontology] = None) -> List[GeneralClassAxiom]:
        """
        Get general class axioms of an ontology
        Ref: https://owlready2.readthedocs.io/en/latest/general_class_axioms.html

        :param ontology: an ontology instance
        :return: A list of ontology axioms in the ontology
        """
        ontology = ontology if ontology else self.main_ontology
        return list(ontology.general_class_axioms())

    def create_ontology_triple_classes(self, subject_class_name: str, object_class_name: str,
                                       predicate_class_name: str, inverse_predicate_class_name: Optional[str] = None,
                                       predicate_python_attribute_name: Optional[str] = None,
                                       inverse_predicate_python_attribute_name: Optional[str] = None,
                                       ontology_subject_parent_class: Optional[Type[Thing]] = None,
                                       ontology_object_parent_class: Optional[Type[Union[Thing, object]]] = None,
                                       ontology_property_parent_class: Type[Property] = ObjectProperty,
                                       ontology_inverse_property_parent_class: Type[Property] = ObjectProperty,
                                       ontology: Optional[Ontology] = None) -> bool:
        """
        Dynamically create ontology triple classes under same namespace with the main ontology,
        as known as {subject, predicate, object}, with the relations among them

        :param subject_class_name: name of the subject class
        :param object_class_name: name of the object class
        :param predicate_class_name: name of predicate class, also used as a Python attribute of the subject class to query object instances
        :param predicate_python_attribute_name: python attribute name designated for the predicate instance
        :param inverse_predicate_class_name: name of inverse predicate
        :param inverse_predicate_python_attribute_name: python attribute name designated for the inverse predicate instance
        :param ontology_subject_parent_class: a parent class of the subject class
        :param ontology_object_parent_class: a parent class of the object class
        :param ontology_property_parent_class: a parent ontology property class, default: owlready2.ObjectProperty
        :param ontology_inverse_property_parent_class: a parent ontology inverse property class, default: owlready2.ObjectProperty
        :param ontology: an owlready2.Ontology in which triples are created
        :return: True if the ontology triple classes are created successfully
        """

        if not predicate_python_attribute_name:
            predicate_python_attribute_name = predicate_class_name
        if not inverse_predicate_python_attribute_name:
            inverse_predicate_python_attribute_name = inverse_predicate_class_name
        ontology = ontology if ontology else self.main_ontology

        # This context manager ensures all classes created here-in share the same namepsace with `self.main_ontology`
        with ontology:
            # Subject
            ontology_subject_class = self.create_ontology_concept_class(subject_class_name,
                                                                        ontology_subject_parent_class,
                                                                        ontology=ontology)
            if not ontology_subject_class:
                logerr(f"{ontology.name}: Failed creating ontology subject class named {subject_class_name}")
                return False

            # Object
            if not ontology_object_parent_class or issubclass(ontology_object_parent_class, Thing):
                ontology_object_class = self.create_ontology_concept_class(object_class_name,
                                                                           ontology_object_parent_class,
                                                                           ontology=ontology) \
                    if (object_class_name != subject_class_name) else ontology_subject_class
            else:
                ontology_object_class = ontology_object_parent_class

            if not ontology_object_class:
                logerr(f"{ontology.name}: Failed creating ontology object class named {object_class_name}")
                return False

            # Predicate
            ontology_predicate_class = self.create_ontology_property_class(predicate_class_name,
                                                                           ontology_property_parent_class,
                                                                           ontology=ontology)
            if not ontology_predicate_class:
                logerr(f"{ontology.name}: Failed creating ontology predicate class named {predicate_class_name}")
                return False
            ontology_predicate_class.domain = [ontology_subject_class]
            ontology_predicate_class.range = [ontology_object_class]
            ontology_predicate_class.python_name = predicate_python_attribute_name

            # Inverse Predicate
            if inverse_predicate_class_name:
                ontology_inverse_predicate_class = self.create_ontology_property_class(inverse_predicate_class_name,
                                                                                       ontology_inverse_property_parent_class,
                                                                                       ontology=ontology)
                if not ontology_inverse_predicate_class:
                    logerr(
                        f"{ontology.name}: Failed creating ontology inverse-predicate class named {inverse_predicate_class_name}")
                    return False
                ontology_inverse_predicate_class.inverse_property = ontology_predicate_class
                ontology_inverse_predicate_class.domain = [ontology_object_class]
                ontology_inverse_predicate_class.range = [ontology_subject_class]
                ontology_inverse_predicate_class.python_name = inverse_predicate_python_attribute_name
        return True

    def create_ontology_linked_designator(self, designator_class: Type[DesignatorDescription],
                                          ontology_concept_name: str,
                                          object_name: str,
                                          ontology_parent_class: Optional[Type[Thing]] = None) \
            -> Optional[DesignatorDescription]:
        """
        Create a designator linked to a given ontology concept

        :param designator_class: A given designator class
        :param ontology_concept_name: Ontology concept name
        :param object_name: Name of object in case of the designator to be created is an Object Designator
        :param ontology_parent_class: Parent ontology class from which the class of designator inherits
        :return: A designator associated with an ontology concept
        """
        ontology_concept_class = self.create_ontology_concept_class(ontology_concept_name, ontology_parent_class)
        return self.create_ontology_linked_designator_by_concept(designator_class=designator_class,
                                                                 ontology_concept_class=ontology_concept_class,
                                                                 object_name=object_name)

    def create_ontology_linked_designator_by_concept(self, designator_class: Type[DesignatorDescription],
                                                     ontology_concept_class: Type[Thing],
                                                     object_name: str) \
            -> Optional[DesignatorDescription]:
        """
        Create a designator that belongs to a given ontology concept class

        :param designator_class: A given designator class
        :param ontology_concept_class: An ontology concept class with which the output designator is associated
        :param object_name: Name of object in case of the designator to be created is an Object Designator
        :return: An object designator associated with the given ontology concept class if created successfully (not already exists), None otherwise
        """
        ontology_concept_name = f'{object_name}_concept'
        if len(OntologyConceptHolderStore().get_designators_of_ontology_concept(ontology_concept_name)) > 0:
            logerr(
                f"A designator named [{object_name}] is already created for ontology concept [{ontology_concept_name}]")
            return None

        # Create a designator of `designator_class`
        is_object_designator = issubclass(designator_class, ObjectDesignatorDescription)
        if is_object_designator:
            if not object_name:
                logerr(
                    f"An empty object name was given as creating its Object designator for ontology concept class [{ontology_concept_class.name}]")
                return None
            designator = designator_class(names=[object_name])
        else:
            designator = designator_class()

        # Link designator with an ontology concept of `ontology_concept_class`
        ontology_concept_holder = OntologyConceptHolderStore().get_ontology_concept_holder_by_name(
            ontology_concept_name)
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
                logerr(f"Ontology concept [{subject_concept.name}] has no predicate named [{predicate_name}]")
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
              for object_concept in getattr(subject_concept_holder.ontology_concept, predicate_name, [])]))

    def create_ontology_object_designator_from_type(self, object_type: ObjectType,
                                                    ontology_concept_class: Type[Thing]) \
            -> Optional[ObjectDesignatorDescription]:
        """
        Create an object designator associated with an ontology concept class from a given object type

        :param object_type: An enumerated type of object
        :param ontology_concept_class: An ontology concept class
        :return: An object designator if created successfully (if not already existing), otherwise None
        """
        object_type_name = object_type.name.lower()
        object_designator = \
            self.create_ontology_linked_designator_by_concept(designator_class=ObjectDesignatorDescription,
                                                              ontology_concept_class=ontology_concept_class,
                                                              object_name=object_type_name)
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

    def create_rule_reflexivity(self, ontology_concept_class_name: str,
                                predicate_name: str,
                                ontology: Optional[Ontology] = None) -> Imp:
        """
        Create the rule of reflexivity for a given ontology concept class.
        Same effect is obtained by creating a dynamic ontology predicate class, subclassing owlready2.ReflexiveProperty.
        Ref: https://en.wikipedia.org/wiki/Reflexive_relation

        :param ontology_concept_class_name: Name of the ontology concept class having the relation defined
        :param predicate_name: Name of the ontology predicate signifying the reflexive relation
        :param ontology: The ontology for which the rule is created
        :return: Rule of transitivity
        """
        ontology = ontology if ontology else self.main_ontology
        with ontology:
            rule = Imp()
            rule.set_as_rule(f"""{ontology_concept_class_name}(?a)
                                 -> {predicate_name}(?a, ?a)""")
            return rule

    def create_rule_symmetry(self, ontology_concept_class_name: str,
                             predicate_name: str,
                             ontology: Optional[Ontology] = None) -> Imp:
        """
        Create the rule of transitivity for a given ontology concept class.
        Same effect is obtained by creating a dynamic ontology predicate class, subclassing owlready2.SymmetricProperty.
        Ref: https://en.wikipedia.org/wiki/Symmetric_relation

        :param ontology_concept_class_name: Name of the ontology concept class having the relation defined
        :param predicate_name: Name of the ontology predicate signifying the symmetric relation
        :param ontology: The ontology for which the rule is created
        :return: Rule of symmetry
        """
        ontology = ontology if ontology else self.main_ontology
        with ontology:
            rule = Imp()
            rule.set_as_rule(f"""{ontology_concept_class_name}(?a), {ontology_concept_class_name}(?b),
                                 {predicate_name}(?a, ?b)
                                 -> {predicate_name}(?b, ?a)""")
            return rule

    def create_rule_transitivity(self, ontology_concept_class_name: str,
                                 predicate_name: str,
                                 ontology: Optional[Ontology] = None) -> Imp:
        """
        Create the rule of transitivity for a given ontology concept class.
        Same effect is obtained by creating a dynamic ontology predicate class, subclassing owlready2.TransitiveProperty.
        Ref:
        - https://en.wikipedia.org/wiki/Transitive_relation
        - https://owlready2.readthedocs.io/en/latest/properties.html#obtaining-indirect-relations-considering-subproperty-transitivity-etc

        :param ontology_concept_class_name: Name of the ontology concept class having the relation defined
        :param predicate_name: Name of the ontology predicate signifying the transitive relation
        :param ontology: The ontology for which the rule is created
        :return: Rule of transitivity
        """
        ontology = ontology if ontology else self.main_ontology
        with ontology:
            rule = Imp()
            rule.set_as_rule(
                f"""{ontology_concept_class_name}(?a), {ontology_concept_class_name}(?b), {ontology_concept_class_name}(?c),
                                 {predicate_name}(?a, ?b),
                                 {predicate_name}(?b, ?c)
                                 -> {predicate_name}(?a, ?c)""")
            return rule

    def reason(self, world: OntologyWorld = None, use_pellet_reasoner: bool = True) -> bool:
        """
        Run the reasoning on a given ontology world or :attr:`main_ontology_world` with Pellet or HermiT reasoner,
        the two currently supported by owlready2
        - By default, the reasoning works on `owlready2.default_world`
        - The reasoning also automatically save ontologies (to either in-memory cache or a temporary sqlite3 file)
        Ref:
        - https://owlready2.readthedocs.io/en/latest/reasoning.html
        - https://owlready2.readthedocs.io/en/latest/rule.html
        - https://www.researchgate.net/publication/200758993_Benchmarking_OWL_reasoners
        - https://www.researchgate.net/publication/345959058_OWL2Bench_A_Benchmark_for_OWL_2_Reasoners

        :param world: An owlready2.World to reason about. If None, use :attr:`main_ontology_world`
        :param use_pellet_reasoner: Use Pellet reasoner, otherwise HermiT
        :return: True if the reasoning was successful, otherwise False
        """
        reasoner_name = None
        reasoning_world = world if world else self.main_ontology_world
        try:
            if use_pellet_reasoner:
                reasoner_name = "Pellet"
                sync_reasoner_pellet(x=reasoning_world, infer_property_values=True,
                                     infer_data_property_values=True)
            else:
                reasoner_name = "HermiT"
                sync_reasoner_hermit(x=reasoning_world, infer_property_values=True)
        except OwlReadyError as error:
            logerr(f"{reasoner_name} reasoning failed: {error}")
            return False
        loginfo(f"{reasoner_name} reasoning finishes!")
        return True
