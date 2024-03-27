import inspect
from pathlib import Path
from typing import Optional, List, Type, Callable

from owlready2 import *

from pycram.enums import ObjectType
from pycram.helper import Singleton
from pycram.designator import DesignatorDescription, ObjectDesignatorDescription

SOMA_HOME_ONTOLOGY = "http://www.ease-crc.org/ont/SOMA-HOME.owl"
SOMA_ONTOLOGY = "http://www.ease-crc.org/ont/SOMA.owl"

class OntologyManager(object, metaclass=Singleton):
    """
    Singleton class as the adapter accessing data of an OWL ontology, largely based on owlready2.
    """

    """
    The ontology instance as the result of an ontology loading operation
    """
    onto = None
    """
    The SOMA ontology instance, referencing :attr:`onto` in case of ontology loading from SOMA.owl
    Ref: http://www.ease-crc.org/ont/SOMA.owl
    """
    soma = None
    """
    The DUL ontology instance, referencing :attr:`onto` in case of ontology loading from DUL.owl
    Ref: http://www.ease-crc.org/ont/DUL.owl
    """
    dul = None
    """
    Ontology world, place holder of triples stored by owlready2. Ref: https://owlready2.readthedocs.io/en/latest/world.html 
    """
    onto_world = None
    """
    Full name path of the file from which the ontology is loaded
    """
    onto_filename = None
    """
    Namespace of the loaded ontology
    """
    onto_namespace = None

    @classmethod
    def print_ontology_class(cls, onto_class):
        """
        Print information (ancestors, super classes, subclasses, properties, etc.) of an ontology class
        """
        if onto_class is None:
            return
        print("-------------------")
        print(onto_class, type(onto_class))
        print('Super classes: ', onto_class.is_a)
        print('Ancestors: ', onto_class.ancestors())
        print('Subclasses: ', list(onto_class.subclasses()))
        print('Properties: ', list(onto_class.get_class_properties()))
        print("Instances: ", list(onto_class.instances()))
        print("Direct Instances: ", list(onto_class.direct_instances()))
        print("Inverse Restrictions: ", list(onto_class.inverse_restrictions()))

    @classmethod
    def browse_ontologies(cls, condition: Callable, func: Optional[Callable] = None, **kwargs):
        if cls.onto is None:
            assert False, "Main ontology has not been loaded!"

        do_func = func is not None
        if condition is None:
            if do_func:
                func(cls.onto, **kwargs)
                for sub_onto in cls.onto.get_imported_ontologies():
                    func(sub_onto, **kwargs)
        elif condition(cls.onto, **kwargs):
            if do_func: func(cls.onto, **kwargs)
        else:
            for sub_onto in cls.onto.get_imported_ontologies():
                if condition(sub_onto, **kwargs) and do_func:
                    func(sub_onto, **kwargs)
                    break

    def __init__(self, onto_filename: str, onto_search_path=f"{Path.home()}/ontologies"):
        """
        Create the singleton object of OntologyManager class

        :param onto_filename: full name path of to be loaded ontology file
        :param onto_search_path: directory path from which a possibly existing ontology is searched. This is appended
        to `onto_path`, a global variable by owlready2 containing a list of directories for searching local copies of
        ontologies (similarly to python `sys.path` for modules/packages).
        """
        Path(onto_search_path).mkdir(parents=True, exist_ok=True)
        onto_path.append(onto_search_path)
        onto_name = Path(onto_filename).stem
        OntologyManager.onto_filename = onto_filename

        # Create an ontology world with parallelized file parsing enabled
        OntologyManager.onto_world = World(filename=f"{onto_search_path}/{onto_name}.sqlite3", exclusive=False,
                                           enable_thread_parallelism=True)

        onto_ = OntologyManager.onto_world.get_ontology(onto_filename).load()
        if onto_.loaded:
            print(f'Main Ontology [{onto_.base_iri}]\'s name: {onto_.name} has been loaded')
            OntologyManager.onto = onto_
            OntologyManager.onto_namespace = get_namespace(onto_.base_iri).name
            print(f'Main Ontology namespace: {OntologyManager.onto_namespace}')

            print(f'Loaded ontologies:')
            OntologyManager.browse_ontologies(condition=None, func=lambda ontology: print(ontology.base_iri))

            # Search for SOMA & DUL from imported sub-ontologies
            def is_matching_onto(ontology, onto_name):
                return get_namespace(ontology.base_iri).name.lower() == onto_name.lower()

            def set_soma(ontology, onto_name):
                OntologyManager.soma = ontology

            def set_dul(ontology, onto_name):
                OntologyManager.dul = ontology

            OntologyManager.browse_ontologies(condition=is_matching_onto, func=set_soma, onto_name="SOMA")
            OntologyManager.browse_ontologies(condition=is_matching_onto, func=set_dul, onto_name="DUL")
        else:
            assert False, f'Ontology [{onto_.base_iri}]\'s name: {onto_.name} failed being loaded'

        with onto_:
            class OntologyConcept(Thing):
                """
                A default ontology concept class that inherits from owlready2.Thing with a list of designators as its attribute
                """
                namespace = onto_

                def __init__(self, name: str):
                    """
                    Create a new ontology concept

                    :param name: concept name
                    """
                    super().__init__(name)
                    self.designators = []
                    self.resolve = None

                def get_default_designator(self) -> DesignatorDescription:
                    """
                    Return the first element of designators if there is, else None
                    """
                    return self.designators[0] if len(self.designators) > 0 else None

    @classmethod
    def save(cls):
        """
        Save the current ontology to disk
        """
        cls.onto.save(file=cls.onto_filename)
        cls.onto_world.save()

    @classmethod
    def create_ontology_concept_class(cls, class_name: str,
                                      onto_parent_concept_class: Optional[owlready2.Thing] = None)\
            -> Type[owlready2.Thing]:
        """
        Create a new concept class in ontology

        :param class_name: A given name to the new class
        :param onto_parent_concept_class: An optional parent ontology class of the new class
        :return: The created ontology class
        """
        return types.new_class(class_name, (cls.onto.OntologyConcept, onto_parent_concept_class,)
                               if inspect.isclass(onto_parent_concept_class) else (cls.onto.OntologyConcept,))

    @classmethod
    def create_ontology_property_class(cls, class_name: str,
                                       onto_parent_property_class: Optional[owlready2.Property] = None)\
            -> Type[owlready2.Property]:
        """
        Create a new property class in ontology

        :param class_name: A given name to the new class
        :param onto_parent_property_class: An optional parent ontology property class of the new class
        :return: The created ontology class
        """
        if onto_parent_property_class:
            assert issubclass(onto_parent_property_class, owlready2.Property), \
                f"{onto_parent_property_class} must be a subclass of one implementing owlready2.Property"
        return types.new_class(class_name, (onto_parent_property_class,)
                               if inspect.isclass(onto_parent_property_class) else (owlready2.ObjectProperty,))

    @classmethod
    def get_ontology_classes_by_condition(cls, condition: Callable, print_info=False, first_match_only=False, **kwargs)\
            -> List[Type[owlready2.Thing]]:
        """
        Get an ontology class by a given condition

        :param condition: condition of searching
        :param print_info: print essential information of the class (super/sub-classes, ancestors, properties, etc.)
        :param first_match_only: whether to only fetch the first class matching the given condition
        :return: The ontology class satisfying the given condition if found else None
        """
        out_classes = []
        for onto_class in list(cls.onto.classes()):
            if condition(onto_class, **kwargs):
                out_classes.append(onto_class)
                if first_match_only:
                    return out_classes

        for sub_onto in cls.onto.get_imported_ontologies():
            for sub_onto_class in list(sub_onto.classes()):
                if condition(sub_onto_class, **kwargs):
                    out_classes.append(sub_onto_class)
                    if first_match_only:
                        return out_classes

        if len(out_classes):
            if print_info:
                for out_class in out_classes: cls.print_ontology_class(out_class)
        else:
            print(f"No class with {kwargs} is found in the ontology {cls.onto}")
        return out_classes

    @classmethod
    def get_ontology_class(cls, class_name: str, print_info=False) -> Type[owlready2.Thing]:
        """
        Get an ontology class by name

        :param class_name: name of the searched-for ontology class
        :param print_info: print essential information of the class (super/sub-classes, ancestors, properties, etc.)
        :return: The ontology class of the given name if existing else None
        """

        def is_matching_class_name(onto_class: Type[owlready2.Thing], onto_class_name: str):
            return onto_class.name == onto_class_name

        found_classes = cls.get_ontology_classes_by_condition(condition=is_matching_class_name,
                                                              onto_class_name=class_name,
                                                              print_info=print_info, first_match_only=True)
        return found_classes[0] if len(found_classes) > 0 else None

    @classmethod
    def get_ontology_classes_by_namespace(cls, onto_namespace: str, print_info=False) -> List[Type[owlready2.Thing]]:
        """
        Get all ontologies classes by namespace

        :param onto_namespace: namespace of the searched-for ontology classes
        :param print_info: whether to print the found class info
        :return: A list of the ontology classes under the given namespace
        """

        def is_matching_onto_namespace(onto_class: Type[owlready2.Thing], ontology_namespace: str):
            return onto_class.namespace.name == ontology_namespace

        return cls.get_ontology_classes_by_condition(condition=is_matching_onto_namespace,
                                                     ontology_namespace=onto_namespace,
                                                     print_info=print_info)

    @classmethod
    def get_ontology_classes_by_subname(cls, class_subname: str, print_info=False) -> List[Type[owlready2.Thing]]:
        """
        Get all ontologies classes by subname

        :param class_subname: a string as part of the full names of the searched-for ontology classes
        :param print_info: whether to print the found class info
        :return: A list of the ontology classes of which the name contains the given subname
        """

        def is_matching_class_subname(onto_class: Type[owlready2.Thing], onto_class_subname: str):
            return onto_class_subname.lower() in onto_class.name.lower()

        return cls.get_ontology_classes_by_condition(condition=is_matching_class_subname,
                                                     onto_class_subname=class_subname,
                                                     print_info=print_info)

    @classmethod
    def get_ontology_descendant_classes(cls, ancestor_class: Type[owlready2.Thing], class_subname: str = "")\
            -> List[Type[owlready2.Thing]]:
        """
        Get ontology descendant classes of an ancestor class given descendant class subname

        :param class_subname: a string as part of the ancestor class full name
        :return: A list of the ontology descendant classes
        """
        return [onto_class for onto_class in cls.onto.classes()
                if (class_subname.lower() in onto_class.name.lower()) and
                (ancestor_class in onto_class.ancestors())]

    @classmethod
    def create_ontology_triple_classes(cls, subject_class_name: str, object_class_name: str,
                                       predicate_name: str, inverse_predicate_name: str,
                                       onto_subject_parent_class: Optional[Type[owlready2.Thing]] = None,
                                       onto_object_parent_class: Optional[Type[owlready2.Thing]] = None,
                                       onto_property_parent_class: Optional[Type[
                                           owlready2.Property]] = owlready2.ObjectProperty,
                                       onto_inverse_property_parent_class: Optional[Type[
                                           owlready2.Property]] = owlready2.ObjectProperty):
        """
        Dynamically create ontology triple classes under same namespace with the main ontology,
        aka subject, predicate, object, with relation among them

        :param subject_class_name: name of the subject class
        :param object_class_name: name of the object class
        :param predicate_name: name of predicate class, also used as a Python attribute of the subject class to
        query object instances
        :param inverse_predicate_name: name of inverse predicate
        :param onto_subject_parent_class: a parent class of the subject class
        :param onto_object_parent_class: a parent class of the object class
        :param onto_property_parent_class: a parent ontology property class, default: owlready2.ObjectProperty
        :param onto_inverse_property_parent_class: a parent ontology inverse property class, default: owlready2.ObjectProperty
        """

        # This context manager ensures all classes created here-in share the same namepsace with `cls.onto`
        with cls.onto:
            # Subject
            onto_subject_class = cls.create_ontology_concept_class(subject_class_name, onto_subject_parent_class)

            # Object
            onto_object_class = cls.create_ontology_concept_class(object_class_name, onto_object_parent_class)

            # Predicate
            onto_predicate_class = cls.create_ontology_property_class("OntologyPredicate",
                                                                      onto_property_parent_class)
            onto_predicate_class.domain = [onto_subject_class]
            onto_predicate_class.range = [onto_object_class]
            onto_predicate_class.python_name = predicate_name

            # Inverse Predicate
            onto_inverse_predicate = cls.create_ontology_property_class("OntologyInversePredicate",
                                                                        onto_inverse_property_parent_class)
            onto_inverse_predicate.inverse_property = onto_predicate_class
            onto_inverse_predicate.python_name = inverse_predicate_name

    @classmethod
    def create_ontology_linked_designator(cls, designator_name: str, designator_class: Type[DesignatorDescription],
                                          onto_concept_name: str, onto_parent_class: Optional[Type[owlready2.Thing]] = None)\
            -> DesignatorDescription:
        """
        Create an object designator linked to a given ontology concept

        :param designator_name: Designator name
        :param designator_class: Designator class
        :param onto_concept_name: Ontology concept name
        :param onto_parent_class: Parent ontology class from which the class of designator inherits
        :return: An object designator associated with an ontology concept
        """
        onto_concept_class = cls.create_ontology_concept_class(onto_concept_name, onto_parent_class)
        return cls.create_ontology_linked_designator_by_concept(designator_name, designator_class, onto_concept_class)

    @classmethod
    def create_ontology_linked_designator_by_concept(cls, designator_name: str,
                                                     designator_class: Type[DesignatorDescription],
                                                     onto_concept_class: Type[owlready2.Thing]) -> DesignatorDescription:
        """
        Create an object designator that belongs to a given ontology concept class

        :param designator_name: Designator name
        :param designator_class: Designator class
        :param onto_concept_class: Ontology concept class which the output designator is associated with
        :return: An object designator associated with the given ontology concept class
        """
        designator = designator_class(names=[designator_name]) if issubclass(designator_class,
                                                                             ObjectDesignatorDescription) \
                                                               else designator_class()
        desig_onto_concept = onto_concept_class(name=f'{designator_name}_concept')
        cls.set_ontology_concept_designator_connection(designator, desig_onto_concept)
        return designator

    @classmethod
    def set_ontology_concept_designator_connection(cls, designator: DesignatorDescription,
                                                   ontology_concept: owlready2.Thing):
        """
        Set two-way connection between a designator and an ontology concept

        :param designator: Designator
        :param ontology_concept: Ontology concept
        """
        designator.onto_concepts.append(ontology_concept)
        ontology_concept.designators.append(designator)

    @classmethod
    def set_ontology_relation(cls, subject_designator: DesignatorDescription,
                              object_designator: DesignatorDescription,
                              predicate_name: str):
        """
        Set ontology relation between subject and object designators

        :param subject_designator: An object designator as the ontology subject
        :param object_designator: An object designator as the ontology object
        :param predicate_name: Name of the predicate
        """
        for subject_onto_concept in subject_designator.onto_concepts:
            getattr(subject_onto_concept, predicate_name).extend(object_designator.onto_concepts)

    @classmethod
    def get_designators_by_subject_predicate(cls, subject: DesignatorDescription,
                                             predicate_name: str) -> List[DesignatorDescription]:
        """
        Get list of designators for a given subject designator and predicate

        :param subject: The subject designator
        :param predicate_name: The predicate name of the relation
        :return: List of object designators
        """
        designators = list(itertools.chain(
            *[onto_subject.designators for subject_onto_concept in subject.onto_concepts
              for onto_subject in getattr(subject_onto_concept, predicate_name)]))
        return designators

    @classmethod
    def create_ontology_object_designator_from_type(cls, object_type: ObjectType,
                                                    onto_concept_class=Type[owlready2.Thing]) -> ObjectDesignatorDescription:
        obj_type_name = object_type.name.lower()
        obj_designator = \
            cls.create_ontology_linked_designator_by_concept(obj_type_name,
                                                             ObjectDesignatorDescription,
                                                             onto_concept_class)
        obj_designator.types = [obj_type_name]
        return obj_designator

    @classmethod
    def demo_placeable_on_candidates_query(cls):
        """
        Demonstrates how to create ontology-based object designators dynamically with ontology relations among them
        """
        PLACEABLE_ON_PREDICATE_NAME = "placeable_on"
        HOLD_OBJ_PREDICATE_NAME = "hold_obj"
        cls.create_ontology_triple_classes(onto_subject_parent_class=cls.onto.Container,
                                           subject_class_name="OntologyPlaceHolderObject",
                                           onto_object_parent_class=cls.onto.PhysicalObject,
                                           object_class_name="OntologyHandheldObject",
                                           predicate_name=PLACEABLE_ON_PREDICATE_NAME,
                                           inverse_predicate_name=HOLD_OBJ_PREDICATE_NAME,
                                           onto_property_parent_class=cls.soma.affordsBearer,
                                           onto_inverse_property_parent_class=cls.soma.isBearerAffordedBy)

        def create_ontology_handheld_object(obj_name: str, onto_parent_class: Type[owlready2.Thing]):
            return OntologyManager.create_ontology_linked_designator(designator_name=obj_name,
                                                                     designator_class=ObjectDesignatorDescription,
                                                                     onto_concept_name=f"Onto{obj_name}",
                                                                     onto_parent_class=cls.onto.OntologyHandheldObject)

        # Holdable Objects
        cookie_box = create_ontology_handheld_object("cookie_box", cls.onto.OntologyHandheldObject)
        egg = create_ontology_handheld_object("egg", cls.onto.OntologyHandheldObject)

        # Placeholder objects
        placeholders = [create_ontology_handheld_object(obj_name, cls.onto.OntologyPlaceHolderObject)
                        for obj_name in ['table', 'stool', 'shelf']]

        egg_tray = create_ontology_handheld_object("egg_tray", cls.onto.OntologyPlaceHolderObject)

        # Create ontology relation between [Place-holders] and [Holdable objs]
        for place_holder in placeholders:
            cls.set_ontology_relation(subject_designator=cookie_box, object_designator=place_holder,
                                      predicate_name=PLACEABLE_ON_PREDICATE_NAME)

        cls.set_ontology_relation(subject_designator=egg_tray, object_designator=egg,
                                  predicate_name=HOLD_OBJ_PREDICATE_NAME)

        # Make queries for potential designator candidates based on above-set ontology relations among them
        print(f"{cookie_box.names}'s placeholder candidates:",
              f"""{[placeholder.names for placeholder in
                    cls.get_designators_by_subject_predicate(subject=cookie_box, predicate_name=PLACEABLE_ON_PREDICATE_NAME)]}""")

        print(f"{egg.names}'s placeholder candidates:",
              f"""{[placeholder.names for placeholder in
                    cls.get_designators_by_subject_predicate(subject=egg, predicate_name=PLACEABLE_ON_PREDICATE_NAME)]}""")

        for place_holder in placeholders:
            print(f"{place_holder.names} can hold:",
                  f"""{[placeholder.names for placeholder in
                        cls.get_designators_by_subject_predicate(subject=place_holder,
                                                                 predicate_name=HOLD_OBJ_PREDICATE_NAME)]}""")

        print(f"{egg_tray.names}'s can hold:",
              f"""{[placeholder.names for placeholder in
                    cls.get_designators_by_subject_predicate(subject=egg_tray,
                                                             predicate_name=HOLD_OBJ_PREDICATE_NAME)]}""")

    @classmethod
    def demo_edible_object_types_query(cls):
        generic_edible_class = cls.create_ontology_concept_class('GenericEdible')

        edible_obj_types = [ObjectType.MILK, ObjectType.BREAKFAST_CEREAL]
        for object_type in ObjectType:
            if object_type in edible_obj_types:
                # Create a designator for that edible object
                cls.create_ontology_object_designator_from_type(object_type, generic_edible_class)

        print(f"{generic_edible_class.name} object types:")
        for edible_onto_concept in generic_edible_class.instances():
            print(edible_onto_concept, [des.types for des in edible_onto_concept.designators])


if __name__ == "__main__":
    # Initialize ontologies
    OntologyManager(SOMA_HOME_ONTOLOGY)
    # Main ontology
    onto = OntologyManager.onto

    # Imported ontologies
    soma = OntologyManager.soma
    dul = OntologyManager.dul

    #print(OntologyManager.get_ontology_classes_by_namespace('SOMA'))
    print(OntologyManager.get_ontology_classes_by_subname('Container'))
    print(f'Subclasses of {soma.name}.DesignedContainer: ',
          OntologyManager.get_ontology_descendant_classes(soma.DesignedContainer))

    OntologyManager.demo_placeable_on_candidates_query()
    OntologyManager.demo_edible_object_types_query()
