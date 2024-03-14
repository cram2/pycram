import sys, inspect
from abc import ABCMeta
from pathlib import Path
from typing import List

from pycram.designator import ObjectDesignatorDescription
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.helper import Singleton

from owlready2 import *

class OntologyOWL(object, metaclass=Singleton):
    """
    Singleton class as the adapter accessing data of an OWL ontology, largely based on owlready2.
    """
    onto = None
    onto_world = None
    onto_filename = None
    onto_namespace = None

    @classmethod
    def print_ontology_class(cls, onto_class):
        """
        Print information (ancestors, super classes, subclasses, properties, etc.) of an ontology class
        """
        print(onto_class, type(onto_class))
        print('Super classes: ', onto_class.is_a)
        print('Ancestors: ', onto_class.ancestors())
        print('Subclasses: ', list(onto_class.subclasses()))
        print('Properties: ', list(onto_class.get_class_properties()))

    def __init__(self, onto_filename: str, onto_search_path=f"{Path.home()}/ontologies"):
        """
        Create the singleton object of OntologyOWL class

        :param onto_filename: full name path of to be loaded ontology file
        :param onto_search_path: directory path from which a possibly existing ontology is searched. This is appeneded
        to `onto_path`, a global variable by owlready2 containing a list of directories for searching local copies of
        ontologies (similarly to python `sys.path` for modules/packages).
        """
        Path(onto_search_path).mkdir(parents=True, exist_ok=True)
        onto_path.append(onto_search_path)
        onto_name = Path(onto_filename).stem

        # Create an ontology world with parallelized file parsing enabled
        OntologyOWL.onto_world = World(filename=f"{onto_search_path}/{onto_name}.sqlite3", exclusive=False,
                                       enable_thread_parallelism=True)

        onto = OntologyOWL.onto_world.get_ontology(onto_filename).load()
        if onto.loaded:
            print(f'Ontology [{onto.base_iri}]\'s name: {onto.name} has been loaded')
            OntologyOWL.onto = onto
            OntologyOWL.onto_namespace = get_namespace(onto.base_iri).name
            print(f'Ontology namespace: {OntologyOWL.onto_namespace}')
            OntologyOWL.onto_filename = onto_filename
        else:
            assert False, f'Ontology [{onto.base_iri}]\'s name: {onto.name} failed being loaded'

    @classmethod
    def save(cls):
        """
        Save the current ontology to disk
        """
        cls.onto.save(file=cls.onto_filename)
        cls.onto_world.save()

    @classmethod
    def create_ontology_class(cls, parent_onto_class, class_name):
        """
        Create a new class in ontology

        :param parent_onto_class: a parent ontology class for the new class
        :class_name: a given name to the new class
        """
        assert issubclass(parent_onto_class, Thing)
        return types.new_class(class_name, (parent_onto_class,))

    @classmethod
    def get_ontology_class(cls, class_name):
        """
        Get an ontology class

        :param class_name: name of the searched-for ontology class
        """
        for onto_class in cls.onto.classes():
            if onto_class.name == class_name:
                print(onto_class, type(onto_class))
                print('Super classes: ', onto_class.is_a)
                print('Ancestors: ', onto_class.ancestors())
                print('Subclasses: ', list(onto_class.subclasses()))
                print('Properties: ', list(onto_class.get_class_properties()))
                return onto_class
        return None

    @classmethod
    def get_ontology_classes_by_namespace(cls, namespace: str):
        """
        Get all ontologies classes by namespace

        :param namespace: namespace of the searched-for ontology classes
        """
        return [onto_class for onto_class in cls.onto.classes() if onto_class.namespace.name == namespace]

    @classmethod
    def get_ontology_classes_by_subname(cls, class_subname: str):
        """
        Get all ontologies classes by subname

        :param class_subname: a string as part of the full names of the searched-for ontology classes
        """
        return [onto_class for onto_class in cls.onto.classes() if class_subname.lower() in onto_class.name.lower()]

    @classmethod
    def get_ontology_descendants(cls, ancestor_class, class_subname: str):
        """
        Get ontology descendant classes of an ancestor_class given descendant class subname

        :param class_subname: a string as part of the full names of the given ancestor class
        """
        descendants = []
        for onto_class in cls.onto.classes():
            if (class_subname.lower() in onto_class.name.lower()) and (ancestor_class in onto_class.ancestors()):
                descendants.append(onto_class)
        return descendants


if __name__ == "__main__":
    OntologyOWL("http://www.ease-crc.org/ont/SOMA.owl")

    print(OntologyOWL.get_ontology_classes_by_namespace("SOMA"))
    print(OntologyOWL.get_ontology_classes_by_subname('Container'))
    print(f'{OntologyOWL.onto.name}.Container: ', OntologyOWL.onto.Container)
    print(OntologyOWL.get_ontology_descendants(OntologyOWL.onto.Container, 'Container'))

    print(OntologyOWL.get_ontology_classes_by_subname('Navigating'))
    print(OntologyOWL.get_ontology_class('Navigating'))

    # enum -> Onto class, the gate to ontology knowledge related to ObjectType
    print(OntologyOWL.get_ontology_classes_by_subname('Container'))
    parent_onto_cls_name = 'Container'
    child_onto_cls_name = 'CustomContainer'
    ParentOntoClass = OntologyOWL.get_ontology_class(parent_onto_cls_name)
    if ParentOntoClass:
        ChildOntoClass = OntologyOWL.create_ontology_class(ParentOntoClass, child_onto_cls_name)
        print(ChildOntoClass)

        # PyCram obj -> Onto concept
        onto_concept = ChildOntoClass('child_onto')
        print('\nonto_concept:', onto_concept, '- type: ', type(onto_concept))
        pycram_obj = ObjectDesignatorDescription(names=["obj"])
        pycram_obj.pose = lambda: Pose
        pycram_obj.onto_concept = onto_concept
    else:
        print(f"No {parent_onto_cls_name} found in {OntologyOWL.onto}")

