import collections

import networkx
import networkx as nx
import owlready2
from graph_tool.topology import topological_sort
from owlready2 import ThingClass, Thing, ObjectProperty, PropertyClass
from owlready2.base import *
from owlready2.class_construct import Restriction, SOME, ONLY, VALUE, HAS_SELF, _restriction_type_2_label
import tqdm
from typing_extensions import List, Any


class Parser:
    """
    A class that parses all definitions from an ontology into python files that are owlready2 compatible.

    TODO: Labels metadata and SWRL is not parsed
    """

    ontology: owlready2.Ontology
    """
    The ontology to parse.
    """

    path: str
    """
    The path to write the files of the parsed ontology into.
    """

    base_file_name: str = "base"
    """
    The file name where the base classes are written to.
    """

    classes_file_name: str = "classes"
    """
    The file name where all elements from ontology.classes() are written to.
    """

    object_properties_file_name: str = "object_properties"
    """
    The file name where all elements from ontology.properties() are written to.
    """

    data_properties_file_name: str = "data_properties"

    restrictions_file_name: str = "restrictions"

    individuals_file_name: str = "individuals"

    file_extension: str = ".py"

    indentation = 4
    """
    The indentation to use for the python file.
    """

    current_file: Any

    def __init__(self, ontology: owlready2.Ontology, path: str):
        self.ontology = ontology
        self.path = path
        render_func_without_namespace = lambda entity: entity.name
        owlready2.set_render_func(render_func_without_namespace)


    def parse(self):
        """
        Parses the ontology into a python file.
        """
        self.create_base()
        self.create_classes()
        self.create_object_properties()
        self.create_data_properties()
        self.create_individuals()
        self.create_restrictions()
        # create swrl rules
        self.create_init()

    def create_init(self):
        self.current_file = open(f"{os.path.join(self.path, '__init__')}{self.file_extension}", "w")
        self.create_import_from_classes()
        self.create_import_from_properties()
        self.import_individuals()
        self.current_file.close()

    def create_base(self):
        self.current_file = open(f"{os.path.join(self.path, self.base_file_name)}{self.file_extension}", "w")
        self.create_base_imports()
        self.current_file.write("\n" * 2)
        self.current_file.write("ontology_file = tempfile.NamedTemporaryFile()\n")
        self.current_file.write('ontology = owlready2.get_ontology("file://" + ontology_file.name).load()\n')
        self.current_file.write("\n" * 2)
        self.create_base_class()
        self.create_base_property()
        self.current_file.close()

    def create_classes(self):
        self.current_file = open(f"{os.path.join(self.path, self.classes_file_name)}{self.file_extension}", "w")
        self.create_import_from_base()
        self.current_file.write("\n" * 2)
        classes = list(self.ontology.classes())
        for cls in tqdm.tqdm(classes, desc="Parsing classes"):
            self.parse_class(cls)
        self.current_file.close()

    def create_object_properties(self):
        self.current_file = open(f"{os.path.join(self.path, self.object_properties_file_name)}{self.file_extension}", "w")
        self.create_import_from_base()
        self.current_file.write("\n" * 2)
        properties = list(self.ontology.object_properties())
        for prop in tqdm.tqdm(properties, desc="Parsing object properties"):
            self.parse_property(prop)
        self.current_file.close()

    def create_data_properties(self):
        self.current_file = open(f"{os.path.join(self.path, self.data_properties_file_name)}{self.file_extension}", "w")
        self.create_import_from_base()
        self.create_import_from_classes()
        self.current_file.write("\n" * 2)
        properties = list(self.ontology.data_properties())
        for prop in tqdm.tqdm(properties, desc="Parsing data properties"):
            self.parse_property(prop)
        self.current_file.close()

    def create_restrictions(self):
        self.current_file = open(f"{os.path.join(self.path, self.restrictions_file_name)}{self.file_extension}", "w")
        self.create_import_from_classes()
        self.import_individuals()
        self.current_file.write("\n" * 2)

        elements = list(self.ontology.classes()) + list(self.ontology.properties())

        for element in tqdm.tqdm(elements, desc="Parsing restrictions"):
            self.parse_restrictions_for(element)
            self.current_file.write("\n")
        self.current_file.close()

    def parse_restrictions_for(self, element):
        if isinstance(element, ThingClass):
            self.parse_restrictions_for_class(element)
        elif isinstance(element, PropertyClass):
            self.parse_restrictions_for_property(element)

    def parse_restrictions_for_class(self, cls: ThingClass):
        # write is_a restrictions
        is_a = self.parse_elements(cls.is_a)
        if is_a:
            is_a = f"{repr(cls)}.is_a = [{is_a}]"
            self.current_file.write(is_a)
            self.current_file.write("\n")

        # write equivalent_to restrictions
        self.write_equivalent_to(cls)

    def import_individuals(self):
        self.current_file.write("from .individuals import *\n")

    def parse_restrictions_for_property(self, prop):
        #write is_a restrictions
        is_a = self.parse_elements(prop.is_a)
        if is_a:
            is_a = f"{repr(prop)}.is_a = [{is_a}]"
            self.current_file.write(is_a)
            self.current_file.write("\n")

        # write domain restrictions
        domain_string = self.parse_elements(prop.domain)
        if domain_string:
            domain_string = f"{repr(prop)}.domain = [{domain_string}]"
            self.current_file.write(domain_string)
            self.current_file.write("\n")

        # write range restrictions
        range_string = self.parse_elements(prop.range)
        if range_string:
            range_string = f"{repr(prop)}.range = [{range_string}]"
            self.current_file.write(range_string)
            self.current_file.write("\n")

    def create_individuals(self):
        self.current_file = open(f"{os.path.join(self.path, self.individuals_file_name)}{self.file_extension}", "w")
        self.create_import_from_base()
        self.create_import_from_classes()
        self.create_import_from_properties()
        self.current_file.write("\n" * 2)

        individuals = list(self.ontology.individuals())
        for individual in tqdm.tqdm(individuals, desc="Parsing individuals"):
            self.parse_individual(individual)
        self.current_file.write("\n")
        for individual in tqdm.tqdm(individuals, desc="Parsing individuals"):
            self.parse_individual_properties(individual)
            if individual.get_properties():
                self.current_file.write("\n")
        self.current_file.close()

    def parse_individual(self, individual: owlready2.Thing):
        self.current_file.write(f"{individual.name} = {repr(individual.__class__)}(namespace = ontology)")
        self.current_file.write("\n")

    def parse_individual_properties(self, individual: owlready2.Thing):
        for prop in individual.get_properties():
            self.current_file.write(f"{individual.name}.{repr(prop)} = {individual.__getattr__(repr(prop))}")
            self.current_file.write("\n")


    def create_base_imports(self):
        self.current_file.write("from owlready2 import *\n")
        self.current_file.write("import tempfile\n")

    def create_import_from_base(self):
        self.current_file.write("from .base import *\n")

    def create_import_from_classes(self):
        self.current_file.write("from .classes import *\n")

    def create_import_from_properties(self):
        self.current_file.write("from .object_properties import *\n")
        self.current_file.write("from .data_properties import *\n")

    def create_namespace(self):
        self.current_file.write(f"ontology = get_ontology('{self.ontology.base_iri}').load()\n\n")

    def create_base_class(self):
        self.current_file.write("class Base(Thing):\n")
        self.current_file.write(self.apply_indent_to("namespace = ontology"))
        self.current_file.write("\n" * 3)

    def create_base_property(self):
        self.current_file.write("class BaseProperty(ObjectProperty):\n")
        self.current_file.write(self.apply_indent_to("namespace = ontology"))
        self.current_file.write("\n" * 3)

    def apply_indent_to(self, string):
        return " " * self.indentation + string.replace('\n', '\n' + ' ' * self.indentation)

    def get_docstring(self, cls):
        """
        Get the docstring for a class.
        """
        docstring = cls.comment
        if docstring:
            docstring = f"\n".join(docstring)
            return (f'"""\n'
                    f'{docstring}\n'
                    f'"""\n')
        else:
            return "...\n"

    def parse_element(self, element):
        """
        Parse an element from the `is_a` field of a class.
        """
        return repr(element)

    def parse_elements(self, elements: List) -> str:
        """
        Parse a list of elements from the `is_a` field of a class.

        :param elements: A list of elements to parse.
        :return: A string representation of the elements.
        """
        return ", ".join(self.parse_element(element) for element in elements)

    def write_docstring(self, cls):
        # apply indent to docstring
        docstring = self.get_docstring(cls)
        docstring = self.apply_indent_to(docstring)
        self.current_file.write(docstring)

    def write_equivalent_to(self, cls):
        equivalent_to = self.parse_elements(cls.equivalent_to)
        if equivalent_to:
            equivalent_to = f"{repr(cls)}.equivalent_to = [{equivalent_to}]"
            self.current_file.write(equivalent_to)
            self.current_file.write("\n")

    def write_is_a(self, cls):
        is_a = self.parse_elements(cls.is_a)
        is_a = f"{repr(cls)}is_a = [{is_a}]"
        self.current_file.write(is_a)
        self.current_file.write("\n")

    def parse_class(self, cls):
        inherited_classes_sting = "Base"
        self.current_file.write(f"class {cls.name}({inherited_classes_sting}):\n")
        self.write_docstring(cls)
        self.current_file.write("\n\n")

    def parse_property(self, prop):
        self.current_file.write(f"class {prop.name}(Base):\n")
        self.write_docstring(prop)
        self.current_file.write("\n")
