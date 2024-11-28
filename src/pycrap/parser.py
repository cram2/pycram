import collections

import owlready2
from owlready2.base import *
from owlready2.class_construct import Restriction, SOME, ONLY, VALUE, HAS_SELF, _restriction_type_2_label
import tqdm
from typing_extensions import List


class Parser:
    """
    A class that parses all definitions from an ontology into python files that are owlready2 compatible.

    Not parsed are:
        - Restrictions
        - Inverse properties
        - Functional properties
        and perhaps something more.
    """

    ontology: owlready2.Ontology
    """
    The ontology to parse.
    """

    file_name: str
    """
    The file name to write the parsed ontology to.
    """

    indentation = 4
    """
    The indentation to use for the python file.
    """

    def __init__(self, ontology: owlready2.Ontology, file_name: str):
        self.ontology = ontology
        self.file_name = file_name
        self.file = None
        render_func_without_namespace = lambda entity: entity.name
        owlready2.set_render_func(render_func_without_namespace)


    def parse(self):
        """
        Parses the ontology into a python file.
        """
        self.file = open(self.file_name, "w")
        self.create_imports()
        self.create_namespace()
        self.create_base_class()
        self.create_base_property()
        self.parse_classes()
        self.parse_properties()
        self.file.close()

    def create_imports(self):
        self.file.write("from owlready2 import *\n\n")

    def create_namespace(self):
        self.file.write(f"default_namespace = get_ontology('{self.ontology.base_iri}').load()\n\n")

    def create_base_class(self):
        self.file.write("class Base(Thing):\n")
        self.file.write("    namespace = default_namespace\n\n")

    def create_base_property(self):
        self.file.write("class BaseProperty(ObjectProperty):\n")
        self.file.write("    namespace = default_namespace\n\n")

    def classes_in_bfs(self):
        visited = []
        all_classes = collections.deque(owlready2.Thing.subclasses())
        while all_classes:
            cls = all_classes.popleft()
            if cls in visited:
                continue
            all_classes.extend(cls.subclasses())
            yield cls
            visited.append(cls)

    def properties_in_bfs(self):
        visited = []
        all_properties = collections.deque(owlready2.ObjectProperty.subclasses())
        while all_properties:
            prop = all_properties.popleft()

            if prop in visited:
                continue

            all_properties.extend(prop.subclasses())
            yield prop
            visited.append(prop)

    def apply_indent_to(self, string):
        return " " * self.indentation + string.replace('\n', '\n' + ' ' * self.indentation)

    def get_docstring(self, cls):
        """
        Get the docstring for a class.
        """
        docstring = cls.comment
        docstring = f"\n".join(docstring)
        return (f'"""\n'
                f'{docstring}\n'
                f'"""\n')

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
        self.file.write(docstring)

    def write_equivalent_to(self, cls):
        equivalent_to = self.parse_elements(cls.equivalent_to)
        if equivalent_to:
            equivalent_to = self.apply_indent_to("equivalent_to = [{equivalent_to}]")
            self.file.write(equivalent_to)
            self.file.write("\n")

    def write_is_a(self, cls):
        is_a = self.parse_elements(cls.is_a)
        is_a = self.apply_indent_to(f"is_a = [{is_a}]")
        self.file.write(is_a)
        self.file.write("\n")

    def parse_class(self, cls):
        inherited_classes_sting = "Base"  # , ".join(self.parse_element(parent) for parent in cls.is_a)
        self.file.write(f"class {cls.name}({inherited_classes_sting}):\n")
        self.write_docstring(cls)
        self.file.write("\n")
        is_a = self.parse_elements(cls.is_a)
        is_a = f"    is_a = [{is_a}] \n"
        self.file.write(is_a)
        self.write_equivalent_to(cls)

    def parse_classes(self):
        """
        Parses all classes from the ontology.
        """
        for cls in self.classes_in_bfs():
            self.parse_class(cls)
            self.file.write("\n\n")

    def parse_property(self, prop):
        self.file.write(f"class {prop.name}(Base):\n")
        self.write_docstring(prop)
        self.file.write("\n")

        domain_string = self.parse_elements(prop.domain)
        if domain_string:
            domain_string = self.apply_indent_to(f"domain = [{domain_string}]")
            self.file.write(domain_string)
            self.file.write("\n")

        range_string = self.parse_elements(prop.range)
        if range_string:
            range_string = self.apply_indent_to(f"range = [{range_string}]")
            self.file.write(range_string)
            self.file.write("\n")

        self.write_equivalent_to(prop)
        self.write_is_a(prop)

        # check if an inverse property exists
        if prop.inverse_property:
            inverse_property = self.apply_indent_to(f"inverse_property = {self.parse_element(prop.inverse_property)}")
            self.file.write(inverse_property)
            self.file.write("\n")


    def parse_properties(self):
        """
        Parses all properties from the ontology.

        ..Note:: This does not check for transitively, etc.

        """
        for prop in self.properties_in_bfs():
            self.parse_property(prop)
            self.file.write("\n")
