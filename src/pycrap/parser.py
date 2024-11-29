import collections

import networkx
import networkx as nx
import owlready2
from graph_tool.topology import topological_sort
from owlready2 import ThingClass, Thing, ObjectProperty
from owlready2.base import *
from owlready2.class_construct import Restriction, SOME, ONLY, VALUE, HAS_SELF, _restriction_type_2_label
import tqdm
from typing_extensions import List


class Parser:
    """
    A class that parses all definitions from an ontology into python files that are owlready2 compatible.

    Not parsed are:
        - Circular dependencies

    """

    ontology: owlready2.Ontology
    """
    The ontology to parse.
    """

    path: str
    """
    The path to write the files of the parsed ontology into.
    """

    classes_file_name: str = "classes.py"
    """
    The file name where all elements from ontology.classes() are written to.
    """

    properties_file_name: str = "properties.py"
    """
    The file name where all elements from ontology.properties() are written to.
    """

    restrictions_file_name: str = "restrictions.py"

    individuals_file_name: str = "individuals.py"

    indentation = 4
    """
    The indentation to use for the python file.
    """

    dependency_graph: nx.DiGraph

    def __init__(self, ontology: owlready2.Ontology, file_name: str):
        self.ontology = ontology
        self.file_name = file_name
        self.file = None
        render_func_without_namespace = lambda entity: entity.name
        owlready2.set_render_func(render_func_without_namespace)

    def create_dependency_graph(self):
        """
        Create the dependency graph of the ontology.
        """
        self.dependency_graph = nx.DiGraph()
        for cls in self.ontology.classes():
            print(cls)
            self.dependency_graph.add_node(cls)
            for other in self.get_concepts_of_elements(cls.is_a):
                self.dependency_graph.add_edge(other, cls)
            for other in self.get_concepts_of_elements(cls.equivalent_to):
                self.dependency_graph.add_edge(other, cls)

        # for prop in owlready2.ObjectProperty.subclasses():
        #     self.dependency_graph.add_node(prop)
        #     for other in self.get_concepts_of_elements(prop.is_a):
        #         self.dependency_graph.add_edge(other, prop)
        #     for other in self.get_concepts_of_elements(prop.equivalent_to):
        #         self.dependency_graph.add_edge(other, prop)
        #     if prop.domain:
        #         for domain in self.get_concepts_of_elements(prop.domain):
        #             self.dependency_graph.add_edge(domain, prop)
        #     if prop.range:
        #         for range in self.get_concepts_of_elements(prop.range):
        #             self.dependency_graph.add_edge(range, prop)
            # if prop.inverse_property:
            #     for inverse_property in self.get_concepts_of_element(prop.inverse_property):
            #         self.graph.add_edge(prop, inverse_property)

    def get_concepts_of_elements(self, elements: List) -> List:
        return [e for element in elements for e in self.get_concepts_of_element(element)]

    def get_concepts_of_element(self, element) -> List:
        if element is ThingClass:
            return []
        elif isinstance(element, ThingClass):
            return [element]
        elif isinstance(element, Thing):
            return [element]
        elif element.__module__ == 'builtins':
            return [element]
        elif isinstance(element, owlready2.prop.DatatypeClass):
            return [element]
        elif isinstance(element, normstr):
            return [element]
        elif isinstance(element, owlready2.prop.PropertyClass) and not isinstance(element, ObjectProperty):
            return []
        elif isinstance(element, owlready2.class_construct.And):
            return self.get_concepts_of_elements(element.Classes)
        elif isinstance(element, owlready2.class_construct.Or):
            return self.get_concepts_of_elements(element.Classes)
        elif isinstance(element, owlready2.class_construct.Restriction):
            return self.get_concepts_of_element(element.value)
        elif isinstance(element, owlready2.class_construct.Not):
            return self.get_concepts_of_element(element.Class)
        elif isinstance(element, owlready2.class_construct.OneOf):
            return self.get_concepts_of_elements(element.instances)
        elif element.__module__ == "owlready2.util":
            return []
        else:
            raise NotImplementedError(f"Cant get concepts of {element} with type {type(element)}")

    def parse(self):
        """
        Parses the ontology into a python file.
        """
        self.create_dependency_graph()
        self.file = open(self.file_name, "w")
        self.create_imports()
        self.create_namespace()
        self.create_base_class()
        self.create_base_property()
        self.create_nodes_and_properties()
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


    def create_nodes_and_properties(self):

        # start will all nodes that have no incoming edges
        assert nx.is_directed_acyclic_graph(self.dependency_graph), "Only DAGs can be parsed for now."
        for node in nx.topological_sort(self.dependency_graph):
            if isinstance(node, owlready2.prop.ObjectPropertyClass):
                self.parse_property(node)
            elif isinstance(node, owlready2.ThingClass):
                self.parse_class(node)
            else:
                continue
                raise NotImplementedError(f"Parsing of node {node} with type {type(node)} is not supported.")
            self.file.write("\n\n")


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
            equivalent_to = self.apply_indent_to(f"equivalent_to = [{equivalent_to}]")
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
        self.write_is_a(cls)
        self.write_equivalent_to(cls)

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
