from os import mkdir

import networkx as nx
import owlready2
import tqdm
from owlready2 import ThingClass, PropertyClass
from owlready2.base import *
from typing_extensions import List, Any


def to_snake_case(string: str) -> str:
    """
    Convert a string to snake case.

    :param string: The string to convert.
    :return: The string in snake case.
    """
    return string.replace(" ", "_").lower()

def to_camel_case(string: str) -> str:
    """
    Convert a string to camel case.

    :param string: The string to convert.
    :return: The string in camel case.
    """
    return ''.join(x for x in string.title() if not x.isspace())

class OntologyParser:
    """
    A class that parses everything from an owlready2 compatible ontology into python files that
    represent the same ontology.

    It will create several files in a directory specified by the constructor:
    - base: A file that contains an intermediate based class and temporary ontology definitions such that a user
    can configure it at the end of the process.
    - classes: A file that contains all classes from the ontology without any restrictions.
    - object_properties: A file the contains all object properties from the ontology without any restrictions.
    - data_properties: A file the contains all data properties from the ontology without any restrictions.
    - restrictions: The restrictions for all classes and properties.
    - individuals: All indivduals with the resepctive properties from the ontology.
    - __init__.py: A package initialization that loads the content of all files.

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
    """
    The current file where definitions are written into.
    """

    def __init__(self, ontology: owlready2.Ontology, path: str):
        self.ontology = ontology
        self.path = path
        render_func_without_namespace = lambda entity: entity.name
        owlready2.set_render_func(render_func_without_namespace)


    def digit_to_string(self, cls):
        # Mapping of digits to words
        digit_map = {
            '0': 'Zero', '1': 'One', '2': 'Two', '3': 'Three', '4': 'Four',
            '5': 'Five', '6': 'Six', '7': 'Seven', '8': 'Eight', '9': 'Nine'
        }

        # Replace each digit with its corresponding word
        converted_name = ''.join(digit_map[char] if char.isdigit() else char for char in cls)

        return converted_name

    def parse(self, additional_imports=None):
        """
        Parses the ontology into a python file.
        """
        self.create_base(additional_imports)
        self.create_classes()
        self.create_object_properties()
        self.create_data_properties()
        self.create_individuals()
        self.create_restrictions()
        # create swrl rules
        self.create_init()

    def create_init(self):
        """
        Create the __init__.py
        """
        self.current_file = open(self.path_for_file("__init__"), "w")
        self.import_from_classes()
        self.import_from_properties()
        self.import_individuals()
        self.current_file.close()

    def create_base(self, additional_imports=None):
        """
        Create the base.py
        """
        self.current_file = open(self.path_for_file(self.base_file_name), "w")
        self.create_base_imports(additional_imports)
        self.current_file.write("\n" * 2)
        self.current_file.write("ontology_file = tempfile.NamedTemporaryFile()\n")
        self.current_file.write('ontology = owlready2.get_ontology("file://" + ontology_file.name).load()\n')
        self.current_file.write("\n" * 2)
        self.create_base_class()
        self.create_base_property()
        self.current_file.close()

    def path_for_file(self, file_name):
        """
        Generate the path for a file.

        Example:
        >>> parser = OntologyParser(ontology, "/tmp")
        >>> parser.path_for_file("base")
            "/tmp/base.py"

        :param file_name: The file name.
        :return: The path to the file.
        """
        return f"{os.path.join(self.path, file_name)}{self.file_extension}"

    def create_classes(self):
        """
        Create the classes.py
        """
        self.current_file = open(self.path_for_file(self.classes_file_name), "w")
        self.import_from_base()
        self.current_file.write("\n" * 2)
        classes = list(self.ontology.classes())
        for cls in tqdm.tqdm(classes, desc="Parsing classes"):
            self.parse_class(cls)
        self.current_file.close()

    def create_object_properties(self):
        """
        Create the object_properties.py
        """
        self.current_file = open(self.path_for_file(self.object_properties_file_name), "w")
        self.import_from_base()
        self.current_file.write("\n" * 2)
        properties = list(self.ontology.object_properties())
        for prop in tqdm.tqdm(properties, desc="Parsing object properties"):
            self.parse_property(prop)
        self.current_file.close()

    def create_data_properties(self):
        """
        Create the data_properties.py
        """
        self.current_file = open(self.path_for_file(self.data_properties_file_name), "w")
        self.import_from_base()
        self.import_from_classes()
        self.current_file.write("\n" * 2)
        properties = list(self.ontology.data_properties())
        for prop in tqdm.tqdm(properties, desc="Parsing data properties"):
            self.parse_property(prop)
        self.current_file.close()

    def create_restrictions(self):
        """
        Create the restrictions.py
        """
        self.current_file = open(self.path_for_file(self.restrictions_file_name), "w")
        self.import_from_classes()
        self.import_individuals()
        self.current_file.write("\n" * 2)

        for cls in self.ontology.classes():
            if cls.name[0].isdigit():
                original_name = cls.name
                new_name = self.digit_to_string(original_name)
                if new_name != original_name:
                    cls.name = new_name

            if "-" in cls.name:
                original_name = cls.name
                new_name = cls.name.replace("-", "")
                if new_name != original_name:
                    cls.name = new_name

        elements = list(self.ontology.classes()) + list(self.ontology.properties())

        for element in tqdm.tqdm(elements, desc="Parsing restrictions"):
            self.parse_restrictions_for(element)
            self.current_file.write("\n")
        self.current_file.close()

    def parse_restrictions_for(self, element):
        """
        Create all restriction for any element of the ontology.
        """
        if isinstance(element, ThingClass):
            self.parse_restrictions_for_class(element)
        elif isinstance(element, PropertyClass):
            self.parse_restrictions_for_property(element)

    def parse_restrictions_for_class(self, cls: ThingClass):
        """
        Create the restrictions for a class.

        :param cls: The class
        """
        # TODO: requiring a digit_to_string for the restrictions body.
        # TODO: array_double will be converted to float for now, discuss
        # write is_a restrictions
        is_a = self.parse_elements(cls.is_a)
        if is_a:
            if "-" in str(is_a):
                is_a = str(is_a).replace("-","")
            if "array_double" in str(is_a):
                is_a = str(is_a).replace("array_double","float")
            if "<class 'int'>" in str(is_a):
                is_a = str(is_a).replace("<class 'int'>", "int")
            if "<class 'str'>" in str(is_a):
                is_a = str(is_a).replace("<class 'str'>", "str")
            if "<class 'owlready2.util.normstr'>" in str(is_a):
                is_a = str(is_a).replace("<class 'owlready2.util.normstr'>", "normstr")
            if "<class 'float'>" in str(is_a):
                is_a = str(is_a).replace("<class 'float'>", "float")
            is_a = f"{repr(cls)}.is_a = [{is_a}]"
            self.current_file.write(is_a)
            self.current_file.write("\n")

        # write equivalent_to restrictions
        self.write_equivalent_to(cls)

    def import_individuals(self):
        """
        Write the import statement that imports individuals.
        """
        self.current_file.write("from .individuals import *\n")

    def parse_restrictions_for_property(self, prop):
        """
        Write all restrictions for a property.
        :param prop: The property
        """
        # write is_a restrictions
        is_a = self.parse_elements(prop.is_a)
        # Skip AnnotationProperties for now
        if not "AnnotationProperty" in is_a:
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
                if "<class 'int'>" in str(range_string):
                    range_string = str(range_string).replace("<class 'int'>", "int")
                if "<class 'str'>" in str(range_string):
                    range_string = str(range_string).replace("<class 'str'>", "str")
                if "<class 'owlready2.util.normstr'>" in str(range_string):
                    range_string = str(range_string).replace("<class 'owlready2.util.normstr'>", "normstr")
                if "<class 'float'>" in str(range_string):
                    range_string = str(range_string).replace("<class 'float'>", "float")
                if "array_double" in str(range_string):
                    range_string = str(range_string).replace("array_double","float")
                if "<class 'datetime.datetime'>" in str(range_string):
                    range_string = str(range_string).replace("<class 'datetime.datetime'>", "datetime")


                range_string = f"{repr(prop)}.range = [{range_string}]"
                self.current_file.write(range_string)
                self.current_file.write("\n")

    def create_individuals(self):
        """
        Create all individuals of the ontology.
        """
        self.current_file = open(f"{os.path.join(self.path, self.individuals_file_name)}{self.file_extension}", "w")
        self.import_from_base()
        self.import_from_classes()
        self.import_from_properties()
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
        """
        Parse the construction of an individual.
        :param individual: The individual.
        """
        self.current_file.write(f"{individual.name} = {repr(individual.__class__)}(namespace = ontology)")
        self.current_file.write("\n")

    def parse_individual_properties(self, individual: owlready2.Thing):
        """
        Parse the properties of an individual.
        :param individual: The individual.
        """
        for prop in individual.get_properties():
            self.current_file.write(f"{individual.name}.{repr(prop)} = {individual.__getattr__(repr(prop))}")
            self.current_file.write("\n")

    def create_base_imports(self, additional_imports=None):
        """
        Create the imports for the base.py
        """
        self.current_file.write("from owlready2 import *\n")
        self.current_file.write("import tempfile\n")

        # Additional imports for other ontologies
        if additional_imports:
            for import_path in additional_imports:
                self.current_file.write(f"from {import_path} import *\n")

    def import_from_base(self):
        """
        Create the import statement to get everything from the base.py
        """
        self.current_file.write("from .base import *\n")

    def import_from_classes(self):
        """
        Create the import statement to get everything from the classes.py
        """
        self.current_file.write("from .classes import *\n")

    def import_from_properties(self):
        """
        Create the import statement to get everything from the properties
        """
        self.current_file.write("from .object_properties import *\n")
        self.current_file.write("from .data_properties import *\n")

    def create_base_class(self):
        """
        Create the base class for concepts.
        """
        self.current_file.write("class Base(Thing):\n")
        self.current_file.write(self.apply_indent_to("namespace = ontology"))
        self.current_file.write("\n" * 3)

    def create_base_property(self):
        """
        Create the base class for properties.
        """
        self.current_file.write("class BaseProperty(ObjectProperty):\n")
        self.current_file.write(self.apply_indent_to("namespace = ontology"))
        self.current_file.write("\n" * 3)

    def apply_indent_to(self, string):
        """
        Indent a statement at the beginning of every new line.

        :param string: The statement.
        :return: The indented string.
        """
        return " " * self.indentation + string.replace('\n', '\n' + ' ' * self.indentation)

    def get_docstring(self, cls) -> str:
        """
        Get the docstring for a class.

        :param cls: The class
        :return: The docstring for the class or "..." if no docstring is found.
        """
        docstring = cls.comment
        if docstring:
            docstring = f"\n".join(docstring)
            return (f'"""\n'
                    f'{docstring}\n'
                    f'"""\n')
        else:
            return "...\n"

    def parse_element(self, element) -> str:
        """
        Parse an element for representation the source code.

        :param element: The element to parse.
        :return: A string representation that can be used in python files.
        """
        return repr(element)

    def parse_elements(self, elements: List) -> str:
        """
        Parse a list of elements from for the representation in the source code.
        An input can be, for instance, the is_a` field of a class.

        :param elements: A list of elements to parse.
        :return: A string representation of the elements.
        """
        return ", ".join(self.parse_element(element) for element in elements)

    def write_docstring(self, cls):
        """
        Write the docstring of a class to the current file.
        :param cls: The class.
        """
        # apply indent to docstring
        docstring = self.get_docstring(cls)
        docstring = self.apply_indent_to(docstring)
        self.current_file.write(docstring)

    def write_equivalent_to(self, cls):
        """
        Write the `equivalent_to` field of a class

        :param cls: The class.
        """
        equivalent_to = self.parse_elements(cls.equivalent_to)
        if equivalent_to:
            equivalent_to = f"{repr(cls)}.equivalent_to = [{equivalent_to}]"
            self.current_file.write(equivalent_to)
            self.current_file.write("\n")

    def write_is_a(self, cls):
        """
        Write the `is_a` field of a class
        :param cls: The class.
        """
        is_a = self.parse_elements(cls.is_a)
        is_a = f"{repr(cls)}is_a = [{is_a}]"
        self.current_file.write(is_a)
        self.current_file.write("\n")



    # TODO: Decide upon a better solution for the hyphen symbol
    def parse_class(self, cls):
        """
        Parse a class without restrictions.
        :param cls: The class.
        """
        if cls.name[0].isdigit():
            # Prepend "I" to make the class name valid
            #modified_class_name = "I" + cls.name
            modified_class_name = self.digit_to_string(cls.name)
        else:
            modified_class_name = cls.name

        if "-" in modified_class_name:
            modified_class_name = modified_class_name.replace("-", "")

        inherited_classes_sting = "Base"
        self.current_file.write(f"class {modified_class_name}({inherited_classes_sting}):\n")
        self.write_docstring(cls)
        self.current_file.write("\n\n")

    def parse_property(self, prop):
        """
        Parse a property without restrictions.
        :param prop: The property.
        """
        self.current_file.write(f"class {prop.name}(BaseProperty):\n")
        self.write_docstring(prop)
        self.current_file.write("\n")


class OntologiesParser:
    """
    Class that parses multiple ontologies at once.

    The resulting python package has the following form


    path/__init__.py
    path/base.py
    path/ontology1/__init__.py
    path/ontology1/classes.py
    path/ontology1/object_properties.py
    path/ontology1/data_properties.py
    path/ontology1/restrictions.py
    path/ontology1/individuals.py
    path/ontology2/__init__.py
    path/ontology2/classes.py
    ...
    """

    ontologies: List[owlready2.Ontology]
    """
    The ontologies to parse.
    """

    path: str
    """
    The path to write the packages of the parsed ontology into.
    """

    include_imported_ontologies = True
    """
    If True, the imported ontologies are also parsed.
    """

    dependency_graph: nx.DiGraph
    """
    The dependency graph of the ontologies.
    """

    def __init__(self, ontologies: List[owlready2.Ontology], path: str):
        self.ontologies = ontologies
        self.path = path

        if self.include_imported_ontologies:
            self.ontologies += [imported for onto in self.ontologies for imported in onto.imported_ontologies]

        # make ontologies unique
        self.ontologies = list(set(self.ontologies))

        self.create_dependency_graph()

    def create_dependency_graph(self):
        """
        Create the dependency graph of the ontologies.
        """
        self.dependency_graph = nx.DiGraph()
        for onto in self.ontologies:
            self.dependency_graph.add_node(onto)
            for imported in onto.imported_ontologies:
                self.dependency_graph.add_edge(imported, onto)



    # def create_ontologies(self):
    #     for onto in nx.topological_sort(self.dependency_graph):
    #         mkdir(os.path.join(self.path, to_snake_case(onto.name)))
    #         print(to_snake_case(onto.name))
    #         parser = OntologyParser(onto, os.path.join(self.path, to_snake_case(onto.name)))
    #         # Determine which ontologies depend on this one
    #         dependents = [to_snake_case(dep.name) for dep in self.dependency_graph.successors(onto)]
    #         additional_imports = [f"..{dep}" for dep in dependents]
    #
    #         # Parse the ontology, passing its dependents for imports
    #         parser.parse(additional_imports)

    def create_ontologies(self):
        for node, successors in self.dependency_graph.adjacency():
            print(f"{node.name}: {[succ.name for succ in successors]}")

        self.dependency_graph = nx.reverse(self.dependency_graph, copy=True)
        for onto in nx.topological_sort(self.dependency_graph):

            # Create the directory for the ontology
            ontology_path = os.path.join(self.path, to_snake_case(onto.name))
            mkdir(ontology_path)

            # Parse the ontology
            parser = OntologyParser(onto, ontology_path)

            # Determine which ontologies depend on this one
            dependents = [to_snake_case(dep.name) for dep in self.dependency_graph.successors(onto)]
            print(f"Dependents of {onto.name}: {dependents}")

            additional_imports = [f"..{dep}" for dep in dependents]
            print(f"Imports added to {onto.name}: {additional_imports}")

            # Parse the ontology, passing its dependents for imports
            parser.parse(additional_imports)