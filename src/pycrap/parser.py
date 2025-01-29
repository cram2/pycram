import shutil
from os import mkdir

import inflection
import networkx as nx
import owlready2
import tqdm
from owlready2 import ThingClass, PropertyClass, get_ontology, LogicalClassConstruct
from owlready2.base import *
from typing_extensions import List, Any

# Mapping of digits to words
digit_map = {
    '0': 'Zero', '1': 'One', '2': 'Two', '3': 'Three', '4': 'Four',
    '5': 'Five', '6': 'Six', '7': 'Seven', '8': 'Eight', '9': 'Nine'
}


def set_explicit_repr_for_logical_operator():
    def explicit_repr(self):
        s = []
        for x in self.Classes:
            if isinstance(x, LogicalClassConstruct):
                s.append("(%s)" % x)
            else:
                s.append(repr(x))
        return "%s([%s])" % (self.__class__.__name__, ", ".join(s))

    LogicalClassConstruct.__repr__ = explicit_repr


def to_snake_case(string: str) -> str:
    """
    Convert a string to snake case.

    :param string: The string to convert.
    :return: The string in snake case.
    """
    return inflection.underscore(string)


def to_camel_case(string: str) -> str:
    """
    Convert a string to camel case.

    :param string: The string to convert.
    :return: The string in camel case.
    """
    return inflection.camelize(string)


def replace_types(string: str) -> str:
    """
    Replace the types in a string with python types

    Example:
    >>> replace_types("array_double__<class 'int'>")
    "float__int"

    # TODO array_double will be converted to float for now
    :param string: The string to convert
    :return: The string with the types replaced
    """
    if "-" in str(string):
        string = str(string).replace("-", "")
    if "array_double" in str(string):
        string = str(string).replace("array_double", "float")
    if "<class 'int'>" in str(string):
        string = str(string).replace("<class 'int'>", "int")
    if "<class 'str'>" in str(string):
        string = str(string).replace("<class 'str'>", "str")
    if "<class 'owlready2.util.normstr'>" in str(string):
        string = str(string).replace("<class 'owlready2.util.normstr'>", "normstr")
    if "<class 'float'>" in str(string):
        string = str(string).replace("<class 'float'>", "float")
    if "<class 'bool'>" in str(string):
        string = str(string).replace("<class 'bool'>", "bool")
    if "<class 'datetime.datetime'>" in str(string):
        string = str(string).replace("<class 'datetime.datetime'>", "datetime.datetime")
    return string


def update_class_names(onto: owlready2.Ontology):
    """
    Update the class names to match python conventions.
    """
    for cls in onto.classes():
        if cls is owlready2.Thing:
            continue
        converted_name = replace_types(cls.name)
        converted_name = to_camel_case(converted_name)
        converted_name = ''.join(digit_map[char] if char.isdigit() else char for char in converted_name)
        type.__setattr__(cls, "_name", converted_name)


def update_property_names(onto: owlready2.Ontology):
    """
    Update the property names to match python conventions of functions and members.
    """
    for prop in onto.properties():
        converted_name = to_snake_case(prop.name)
        converted_name = ''.join(digit_map[char] if char.isdigit() else char for char in converted_name)
        type.__setattr__(prop, "original_name", prop.name)
        type.__setattr__(prop, "_name", converted_name)


class AbstractParser:
    """
    An abstract class for parsing
    """

    current_file: Any
    """
    The current file where contents are written into.
    """

    indentation: int
    """
    The indentation to use for the python file.
    """

    file_extension: str = ".py"
    """
    The file extension for the python file.
    """

    path: str
    """
    The path to write the files of the parsed ontology into.
    """

    def __init__(self, path, indentation: int = 4):
        self.path = path
        self.indentation = indentation

    def apply_indent_to(self, string):
        """
        Indent a statement at the beginning of every new line.

        :param string: The statement.
        :return: The indented string.
        """
        return " " * self.indentation + string.replace('\n', '\n' + ' ' * self.indentation)

    def path_for_file(self, file_name):
        """
        Generate the path for a file.

        Example:
        >>> parser = AbstractParser("/tmp")
        >>> parser.path_for_file("base")
            "/tmp/base.py"

        :param file_name: The file name.
        :return: The path to the file.
        """
        return f"{os.path.join(self.path, file_name)}{self.file_extension}"


class OntologyParser(AbstractParser):
    """
    A class that parses everything from an owlready2 compatible ontology into python files that
    represent the same ontology.

    It will create several files in a directory specified by the constructor:
    - dependencies: A file that the imports from other packages/modules that are needed to define this ontology.
    - classes: A file that contains all classes from the ontology without any restrictions.
    - object_properties: A file the contains all object properties from the ontology without any restrictions.
    - data_properties: A file the contains all data properties from the ontology without any restrictions.
    - restrictions: The restrictions for all classes and properties.
    - individuals: All individuals with the respective properties from the ontology.
    - __init__.py: A package initialization that loads the content of all files and hence sets the restrictions
                   and so on.

    TODO: Labels metadata and SWRL is not parsed
    """

    ontology: owlready2.Ontology
    """
    The ontology to parse.
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
    """
    The file name where all elements from ontology.data_properties() are written to.
    """

    restrictions_file_name: str = "restrictions"
    """
    The file name where all restrictions are written to.
    """

    individuals_file_name: str = "individuals"
    """
    The file name where all individuals are written to.
    """

    dependencies_file_name: str = "dependencies"
    """
    The file name where all dependencies are written to.
    """

    dependencies: List[owlready2.Ontology]
    """
    The other ontologies that have to be imported.
    """

    def __init__(self, ontology: owlready2.Ontology, dependencies: List[owlready2.Ontology], path: str,
                 indentation: int = 4):
        super().__init__(path, indentation)
        self.ontology = ontology
        self.dependencies = dependencies
        # TODO update class and property names here to match python conventions

    def parse(self):
        """
        Parses the ontology into a python file.
        """
        self.create_dependencies()
        self.create_classes()
        self.create_object_properties()
        self.create_data_properties()
        self.create_individuals()
        self.create_restrictions()
        # create swrl rules
        self.create_init()

    def create_dependencies(self):
        self.current_file = open(self.path_for_file(self.dependencies_file_name), "w")
        self.current_file.write("from ..base import *\n")
        for dependency in self.dependencies:
            self.current_file.write(f"from ..{to_snake_case(dependency.name)} import *\n")
        self.current_file.close()

    def create_init(self):
        """
        Create the __init__.py
        """
        self.current_file = open(self.path_for_file("__init__"), "w")
        self.import_classes()
        self.import_properties()
        self.import_individuals()
        self.import_restrictions()
        self.current_file.close()

    def import_restrictions(self):
        """
        Write the import statement that imports restrictions.
        """
        self.current_file.write("from .restrictions import *\n")

    def import_dependencies(self):
        """
        Import from the dependencies.
        """
        self.current_file.write(f"from .{self.dependencies_file_name} import *\n")

    def create_classes(self):
        """
        Create the classes.py
        """
        self.current_file = open(self.path_for_file(self.classes_file_name), "w")
        self.import_dependencies()
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
        self.import_dependencies()
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
        self.import_dependencies()
        self.import_classes()
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
        self.import_dependencies()
        self.import_classes()
        self.import_individuals()
        self.current_file.write("\n" * 2)

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
        # write is_a restrictions
        is_a = self.parse_elements(cls.is_a)
        if is_a:
            is_a = f"{repr(cls)}.is_a = [{replace_types(str(is_a))}]"
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
        #Some Properties restrictions may not be converted to snake case yet, this function ensures that restrictions of
        #the properties matches the defined properties.
        updated_is_a = []
        for i in prop.is_a:
            if i not in self.ontology.properties():
                for l in self.ontology.properties():
                    if l.name == to_snake_case(i.name):
                        i = l
            updated_is_a.append(i)

        is_a = self.parse_elements(updated_is_a)
        #print(*self.ontology.properties())
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
                range_string = f"{repr(prop)}.range = [{replace_types(str(range_string))}]"
                self.current_file.write(range_string)
                self.current_file.write("\n")

    def create_individuals(self):
        """
        Create all individuals of the ontology.
        """
        self.current_file = open(f"{os.path.join(self.path, self.individuals_file_name)}{self.file_extension}", "w")
        self.import_dependencies()
        self.import_classes()
        self.import_properties()
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
            if "original_name" not in dir(prop):
                continue
           #Some properties from individuals may be buggy, so only use it on demand.
#           self.current_file.write(f"{individual.name}.{prop.name} = {individual.__getattr__(prop.original_name)}")
            self.current_file.write("\n")

    def import_classes(self):
        """
        Create the import statement to get everything from the classes.py
        """
        self.current_file.write("from .classes import *\n")

    def import_properties(self):
        """
        Create the import statement to get everything from the properties
        """
        self.current_file.write("from .object_properties import *\n")
        self.current_file.write("from .data_properties import *\n")

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
        label = cls.label
        if docstring and label:
            docstring = f"\n".join(docstring)
            label = f"\n".join(label)
            return (f'"""\n'
                    f'label: {label}\n'
                    f'{docstring}\n'
                    f'"""\n')
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
        # Adding the full iri and base iri as class attributes.
        inherited_classes_sting = "Base"
        fulliri = "full_iri = " + "'" + cls.iri + "'"
        baseiri = "base_iri = " + "'" + cls.iri.split("#")[0] + "'"
        self.current_file.write(f"class {repr(cls)}({inherited_classes_sting}):\n")
        self.write_docstring(cls)
        self.current_file.write(f"\n{self.apply_indent_to(fulliri)}\n")
        #self.current_file.write("\n\n")
        self.current_file.write(f"\n{self.apply_indent_to(baseiri)}\n")
        self.current_file.write("\n\n")

    def parse_property(self, prop):
        """
        Parse a property without restrictions.
        :param prop: The property.
        """
        # Adding the full iri and base iri as property attributes.
        fulliri = "full_iri = " + "'" + prop.iri + "'"
        baseiri = "base_iri = " + "'" + prop.iri.split("#")[0] + "'"
        self.current_file.write(f"class {prop.name}(BaseProperty):\n")
        self.write_docstring(prop)
        self.current_file.write(f"\n{self.apply_indent_to(fulliri)}\n")
        self.current_file.write(f"\n{self.apply_indent_to(baseiri)}\n")
        self.current_file.write("\n\n")


class OntologiesParser(AbstractParser):
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

    include_imported_ontologies = True
    """
    If True, the imported ontologies are also parsed.
    """

    dependency_graph: nx.DiGraph
    """
    The dependency graph of the ontologies.
    """

    base_file_name: str = "base"
    """
    The file name where the base classes are written to.
    """

    clear_existing: bool = True
    """
    If True, the existing directories are cleared.
    """

    def __init__(self, ontologies: List[owlready2.Ontology], path: str, indentation: int = 4):

        render_func_without_namespace = lambda entity: entity.name
        owlready2.set_render_func(render_func_without_namespace)

        super().__init__(path, indentation)
        self.ontologies = ontologies

        if self.include_imported_ontologies:
            self.ontologies += [imported for onto in self.ontologies for imported in onto.imported_ontologies]

        # make ontologies unique
        self.ontologies = list(set(self.ontologies))
        self.ontologies.sort(key=lambda onto: onto.name)
        self.create_dependency_graph()
        set_explicit_repr_for_logical_operator()

    def create_base(self):
        """
        Create the base file
        """
        self.current_file = open(self.path_for_file(self.base_file_name), "w")
        self.current_file.write(
            "from owlready2 import Thing, ThingClass, ObjectProperty, get_ontology, And, Or, Not, OneOf, Inverse, "
            "normstr, DatatypeProperty, TransitiveProperty, SymmetricProperty, AsymmetricProperty, ReflexiveProperty, "
            "IrreflexiveProperty, datetime \n")
        self.current_file.write("import tempfile\n")
        self.current_file.write("\n" * 2)
        self.current_file.write("ontology_file = tempfile.NamedTemporaryFile()\n")
        self.current_file.write('ontology = get_ontology("file://" + ontology_file.name).load()\n')
        self.current_file.write("\n" * 2)
        self.create_base_class()
        self.create_base_property()
        self.current_file.close()

    def create_base_class(self):
        """
        Create the base class for concepts.
        """
        self.current_file.write("class Base(Thing, metaclass=ThingClass):\n")
        self.current_file.write(self.apply_indent_to("namespace = ontology"))
        self.current_file.write("\n" * 3)

    def create_base_property(self):
        """
        Create the base class for properties.
        """
        self.current_file.write("class BaseProperty(ObjectProperty):\n")
        self.current_file.write(self.apply_indent_to("namespace = ontology"))
        self.current_file.write("\n" * 3)

    def destroy_all_ontologies(self):
        """
        Destroy all ontologies.
        """
        for onto in self.ontologies:
            onto.destroy()

    def create_dependency_graph(self):
        """
        Create the dependency graph of the ontologies.
        """
        self.dependency_graph = nx.DiGraph()
        for onto in self.ontologies:
            self.dependency_graph.add_node(onto)
            for imported in onto.imported_ontologies:
                self.dependency_graph.add_edge(imported, onto)

    def parse(self):
        self.create_base()
        self.create_ontologies()

    def create_init(self):
        """
        Create the __init__.py
        """
        self.current_file = open(self.path_for_file("__init__"), "w")
        self.current_file.write("from .base import *\n")
        for onto in self.ontologies:
            self.current_file.write(f"from .{to_snake_case(onto.name)} import *\n")
        self.current_file.close()

    def create_ontologies(self):
        self.destroy_all_ontologies()

        for onto in nx.topological_sort(self.dependency_graph):
            loaded_onto: owlready2.Ontology = get_ontology(onto.base_iri)
            # Some Ontologies require a local path to be parsed.
            if loaded_onto.base_iri == "http://knowrob.org/kb/urdf.owl#":
                loaded_onto: owlready2.Ontology = get_ontology("https://raw.githubusercontent.com/knowrob/knowrob/refs/heads/dev/owl/URDF.owl")

            if loaded_onto.base_iri == "http://www.ease-crc.org/ont/SUTURO.owl#":
            #    loaded_onto: owlready2.Ontology = get_ontology("https://raw.githubusercontent.com/SUTURO/suturo_knowledge/refs/heads/fallschool/suturo_knowledge/owl/suturo.owl")
                loaded_onto:  owlready2.Ontology = get_ontology("suturo.owl")

            if loaded_onto.base_iri == "http://www.ease-crc.org/ont/meals#":
                loaded_onto: owlready2.Ontology = get_ontology("meals.owl")

            loaded_onto.load()
            update_class_names(loaded_onto)
            update_property_names(loaded_onto)
            # Create the directory for the ontology
            ontology_path = os.path.join(self.path, to_snake_case(loaded_onto.name))

            if self.clear_existing:
                if os.path.exists(ontology_path):
                    shutil.rmtree(ontology_path)

            mkdir(ontology_path)

            # Parse the ontology
            parser = OntologyParser(loaded_onto, list(self.dependency_graph.predecessors(onto)), ontology_path)
            parser.parse()

        # Create the __init__.py
        self.create_init()
