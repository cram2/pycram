from enum import EnumMeta
from pycram.designators.action_designator import ActionAbstract
from random_events.utils import recursive_subclasses
from owlready2 import *
from random_events.utils import get_full_class_name
from typing import Dict, List, Type, Optional, Any, get_origin, Union, get_args
from dataclasses import dataclass
import inspect
import ast
import re

@dataclass
class ParameterDigest:
    """
    Encapsulation of meta information about a parameter.
    """
    clazz: Any
    """
    Class of the parameter.
    """
    parameter_name: str
    """
    Name of the parameter.
    """
    docstring_of_parameter_clazz: str
    """
    Docstring of the parameter's class.
    """
    docstring_of_parameter: str
    """
    Docstring of the parameter itself (individual to each performable).
    """
    parameter_default_value: Any
    """
    Holds the default value of the parameter if set.
    """
    is_enum: bool
    """
    True if the parameter is an enum. Else False.
    """
    is_optional: bool
    """
    If the parameter type hint is optional, this will be True. Else False.
    """

    def get_default_value(self):
        if self.parameter_default_value == inspect.Parameter.empty:
            return None
        else:
            return [str(self.parameter_default_value)]

class ActionAbstractDigest:
    def __init__(self, clazz):
        self.clazz: Type[ActionAbstract] = clazz
        self.full_name: str = get_full_class_name(clazz)
        self.classname: str = clazz.__name__
        self.docstring: str = clazz.__doc__
        self.parameters: Optional[List[ParameterDigest]] = self.extract_dataclass_parameter_information(clazz)

    @staticmethod
    def extract_dataclass_parameter_information(clazz) -> List[ParameterDigest]:
        """
        Extracts information about dataclass parameters from a dataclass.
        :param clazz: Class to parse
        :return: List of parameter information.
        """

        def is_optional_type(t):
            if get_origin(t) is Union:
                return type(None) in get_args(t)
            else:
                return False

        with open(inspect.getfile(clazz), 'r') as file:
            file_content = file.read()
        tree = ast.parse(file_content)
        class_param_comment: Dict = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == clazz.__name__:
                last_assign: List = []
                for item in node.body:
                    if isinstance(item, ast.AnnAssign):
                        last_assign = [item.target.id]
                        class_param_comment[item.target.id] = ""
                    elif isinstance(item, ast.Assign):
                        last_assign = list(map(lambda tar: tar.id, list(item.targets)))
                        class_param_comment = {**class_param_comment, **{var: "" for var in last_assign}}
                    elif last_assign and isinstance(item, ast.Expr):
                        class_param_comment = {**class_param_comment, **{var: item.value.s for var in last_assign}}
                        last_assign = []

        parameters_inspection = inspect.signature(clazz).parameters
        return [ParameterDigest(
            clazz=clazz.get_type_hints()[param],
            parameter_name=param,
            docstring_of_parameter_clazz=clazz.get_type_hints()[param].__doc__,
            docstring_of_parameter=class_param_comment[param],
            parameter_default_value=parameters_inspection[param].default,
            is_enum=clazz.get_type_hints()[param].__class__ == EnumMeta,
            is_optional=is_optional_type(clazz.get_type_hints()[param])
        ) for param in list(parameters_inspection.keys())]

def create_ontology_from_performables():
    def unwrap_classname(parameter: ParameterDigest) -> str:
        def extract_content_between_quotes(text: str) -> str:
            if match := re.search(r"'(.*?)'", text):
                return match.group(1)
            else:
                return text

        def remove_spaces(text: str) -> str:
            return text.replace(" ", "")
        
        def get_optional_type(t):
            optional_types = [arg for arg in get_args(t) if arg is not type(None)]
            if len(optional_types) > 1:
                print(f"Optional type has more than one type: {optional_types} (Type: {t})")
            return get_full_class_name(optional_types[0]) if len(optional_types) >= 1 else None

        clazz = parameter.clazz
        if parameter.is_optional:
            clazz = get_optional_type(parameter.clazz)
        return remove_spaces(extract_content_between_quotes(str(clazz)))

    classes = [ActionAbstractDigest(clazz) for clazz in recursive_subclasses(ActionAbstract)]

    output_ontology = get_ontology("performables")
    with output_ontology:
        class Performable(Thing): pass
        class Parameter(Thing): pass
        class Enum(Thing): pass
        class has_parameter(Performable >> Parameter): pass
        class has_default_value(DataProperty): pass
        class has_possible_value(Parameter >> Enum): pass
        class is_optional(Parameter >> bool): pass
        class has_description(DataProperty):
            range = [str]

    all_param_classes_to_ontological_class = {}
    for clazz in classes:
        for param in clazz.parameters:
            if (clazzname := unwrap_classname(param)) not in all_param_classes_to_ontological_class.keys():
                parameter_clazz = types.new_class(clazzname, (Parameter,))
                parameter_clazz.has_description = param.docstring_of_parameter_clazz
                if param.is_enum:
                    enum_value_class = types.new_class(clazzname + "_Value", (Enum,))
                    for enum_member in param.clazz.__members__:
                        enum_value_class(enum_member)
                    parameter_clazz.has_possible_value = [enum_value_class]
                all_param_classes_to_ontological_class[clazzname] = parameter_clazz

    for clazz_digest in classes:
        performable = Performable(clazz_digest.classname)
        performable.has_description = [clazz_digest.docstring]
        params = []
        for param in clazz_digest.parameters:
            param_instance = all_param_classes_to_ontological_class[unwrap_classname(param)](param.parameter_name)
            param_instance.has_description = [param.docstring_of_parameter]
            param_instance.is_optional = [True] if param.is_optional else [False]
            
            if param.get_default_value():
                param_instance.has_default_value = param.get_default_value()
            params.append(param_instance)
        performable.has_parameter = params
    
    output_ontology.save(file= "performables.owl", format="rdfxml")

create_ontology_from_performables()