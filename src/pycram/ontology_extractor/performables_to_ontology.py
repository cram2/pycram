from enum import EnumMeta
from types import UnionType

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
    clazzname: str
    parameter_name: str
    docstring_of_parameter_clazz: str
    docstring_of_parameter: str
    parameter_default_value: Any
    is_enum: bool
    is_optional: bool

    def get_default_value(self):
        if not self.parameter_default_value == inspect.Parameter.empty:
            return [str(self.parameter_default_value)]
        else:
            return None

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
        :param clazz: Clazz to parse
        :return: List of parameter information.
        """

        def extract_content_between_quotes(text):
            if match := re.search(r"'(.*?)'", text):
                return match.group(1)
            else:
                return text

        def is_optional_type(t):
            if get_origin(t) is (Union or UnionType):
                return None in get_args(t)
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
            clazzname=extract_content_between_quotes(str(clazz.get_type_hints()[param])),
            parameter_name=param,
            docstring_of_parameter_clazz=clazz.get_type_hints()[param].__doc__,
            docstring_of_parameter=class_param_comment[param],
            parameter_default_value=parameters_inspection[param].default,
            is_enum=clazz.get_type_hints()[param].__class__ == EnumMeta,
            is_optional=is_optional_type(clazz.get_type_hints()[param])
        ) for param in list(parameters_inspection.keys())]

def create_ontology_from_performables():
    classes = [ActionAbstractDigest(clazz) for clazz in recursive_subclasses(ActionAbstract)]

    output_ontology = get_ontology("performables")
    with output_ontology:
        class Performable(Thing):
            pass

        class Parameter(Thing):
            pass

        class has_parameter(Performable >> Parameter):
            pass

        class has_description(DataProperty):
            range = [str]

        class has_default_value(DataProperty):
            pass

        class has_possible_value(Parameter >> DataProperty):
            pass

    all_param_classes_to_ontological_class = {}
    for clazz in classes:
        for parameter in clazz.parameters:
            if (clazzname := parameter.clazzname) not in all_param_classes_to_ontological_class.keys():
                parameter_clazz = types.new_class(clazzname, (Parameter,))
                parameter_clazz.has_description = parameter.docstring_of_parameter_clazz
                if parameter.is_enum:
                    parameter_clazz.has_possible_value = [key for key in parameter.clazz.__members__]
                all_param_classes_to_ontological_class[clazzname] = parameter_clazz

    for clazz_digest in classes:
        performable = Performable(clazz_digest.classname)
        performable.has_description = [clazz_digest.docstring]
        params = []
        for param in clazz_digest.parameters:
            param_instance = all_param_classes_to_ontological_class[param.clazzname](param.parameter_name)
            param_instance.has_description = [param.docstring_of_parameter]
            # TODO Here you must evaluate if the parameter is optional (on a per-instance basis)
            # Question: Do I have a link of the actual parameterDigest for per instance at hand still?!
            if param.get_default_value():
                param_instance.has_default_value = param.get_default_value()
            params.append(param_instance)
        performable.has_parameter = params
    
    output_ontology.save(file= "performables.owl", format="rdfxml")

create_ontology_from_performables()