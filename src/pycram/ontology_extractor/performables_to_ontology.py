from typing_extensions import Self

from pycram.designators.action_designator import ActionAbstract
from random_events.utils import recursive_subclasses
from owlready2 import *
from random_events.utils import get_full_class_name
from typing import Dict, List, Type, Optional, Any
from dataclasses import dataclass
import inspect
import ast

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

    def __str__(self):
        return str(super.__str__(self)) + "\n"

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

        parameters_inspection = inspect.signature(clazz.__init__).parameters
        return [ParameterDigest(
            clazz=clazz.get_type_hints()[param],
            clazzname=str(clazz.get_type_hints()[param].__class__.__name__),
            parameter_name=param,
            docstring_of_parameter_clazz=clazz.get_type_hints()[param].__doc__,
            docstring_of_parameter=class_param_comment[param],
            parameter_default_value=parameters_inspection[param].default
        ) for param in list(parameters_inspection.keys())[1:]]

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

    all_param_classes_to_ontological_class = {}
    for clazz in classes:
        for parameter in clazz.parameters:
            if (clazzname := parameter.clazzname) not in all_param_classes_to_ontological_class.keys():
                all_param_classes_to_ontological_class[clazzname] = types.new_class(clazzname, (Parameter,))
                all_param_classes_to_ontological_class[clazzname].has_description = parameter.docstring_of_parameter_clazz

    for clazz_digest in classes:
        performable = Performable(clazz_digest.classname)
        performable.has_description = [clazz_digest.docstring]
        params = []
        for param in clazz_digest.parameters:
            param_instance = all_param_classes_to_ontological_class[param.clazzname](param.parameter_name)
            param_instance.has_description = [param.docstring_of_parameter]
            if param.get_default_value():
                param_instance.has_default_value = param.get_default_value()
            params.append(param_instance)
        performable.has_parameter = params
    
    output_ontology.save(file= "performables.owl", format="rdfxml")

create_ontology_from_performables()