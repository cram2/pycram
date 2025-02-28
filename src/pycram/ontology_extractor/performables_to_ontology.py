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
    classname: str
    docstring: str
    param_docstring: str
    default_value: Any

class ActionAbstractDigest:
    def __init__(self, clazz):
        self.clazz: Type[ActionAbstract] = clazz
        self.classname: str = get_full_class_name(clazz)
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
            classname=clazz.get_type_hints()[param].__class__.__name__,
            docstring=clazz.get_type_hints()[param].__doc__,
            param_docstring=class_param_comment[param],
            default_value=parameters_inspection[param].default
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

    all_params_classes = {param.classname: param.clazz for clazz in classes for param in clazz.parameters}
    parameter_cls_dict = {classname: types.new_class(classname, (Parameter,))
                          for classname in all_params_classes.keys()}
    for clazz in classes:
        performable = Performable(clazz)
        performable.has_description = [clazz.docstring]
        params = []
        for param in clazz.parameters:
            param_instance = parameter_cls_dict[param.classname]()
            param_instance.has_description = [param.param_docstring]
            params.append(param_instance)
            #TODO Hier werden noch nicht alle informationen aus einem Parameter richtig ausgelesen.
            # Der Docstring der Parameter-Klasse muss in jedem Fall noch dem Parameter-Classe
            # (der Ontologischen) hinterlegt werden.
        performable.has_parameter = params
    
    output_ontology.save(file= "performables.owl", format="rdfxml")

create_ontology_from_performables()