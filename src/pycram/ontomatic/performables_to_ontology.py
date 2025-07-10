from enum import EnumMeta
from owlready2 import get_ontology, Thing, DataProperty, types
from random_events.utils import recursive_subclasses
from random_events.utils import get_full_class_name
from typing_extensions import Dict, List, Type, Optional, Any, get_origin, Union, get_args
from dataclasses import dataclass
from pathlib import Path
import inspect
import ast
import os
import re

from ..robot_plans.actions.base import ActionDescription


@dataclass
class ParameterDigest:
    """
    Encapsulation of meta information about a parameter.
    """
    clazz: Any
    """
    Class of the parameter.
    """
    name: str
    """
    Name of the parameter.
    """
    docstring: str
    """
    Docstring of the parameter itself (individual to each performable).
    """
    default_value: Optional[Any]
    """
    Holds the default value of the parameter if set.
    """

    @property
    def docstring_of_clazz(self) -> str: return self.clazz.__doc__

    @property
    def is_enum(self) -> bool: return self.clazz.__class__ == EnumMeta

    @property
    def is_optional(self) -> bool: return (type(None) in get_args(self.clazz)) if get_origin(
        self.clazz) is Union else False

    def get_default_value(self) -> Optional[List[str]]:
        """
        :return: A list containing the string representation of the default value or
            `None` if no default value exists.
        """
        return None if self.default_value == inspect.Parameter.empty else [str(self.default_value)]


class ActionAbstractDigest:
    """
    Wrap all information about an action abstract class that are necessary for the created ontology.
    """

    def __init__(self, clazz: Type[ActionDescription]):
        self.clazz: Type[ActionDescription] = clazz
        self.full_name: str = get_full_class_name(clazz)
        self.classname: str = clazz.__name__
        self.docstring: str = clazz.__doc__ or ""
        self.parameters: Optional[List[ParameterDigest]] = self.extract_dataclass_parameter_information()

    def extract_dataclass_parameter_information(self) -> List[ParameterDigest]:
        """
        Extract information about dataclass parameters from a dataclass.

        :return: List of parameter information.
        """

        with open(inspect.getfile(self.clazz), 'r') as file:
            file_content = file.read()
        tree = ast.parse(file_content)
        class_param_comment: Dict = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == self.clazz.__name__:
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

        parameters_inspection = inspect.signature(self.clazz).parameters
        parameter_digests = []
        for param in list(parameters_inspection.keys()):
            param_clazz = self.clazz.get_type_hints()[param]
            parameter_digests.append(
                ParameterDigest(
                    clazz=param_clazz,
                    name=param,
                    docstring=class_param_comment[param] if param in class_param_comment.keys() else "",
                    default_value=parameters_inspection[param].default,
                ))
        return parameter_digests


def create_ontology_from_performables(
        output_path: Path = "./performables.owl",
        abstract_actions_to_parse: Union[List[Type[ActionDescription]], Type[ActionDescription]] = None) -> None:
    """
    Create an ontology from the performables.

    :param output_path: Path of the output ontology file.
    :param abstract_actions_to_parse: ActionAbstract classes to parse.
    If not set, all subclasses of ActionAbstract will be parsed.
    """

    def unwrap_classname(parameter: ParameterDigest) -> str:
        """
        Unwrap the class name of the type of parameter.
        When it is an optional type, the class name of the optional type is returned.

        :param parameter: ParameterDigest to return the unwrapped classname from.
        :return: Unwrapped classname of the parameter.
        """

        def extract_content_between_quotes(text: str) -> str:
            """
            :param text: String with quotation marks.
            :return: Text without quotation marks.
            """
            if match := re.search(r"'(.*?)'", text):
                return match.group(1)
            else:
                return text

        def get_optional_type(t) -> Optional[str]:
            """
            :param t: Type hint to check.
            :return: Full class name of the optional type or None if not optional.
            """
            optional_types = [arg for arg in get_args(t) if arg is not type(None)]
            if len(optional_types) > 1:
                print(f"Optional type has more than one type: {optional_types} (Type: {t})")
            return get_full_class_name(optional_types[0]) if len(optional_types) >= 1 else None

        clazz = parameter.clazz
        if parameter.is_optional:
            clazz = get_optional_type(parameter.clazz)
        return extract_content_between_quotes(str(clazz)).replace(" ", "")

    # If parameter is not set, all subclasses of ActionAbstract will be parsed.

    if abstract_actions_to_parse:
        try:
            iter(abstract_actions_to_parse)
        except TypeError:
            abstract_actions_to_parse = [abstract_actions_to_parse]
    else:
        abstract_actions_to_parse = recursive_subclasses(ActionDescription)

    classes = [ActionAbstractDigest(clazz) for clazz in abstract_actions_to_parse]

    # Definition of created ontology
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

    def create_parameter_onto_class(param: ParameterDigest) -> Parameter:
        """
        Create the ontology classes for the parameter classes.

        :param param: ParameterDigest to create the ontology classes for.
        """

        def create_enum_onto_class_and_instances() -> None:
            """
            Create ontology classes for enums and instances for possible values of enums.
            """
            enum_value_class = types.new_class(classname + "_Value", (Enum,))
            parameter_clazz.has_possible_value = [enum_value_class]
            # Create the possible values of the enum as instances of the Enum class
            for enum_member in param.clazz.__members__:
                enum_value_class(enum_member)

        parameter_clazz = types.new_class(classname, (Parameter,))
        parameter_clazz.has_description = param.docstring_of_clazz
        if param.is_enum:
            create_enum_onto_class_and_instances()
        return parameter_clazz

    def create_performable_onto_class() -> None:
        """
        Create an instance of the ontology class Performable for each ActionAbstract and all the relations to its
        ontological parameter classes.
        """

        def create_param_onto_instances() -> List[Parameter]:
            """
            Create ontology instances for the parameters of the Performable.
            """
            params = []
            for param_digest in clazz_digest.parameters:
                param_instance = all_param_classes_to_ontological_class[unwrap_classname(param_digest)](
                    param_digest.name)
                param_instance.has_description = [param_digest.docstring]
                param_instance.is_optional = [True] if param_digest.is_optional else [False]
                if param_digest.get_default_value():
                    param_instance.has_default_value = param_digest.get_default_value()
                params.append(param_instance)
            return params

        performable = Performable(clazz_digest.classname)
        performable.has_description = [clazz_digest.docstring]
        performable.has_parameter = create_param_onto_instances()

    # Creation of ontological classes/instances for the parsed Python classes.
    all_param_classes_to_ontological_class = {}
    for clazz_digest in classes:
        for param in clazz_digest.parameters:
            if (classname := unwrap_classname(param)) not in all_param_classes_to_ontological_class.keys():
                all_param_classes_to_ontological_class[classname] = create_parameter_onto_class(param)

    # Creating the ontology instances based on the created ActionAbstractDigests.
    for clazz_digest in classes:
        create_performable_onto_class()

    (dirname, filename) = os.path.split(output_path)
    os.makedirs(dirname, exist_ok=True)
    output_ontology.save(file=output_path, format="rdfxml")
