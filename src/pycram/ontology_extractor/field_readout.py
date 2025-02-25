from random_events.utils import get_full_class_name
from typing import Dict
import inspect
import ast

def _extract_dataclass_attribute_comments(clazz):
    """
    Parses the given Python file and returns a dictionary that contains the defined classes,
    their attributes and the comment that is given below the attribute.
    """
    file_path = inspect.getfile(clazz)
    with open(file_path, 'r') as file:
        file_content = file.read()
    tree = ast.parse(file_content)
    class_var_comment: Dict= {}
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            classname = node.name
            class_var_comment[classname] = {}
            last_assign = []
            for item in node.body:
                if isinstance(item, ast.AnnAssign):
                    last_assign = [item.target.id]
                    class_var_comment[classname][item.target.id] = ""
                elif isinstance(item, ast.Assign):
                    last_assign = list(map(lambda tar : tar.id, list(item.targets)))
                    class_var_comment[classname] = {**class_var_comment[classname],
                                                    **{var: "" for var in last_assign}}
                elif last_assign and isinstance(item, ast.Expr):
                    class_var_comment[classname] = {**class_var_comment[classname],
                                                    **{var: item.value.s for var in last_assign}}
                    last_assign = []
    return class_var_comment[clazz.__name__]

def parse(clazz):
    output = {"classname": get_full_class_name(clazz), "doc": clazz.__doc__, "parameters": {}}
    parameter_docstring = _extract_dataclass_attribute_comments(clazz)
    parameters = inspect.signature(clazz.__init__).parameters
    for param in list(parameters.keys())[1:]:
        output["parameters"][param] = {
        "class": clazz.get_type_hints()[param],
        "classname": clazz.get_type_hints()[param].__class__.__name__,
        "docstring": clazz.get_type_hints()[param].__doc__,
        "param_docstring": parameter_docstring[param],
        "default_value": parameters[param].default
        }
    return output