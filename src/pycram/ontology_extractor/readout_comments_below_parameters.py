import ast
from typing import Dict
from pathlib import Path


def extract_dataclass_attribute_comments(file_path: Path):
    """
    Parses the given Python file and returns a dictionary that contains the defined classes,
    their attributes and the comment that is given below the attribute.
    """
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
    return class_var_comment


print(extract_dataclass_attribute_comments(Path('test.py')))