import ast

def extract_class_attribute_docstrings(file_path):
    with open(file_path, 'r') as file:
        file_content = file.read()
    tree = ast.parse(file_content)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            print(f"Class: {node.name}")
            for item in node.body:
                if isinstance(item, ast.AnnAssign):
                    print(f"{item.target.id} -> {item.annotation.id}")
                if isinstance(item, ast.Assign):
                    for target in item.targets:
                        print(f"{target.id}")
                if isinstance(item, ast.Expr):
                    print(f"Comment: {item.value.s}")



extract_class_attribute_docstrings('test.py')