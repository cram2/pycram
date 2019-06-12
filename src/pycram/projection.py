import pybullet as p
from pycram.language import _block
from macropy.core.macros import Macros
from macropy.core.hquotes import  macros, hq
from macropy.core.quotes import macros, ast_literal, q

macros = Macros()

@Macros.block
def simulated_robot(tree, target, args):
    with hq as new_tree:
        id = p.connect(p.GUI)
        while True:
            ast_literal[tree]

        new_tree.append(_set_target(target, id))

    return _block(new_tree)

def _set_target(target, value):
    target_laod = ast.Name(target.id, ast.Load())

    with hq as tree:
        ast_literal[target_laod].set_value(value)

    return tree
