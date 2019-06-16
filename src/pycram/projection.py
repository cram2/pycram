import pybullet as p
import ast
import threading
from pycram.bullet_world import BulletWorld
from pycram.language import _block
from macropy.core.macros import Macros
from macropy.core.hquotes import macros, hq
from macropy.core.quotes import macros, ast_literal, q

macros = Macros()

@macros.block
def simulated_robot(tree, target, args, **kw):
    world = BulletWorld()
    thread1 = gui(world)
    thread1.start()



    with hq as tmp_tree:
        ast_literal[tree]

    tmp_tree.append(_set_target(target, world))
    return _block(tmp_tree)


def _set_target(target, value):
    target_load = ast.Name(target.id, ast.Load())

    with hq as tree:
        ast_literal[target_load].set_value(value)

    return tree




