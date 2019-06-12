import pybullet as p
import ast
import threading
from pycram.language import _block
from macropy.core.macros import Macros
from macropy.core.hquotes import macros, hq
from macropy.core.quotes import macros, ast_literal, q

macros = Macros()

@macros.block
def simulated_robot(tree, target, args, **kw):
    print("test")
    thread1 = gui()
    thread1.start()
    with hq as new_tree:
        ast_literal[tree]
    return  new_tree

def _set_target(target, value):
    target_laod = ast.Name(target.id, ast.Load())

    with hq as tree:
        ast_literal[target_laod].set_value(value)

    return tree

class gui(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.id = -1

    def run(self):
        self.id = p.connect(p.GUI)
        while 1:
            a = 1


