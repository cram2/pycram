import pybullet as p
import ast
import threading
from pycram.fluent import Fluent
from pycram.language import _block
from macropy.core.macros import Macros
from macropy.core.hquotes import macros, hq
from macropy.core.quotes import macros, ast_literal, q

macros = Macros()

@macros.block
def simulated_robot(tree, target, args, **kw):
    thread1 = gui(Fluent(name="clien_id"))
    thread1.start()

    new_tree = _init(target)

    with hq as tmp_tree:
        ast_literal[tree]

    client = thread1.client.get_value()
    new_tree.append(tmp_tree)
    new_tree.append(_set_target(target, client))
    print(thread1.get_client().get_value())
    return _block(new_tree)

def _init(target):
    with hq as tree:
        ast_literal[target] = Fluent()

    return tree

def _set_target(target, value):
    target_load = ast.Name(target.id, ast.Load())

    with hq as tree:
        ast_literal[target_load].set_value(value)

    return tree

class gui(threading.Thread):
    def __init__(self, fluent):
        threading.Thread.__init__(self)
        self.client = fluent

    def get_client(self):
        return self.client

    def run(self):
        client = p.connect(p.GUI)
        self.client.set_value(client)
        #print(self.client.get_value())
        while 1:
            a = 1



