from pycram.language import _block
from macropy.core.macros import Macros
from macropy.core.hquotes import macros, hq
from macropy.core.quotes import macros, ast_literal, q

macros = Macros()

@macros.block
def simulated_robot(tree, target, args, **kw):
    with hq as tmp_tree:
        ast_literal[tree]

    return _block(tmp_tree)





