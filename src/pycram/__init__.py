"""Python3 implementation of CRAM.

To use macros one must put the code in an own file and create a second file (the launcher) which activates MacroPy and then imports the file where the macros are used.
E. g. if you have a file target.py which contains your code, create a file run.py:

#!/usr/bin/env python

import macropy.activate
import target

Now launch run.py to start your program.

Modules:
designator -- implementation of designators.
fluent -- implementation of fluents and the whenever macro.
helper -- implementation of helper classes and functions for internal usage only.
language -- implementation of the CRAM language.
process_module -- implementation of process modules.
"""