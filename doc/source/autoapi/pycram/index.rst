:py:mod:`pycram`
================

.. py:module:: pycram

.. autoapi-nested-parse::

   Python3 implementation of CRAM.

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



Subpackages
-----------
.. toctree::
   :titlesonly:
   :maxdepth: 3

   datastructures/index.rst
   designators/index.rst
   external_interfaces/index.rst
   object_descriptors/index.rst
   ontology/index.rst
   orm/index.rst
   process_modules/index.rst
   robot_descriptions/index.rst
   ros/index.rst
   world_concepts/index.rst
   worlds/index.rst


Submodules
----------
.. toctree::
   :titlesonly:
   :maxdepth: 1

   cache_manager/index.rst
   costmaps/index.rst
   description/index.rst
   designator/index.rst
   failure_handling/index.rst
   fluent/index.rst
   helper/index.rst
   language/index.rst
   local_transformer/index.rst
   plan_failures/index.rst
   pose_generator_and_validator/index.rst
   process_module/index.rst
   robot_description/index.rst
   tasktree/index.rst
   utils/index.rst
   world_reasoning/index.rst


Package Contents
----------------

.. py:data:: ch

   

.. py:data:: formatter

   

