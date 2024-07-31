================
Custom Resolvers
================

Default (Py)CRAM designators are sometimes not capable of coming up with a feasible solution.
For that, one can write custom resolver, that fix these issues. Custom resolvers are located in the
:mod:`pycram.resolvers` package.

Resolvers should inherit from a designator iff they improve the performance of that designator. If it is a resolver that
does something on a higher level it does not need to earn from a specific designator.

The interface of custom resolvers should be consistent with the interface used in the same type of designators.

Tutorial
--------
A tutorial for custom resolver creation is found in the notebook below.

