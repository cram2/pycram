========
Examples
========

All types of examples

Object Relational Mapping
=========================

This section discusses the mapping to databases of pycram objects.
Learning from data is usually done by providing a collection propositional data points and fitting a model for the
proposition. However with the highly complex plans and structures of pycram a more sophisticated logging method is
needed. Hence pycram implements an object relational mapper (ORM) using
`sqlalchemy <https://docs.sqlalchemy.org/en/20/index.html#>`_.

The implementation of the ORM classes can be found in :meth:`pycram.orm`. This package contains all propositions and
their relations that can be logged from pycram.
Other classes should then implement a ``to_sql()`` and an ``insert(session, ...)`` with potential extra arguments.
The ``to_sql()`` method creates a proposition without relations from the ORM package.
The ``insert(session, ...)`` method should add the object from the orm method to the session and commit it, such that
the relations are reflected in the database and fields of the object. An example implementation of these methods can be
seen in :meth:`pycram.task.TaskTreeNode.to_sql` and :meth:`pycram.task.TaskTreeNode.insert`.

When using the ORM to record the experiments a MetaData instance is created. For a clean data management it is important
to fill out the description. For this, check the documentation of :meth:`pycram.orm.base.ProcessMetaData`.

ORM Examples
------------
