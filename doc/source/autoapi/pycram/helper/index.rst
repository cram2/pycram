:py:mod:`pycram.helper`
=======================

.. py:module:: pycram.helper

.. autoapi-nested-parse::

   Implementation of helper functions and classes for internal usage only.

   Classes:
   Singleton -- implementation of singleton metaclass



Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.helper.Singleton




.. py:class:: Singleton


   Bases: :py:obj:`type`

   Metaclass for singletons

   .. py:attribute:: _instances

      Dictionary of singleton child classes inheriting from this metaclass, keyed by child class objects.

   .. py:method:: __call__(*args, **kwargs)

      Call self as a function.



