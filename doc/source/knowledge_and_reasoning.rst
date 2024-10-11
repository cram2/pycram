=================================
Knowledge and Reasoning in PyCRAM
=================================

The knowledge engine is able to infer parameters of a designator description from the context given by the properties
attached to its parameters. Since the properties are defined for the parameters of a designator description they add
semantic information to the designator description parameters. The knowledge engine is able to utilize this information
to infer the value of a parameter from the context.

Inference is done very similar to the normal reasoning process where the property function of the designator description
is first resolved and then evaluated. The difference is that we now not only look at the result (if the properties are
satisfied or not) but also a the possible parameter solutions that are generated while reasoning.

We start again by taking the properties of of the designator description and resolve them.

.. image:: ../images/knowledge/property_resolve.png
    :alt: Source Resolve
    :align: center

We then evaluate the properties and generate the possible parameter solutions.

.. image:: ../images/knowledge/property_evaluation.png
    :alt: Source Evaluate
    :align: center