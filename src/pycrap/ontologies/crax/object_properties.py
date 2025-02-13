from .dependencies import *


class is_part_of(BaseProperty):
    """
    A relation between any entities, e.g. 'brain is a part of the human body'. See dul:hasPart for additional documentation.
    """


class has_part(BaseProperty):
    """
    A schematic relation between any entities, e.g. 'the human body has a brain as part', '20th century contains year 1923', 'World War II includes the Pearl Harbour event'.

    Parthood should assume the basic properties of mereology: transitivity, antisymmetry, and reflexivity (propert Parthood of course misses reflexivity).
    However, antisymmetry is not supported in OWL2 explicitly, therefore DUL has to adopt one of two patterns:
    1) dropping asymmetry axioms, while granting reflexivity: this means that symmetry is not enforced, but permitted for the case of reflexivity. Of course, in this way we cannot prevent symmetric usages of hasPart;
    2) dropping the reflexivity axiom, and enforce asymmetry: in this case, we would prevent all symmetric usages, but we loose the possibility of enforcing reflexivity, which is commonsensical in parthood.
    In DUL, we adopt pattern #1 for partOf, and pattern #2 for properPartOf, which seems a good approximation: due to the lack of inheritance of property characteristics, each asymmetric hasPropertPart assertion would also be a reflexive hasPart assertion (reflexive reduction design pattern).

    Subproperties and restrictions can be used to specialize hasPart for objects, events, etc.
    """

class has_parent_link(BaseProperty):
    """
    Relates a joint to the link it connects which is closer to the root of the kinematic chain.
    """

class has_child_link(BaseProperty):
    """
    Relates a joint to the link it connects which is closer to the end of the kinematic chain.
    """