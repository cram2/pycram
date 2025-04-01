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


class contains(BaseProperty):
    """
    A schematic relation asserting containment, understood in a very broad sense, by one Entity of another.
    The relation is defined with domain and range of maximum generality, as it is possible to construe containment to
    apply between Events, between Objects, between Qualities and so on. Care should be taken when using it that the
    construal of containment makes sense and is useful. If a clearer relation expresses the connection between two
    Entities, use that relation instead. For example, rather than saying an Event contains an Object, it is probably
    better to say the Event has that Object as a participant. More specific versions of this relation exist,
    e.g. containsEvent, so it is likely that there will be few situations where it should be used itself.
    However, by acting as a superproperty to several relations, it captures a core similarity between these and enables
    taxonomy-based similarity metrics.
    """


class contains_object(BaseProperty):
    """
    A spatial relation holding between a container, and objects it contains.
    """


class is_contained_in(BaseProperty):
    """
    The inverse of the contains relation. See the contains relation for details.
    """

class has_preferred_alignment(BaseProperty):
    """
    A relation between an object and an alignment that is preferred for that object.
    """

class has_preferred_axis(BaseProperty):
    """
    A relation between an object and an axis identifier.
    """

class has_vertical_alignment(BaseProperty):
    """
    A relation between an object and boolean value indicating if the object should be grasped with a vertical alignment.
    """

class has_rotated_gripper(BaseProperty):
    """
    A relation between an object and boolean value indicating if the gripper should be rotated by 90°.
    """

class has_rim_grasp(BaseProperty):
    """
    A relation between an object and a boolean value indicating if the object should be grasped by the rim.
    """


class is_physically_contained_in(BaseProperty):
    """
    A spatial relation holding between an object (the container), and objects it contains.
    """

class supports(BaseProperty):
    """
    A relation between an object (the supporter) and another object (the supportee) where the supporter cancels the
     effect of gravity on the supportee.
    """

class is_supported_by(BaseProperty):
    """
    A relation between an object (the supporter) and another object (the supportee) where the supporter cancels the
     effect of gravity on the supportee. The inverse of the supports relation.
    """