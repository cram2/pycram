from .dependencies import *


class has_physical_component(BaseProperty):
    """
    A relation used to describe the structure of a PhysicalObject in terms of physical components, i.e. what other PhysicalObjects are components of it.
    """
    
class has_constituent(BaseProperty):
    """
    'Constituency' depends on some layering of  the world described by the ontology. For example, scientific granularities (e.g. body-organ-tissue-cell) or ontological 'strata' (e.g. social-mental-biological-physical) are  typical layerings. 
    Intuitively, a constituent is a part belonging to a lower layer. Since layering is actually a partition of the world described by the ontology, constituents are not properly classified as parts, although this kinship can be intuitive for common sense.
    A desirable advantage of this distinction is that we are able to talk e.g. of physical constituents of non-physical objects (e.g. systems), while this is not possible in terms of parts.
    Example of are the persons constituting a social system, the molecules constituting a person, the atoms constituting a river, etc. 
    In all these examples, we notice a typical discontinuity between the constituted and the constituent object: e.g. a social system is conceptualized at a different layer from the persons that constitute it, a person is conceptualized at a different layer from the molecules that constitute them, and a river is conceptualized at a different layer from the atoms that constitute it.
    """
    
class has_region(BaseProperty):
    """
    A relation between entities and regions, e.g. 'the number of wheels of that truck is 12', 'the time of the experiment is August 9th, 2004', 'the whale has been localized at 34 degrees E, 20 degrees S'.
    """
    
class has_quality(BaseProperty):
    """
    A relation between entities and qualities, e.g. 'Dmitri's skin is yellowish'.
    """
    
class is_region_for(BaseProperty):
    """
    A relation between entities and regions, e.g. 'the color of my car is red'.
    """
    
class is_quality_of(BaseProperty):
    """
    A relation between entities and qualities, e.g. 'Dmitri's skin is yellowish'.
    """
    
class has_base_link(BaseProperty):
    """
    Relates a physical object to the unique base link(=the first link in kinematic chain) of its kinematic chain.
    """
    
class has_root_link(BaseProperty):
    ...
    
class has_child_link(BaseProperty):
    ...
    
class has_collision_shape(BaseProperty):
    ...
    
class has_damping_attribute(BaseProperty):
    ...
    
class has_end_link(BaseProperty):
    """
    Relates a physical object to the end link (= the last link in kinematic chain) of its kinematic chain, there may be several end links (e.g. fingers of a hand).
    """
    
class has_link(BaseProperty):
    ...
    
class has_inertia(BaseProperty):
    ...
    
class has_joint(BaseProperty):
    ...
    
class has_joint_axis(BaseProperty):
    ...
    
class has_joint_limits(BaseProperty):
    ...
    
class has_joint_position(BaseProperty):
    ...
    
class has_joint_reference_positions(BaseProperty):
    ...
    
class has_joint_soft_limits(BaseProperty):
    ...
    
class has_origin(BaseProperty):
    ...
    
class has_parent_link(BaseProperty):
    ...
    
