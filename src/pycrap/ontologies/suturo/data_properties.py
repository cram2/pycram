from .dependencies import *
from .classes import *


class has_name_string(BaseProperty):
    """
    A relation recording some identifier associated to an Entity.
    """
    
class has_position_data(BaseProperty):
    """
    Associates a spatial region to a position.
    Associates a spatial region to a position.
    """
    
class handled(BaseProperty):
    ...
    
class has_data_value(BaseProperty):
    """
    A datatype property that encodes values from a datatype for an Entity. 
    There are several ways to encode values in DOLCE (Ultralite):
    
    1) Directly assert an xsd:_ value to an Entity by using hasDataValue
    2) Assert a Region for an Entity by using hasRegion, and then assert an xsd:_ value to that Region, by using hasRegionDataValue
    3) Assert a Quality for an Entity by using hasQuality, then assert a Region for that Quality, and assert an xsd:_ value to that Region, by using hasRegionDataValue
    4) When the value is required, but not directly observed, assert a Parameter for an xsd:_ value by using hasParameterDataValue, and then associate the Parameter to an Entity by using isConstraintFor
    5) When the value is required, but not directly observed, you can also assert a Parameter for a Region by using parametrizes, and then assert an xsd:_ value to that Region, by using hasRegionDataValue
    
    The five approaches obey different requirements. 
    For example, a simple value can be easily asserted by using pattern (1), but if one needs to assert an interval between two values, a Region should be introduced to materialize that interval, as pattern (2) suggests. 
    Furthermore, if one needs to distinguish the individual Quality of a value, e.g. the particular nature of the density of a substance, pattern (3) can be used. 
    Patterns (4) and (5) should be used instead when a constraint or a selection is modeled, independently from the actual observation of values in the real world.
    """
    
class has_confidence_value(BaseProperty):
    ...
    
class has_data_source(BaseProperty):
    """
    The source of information for this entity can vary. For instance, tables are derived from the semantic map, whereas the objects perceived are obtained from Perception.
    """
    
class has_grasp_pose(BaseDatatype):
    """
    The pose an object shall be grabbed from such as "above" or "side".
    """
    
class has_handle_state(BaseProperty):
    ...
    
class has_position(BaseProperty):
    ...
    
class has_predefined_name(BaseDatatype):
    """
    A predefined name refers to a name or label that is predetermined or established beforehand. It is typically assigned or chosen from a predefined set of names, often based on specific criteria or guidelines.
    """
    
class has_robocup_name(BaseProperty):
    """
    The robocup name of a furniture instance.
    
    TODO: make this more descriptive and generalized.
    """
    
