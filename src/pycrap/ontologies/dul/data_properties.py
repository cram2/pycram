from .dependencies import *
from .classes import *


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
    
class has_region_data_value(BaseProperty):
    """
    A datatype property that encodes values for a Region, e.g. a float for the Region Height.
    """
    
class has_event_date(BaseProperty):
    """
    A datatype property that encodes values from xsd:dateTime for an Event; a same Event can have more than one xsd:dateTime value: begin date, end date, date at which the interval holds, etc.
    """
    
class has_interval_date(BaseProperty):
    """
    A datatype property that encodes values from xsd:dateTime for a TimeInterval; a same TimeInterval can have more than one xsd:dateTime value: begin date, end date, date at which the interval holds, etc.
    """
    
class has_parameter_data_value(BaseProperty):
    """
    Parametrizes values from a datatype. For example, a Parameter MinimumAgeForDriving hasParameterDataValue 18 on datatype xsd:int, in the Italian traffic code. In this example, MinimumAgeForDriving isDefinedIn the Norm ItalianTrafficCodeAgeDriving.
    More complex parametrization requires workarounds. E.g. AgeRangeForDrugUsage could parametrize data value: 14 to 50 on the datatype: xsd:int. Since complex datatypes are not allowed in OWL1.0, a solution to this can only work by creating two 'sub-parameters': MinimumAgeForDrugUsage (that hasParameterDataValue 14) and MaximumAgeForDrugUsage (that hasParameterDataValue 50), which are components of (cf. hasComponent) the main Parameter AgeRangeForDrugUsage.
    Ordering on subparameters can be created by using or specializing the object property 'precedes'.
    """
    
