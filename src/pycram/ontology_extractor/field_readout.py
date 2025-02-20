from pycram.designators.action_designator import OpenAction
from random_events.utils import get_full_class_name
import inspect

output = {}
c = OpenAction
output["classname"] = get_full_class_name(c)
output["doc"] = c.__doc__
output_param = {}
output["parameters"] = output_param
parameters = inspect.signature(c.__init__).parameters
for k in list(parameters.keys())[1:]:
    v = parameters[k]
    output_param[k] = {}
    output_param[k]["class"] = c.get_type_hints()[k]
    print("Doc: " + output_param[k]["class"].__doc__)
    output_param[k]["name"] = v.name
    output_param[k]["default_value"] = v.default
print(output)

#TODO Workaround for Union and typing classes
#TODO Readout the comments under parameters