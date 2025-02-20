from pycram.designator import ActionDesignatorDescription
from pycram.designators.action_designator import ActionAbstract
from random_events.utils import recursive_subclasses, get_full_class_name
import inspect
from owlready2 import *

action_performable_classes = {}
parameter_classes = set()
for c in recursive_subclasses(ActionDesignatorDescription):
    name = get_full_class_name(c)
    action_performable_classes[name] = {}
    performable_comment = c.__doc__
    action_performable_classes[name]['comment'] = performable_comment
    action_performable_classes[name]['parameters'] = {}
    for parameter in inspect.signature(c.__init__).parameters.values():
        if parameter.name == 'self':
            continue
        action_performable_classes[name]['parameters'][parameter.name] = parameter.annotation
        parameter_classes.add(parameter.annotation)
# Create ontology from assessed python structures
output_ontology = get_ontology("pycram_performables.owl")
with output_ontology:
    class Performable(Thing):
        pass

    class Parameter(Thing):
        pass

    class has_parameter(Performable >> Parameter):
        pass

    class has_description(DataProperty):
        range = [str]

    parameter_cls_dict = {parameter_cls: types.new_class(parameter_cls, (Parameter,))
                          for parameter_cls in parameter_classes}

    for perf_name, perf_details in action_performable_classes.items():
        perf_ins = Performable(perf_name)
        perf_ins.has_description = [perf_details['comment']]
        para_instances = []
        for para_name, para_cls in perf_details['parameters'].items():
            para_ins = parameter_cls_dict[para_cls]()
            para_ins.has_description = [para_name]
            para_instances.append(para_ins)
        perf_ins.has_parameter = para_instances

output_ontology.save(file= "performables.owl", format="rdfxml")