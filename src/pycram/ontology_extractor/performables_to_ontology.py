from pycram.designator import ActionDesignatorDescription
from pycram.designators.action_designator import ActionAbstract
from random_events.utils import recursive_subclasses, get_full_class_name
import inspect
from owlready2 import *

def parse_class_structure():
    clazzes = {}
    all_param_clazzes = set()
    for clazz in recursive_subclasses(ActionDesignatorDescription):
        name = get_full_class_name(clazz)
        clazzes[name] = {}
        docstring = clazz.__doc__
        clazzes[name]['doc'] = docstring
        clazzes[name]['parameters'] = {}
        for init_param in inspect.signature(clazz.__init__).parameters.values():
            param_name = init_param.name
            if not param_name == 'self':
                clazzes[name]['parameters'][param_name] = init_param.annotation
                all_param_clazzes.add(init_param.annotation)

    output_ontology = get_ontology("Performable")
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
                              for parameter_cls in all_param_clazzes}

        for clazzname, clazzinfos in clazzes.items():
            performable = Performable(clazzname)
            performable.has_description = [clazzinfos['doc']]
            params = []
            for para_name, para_cls in clazzinfos['parameters'].items():
                parameter = parameter_cls_dict[para_cls]()
                parameter.has_description = para_name
                params.append(parameter)
            performable.has_parameter = params

    output_ontology.save(file= "performables.owl", format="rdfxml")