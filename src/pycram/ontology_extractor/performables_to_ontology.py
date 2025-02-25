from pycram.designator import ActionDesignatorDescription
from pycram.designators.action_designator import ActionAbstract
from random_events.utils import recursive_subclasses, get_full_class_name
import inspect
from owlready2 import *
from field_readout import parse

def parse_class_structure():
    clazzes = {}
    all_param_clazzes = set()
    # TODO: Doesnt work with ActionDesignatorDescription as the root of the class tree to be parsed.
    for clazz in recursive_subclasses(ActionAbstract):
        parse_result = parse(clazz)
        all_param_clazzes.update(set(map(lambda param: param["class"], parse_result["parameters"].values())))

    output_ontology = get_ontology("performables")
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

parse_class_structure()