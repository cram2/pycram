import logging
import os
from dataclasses import is_dataclass

import semantic_digital_twin.orm.ormatic_interface
from krrood.class_diagrams import ClassDiagram
from krrood.ormatic.ormatic import ORMatic
from krrood.ormatic.utils import get_classes_of_ormatic_interface, classes_of_module
from krrood.utils import recursive_subclasses
from semantic_digital_twin.world import WorldModelManager
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.world_modification import (
    WorldModelModificationBlock,
    WorldModelModification,
)

import pycram.datastructures.pose
import pycram.language
from pycram.datastructures import grasp
from pycram.language import SequentialNode, RepeatNode, LanguageNode
from pycram.orm.model import *
from pycram.robot_plans.actions.composite import (
    facing,
    searching,
    tool_based,
    transporting,
)
from pycram.robot_plans.actions.core import (
    container,
    pick_up,
    misc,
    navigation,
    placing,
    robot_body,
)
from pycram.robot_plans.motions import BaseMotion
from pycram.robot_plans.motions import (
    container as motion_container,
    gripper as motion_gripper,
    navigation as motion_navigation,
    misc as motion_misc,
    robot_body as motion_robot_body,
)

# ----------------------------------------------------------------------------------------------------------------------
# This script generates the ORM classes for the pycram package
# Classes that are self_mapped and explicitly_mapped are already mapped in the model.py file. Look there for more
# information on how to map them.
# ----------------------------------------------------------------------------------------------------------------------

# import classes from the existing interface
classes, alternative_mappings, type_mappings = get_classes_of_ormatic_interface(
    semantic_digital_twin.orm.ormatic_interface
)

classes = set(classes)

# create of classes that should be mapped
classes |= set(classes_of_module(pycram.datastructures.pose))
classes |= {ExecutionData}
# classes |= set(classes_of_module(action_designator)) | {ActionDescription}
classes |= set(classes_of_module(facing))
classes |= set(classes_of_module(searching))
classes |= set(classes_of_module(tool_based))
classes |= set(classes_of_module(transporting))
classes |= set(classes_of_module(container))
classes |= set(classes_of_module(pick_up))
classes |= set(classes_of_module(misc))
classes |= set(classes_of_module(navigation))
classes |= set(classes_of_module(placing))
classes |= set(classes_of_module(robot_body))  # | {ActionDescription}
classes |= {ActionDescription}
classes |= {DesignatorDescription}
classes |= {BaseMotion}
classes |= set(classes_of_module(grasp))
classes |= {WorldModelModificationBlock, WorldModelModification}

# Semantic World Classes
classes |= {Body}

# Motion Designator
classes |= set(classes_of_module(motion_gripper))
classes |= set(classes_of_module(motion_navigation))
classes |= set(classes_of_module(motion_container))
classes |= set(classes_of_module(motion_misc))
classes |= set(classes_of_module(motion_robot_body))

classes |= {
    PlanNode,
    SequentialNode,
    RepeatNode,
    ResolvedActionNode,
    Plan,
    PlanEdge,
    LanguageNode,
}
classes |= set(classes_of_module(pycram.language))
classes -= {WorldModelManager}

alternative_mappings += [am for am in recursive_subclasses(AlternativeMapping)]
alternative_mappings = list(set(alternative_mappings))
# keep only dataclasses that are NOT AlternativeMapping subclasses
classes = {
    c for c in classes if is_dataclass(c) and not issubclass(c, AlternativeMapping)
}
classes |= {am.original_class() for am in recursive_subclasses(AlternativeMapping)}

alternative_mappings = [
    am
    for am in recursive_subclasses(AlternativeMapping)
    if am.original_class() in classes
]

# create the new ormatic interface
class_diagram = ClassDiagram(
    list(sorted(classes, key=lambda c: c.__name__, reverse=True))
)

type_mappings.update({np.ndarray: NumpyType})


def generate_orm():
    """
    Generate the ORM classes for the pycram package.
    """
    # Create an ORMatic object with the classes to be mapped
    ormatic = ORMatic(
        class_diagram,
        type_mappings=type_mappings,
        alternative_mappings=alternative_mappings,
    )
    logging.getLogger("krrood").setLevel(logging.DEBUG)

    # Generate the ORM classes
    ormatic.make_all_tables()

    path = os.path.abspath(os.path.join(os.getcwd(), "../src/pycram/orm/"))
    with open(os.path.join(path, "ormatic_interface.py"), "w") as f:
        ormatic.to_sqlalchemy_file(f)


if __name__ == "__main__":
    generate_orm()
