import logging
import os
import sys
from enum import Enum

from ormatic.dao import AlternativeMapping
from ormatic.ormatic import logger, ORMatic
from ormatic.utils import recursive_subclasses, classes_of_module
from pycram.robot_plans.actions.core import container, grasping, misc, navigation, placing, robot_body
from sqlacodegen.generators import TablesGenerator
from sqlalchemy import create_engine
from sqlalchemy.orm import registry, Session
import pycram.datastructures.pose
from pycram.datastructures import grasp
from pycram.language import SequentialNode
from pycram.orm.casts import StringType
from pycram.robot_plans.actions.composite import facing, searching, tool_based, transporting
from pycrap.ontologies import PhysicalObject
from pycram.orm.model import *

# ----------------------------------------------------------------------------------------------------------------------
# This script generates the ORM classes for the pycram package
# Classes that are self_mapped and explicitly_mapped are already mapped in the model.py file. Look there for more
# information on how to map them.
# ----------------------------------------------------------------------------------------------------------------------

# create of classes that should be mapped
classes = set(recursive_subclasses(AlternativeMapping))
classes |= set(classes_of_module(pycram.datastructures.pose))
# classes |= set(classes_of_module(action_designator)) | {ActionDescription}
classes |= set(classes_of_module(facing))
classes |= set(classes_of_module(searching))
classes |= set(classes_of_module(tool_based))
classes |= set(classes_of_module(transporting))
classes |= set(classes_of_module(container))
classes |= set(classes_of_module(grasping))
classes |= set(classes_of_module(misc))
classes |= set(classes_of_module(navigation))
classes |= set(classes_of_module(placing))
classes |= set(classes_of_module(robot_body)) | {ActionDescription}
classes |= set(classes_of_module(grasp))
classes |= {PlanNode, SequentialNode, RepeatNode}

# remove classes that should not be mapped
classes -= set(recursive_subclasses(Enum))

# specify custom type mappings
type_mappings = {
    PhysicalObject: StringType(),
}


def generate_orm():
    """
    Generate the ORM classes for the pycram package.
    """
    # Set up logging
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))

    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    mapper_registry = registry()
    engine = create_engine('sqlite:///:memory:')
    session = Session(engine)

    # Create an ORMatic object with the classes to be mapped
    ormatic = ORMatic(list(classes))

    # Generate the ORM classes
    ormatic.make_all_tables()

    # Create the tables in the database
    mapper_registry.metadata.create_all(session.bind)

    path = os.path.abspath(os.path.join(os.getcwd(), '../src/pycram/orm/'))
    with open(os.path.join(path, 'ormatic_interface.py'), 'w') as f:
        ormatic.to_sqlalchemy_file(f)


if __name__ == '__main__':
    generate_orm()
