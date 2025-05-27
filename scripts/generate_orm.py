import logging
import os
import sys
from enum import Enum

from ormatic.ormatic import logger, ORMatic
from ormatic.utils import recursive_subclasses, classes_of_module, ORMaticExplicitMapping
from sqlacodegen.generators import TablesGenerator
from sqlalchemy import create_engine
from sqlalchemy.orm import registry, Session
import pycram.datastructures.pose
from pycram.datastructures import grasp
from pycram.designator import ActionDescription
from pycram.designators import action_designator
from pycram.orm.casts import StringType
from pycrap.ontologies import PhysicalObject
from pycram.orm.model import *

# ----------------------------------------------------------------------------------------------------------------------
# This script generates the ORM classes for the pycram package
# Classes that are self_mapped and explicitly_mapped are already mapped in the model.py file. Look there for more
# information on how to map them.
# ----------------------------------------------------------------------------------------------------------------------

# create of classes that should be mapped
classes = set(recursive_subclasses(ORMaticExplicitMapping))
classes |= set(classes_of_module(pycram.datastructures.pose))
classes |= set(classes_of_module(action_designator)) | {ActionDescription}
classes |= set(classes_of_module(grasp))
classes |= {PlanNode, SequentialNode, RepeatNode}

# remove classes that should not be mapped
classes -= set(recursive_subclasses(Enum))

# specify custom type mappings
type_mappings = {
    PhysicalObject: StringType(),
}


# self_mapped_classes += [PlanNode,
#                         SequentialNode,
#                         RepeatNode,



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
    ormatic = ORMatic(list(classes), mapper_registry, type_mappings)

    # Generate the ORM classes
    ormatic.make_all_tables()

    # Create the tables in the database
    mapper_registry.metadata.create_all(session.bind)

    # Write the generated code to a file
    generator = TablesGenerator(mapper_registry.metadata, session.bind, [])

    path = os.path.abspath(os.path.join(os.getcwd(), '../src/pycram/orm/'))
    with open(os.path.join(path, 'ormatic_interface.py'), 'w') as f:
        ormatic.to_python_file(generator, f)


if __name__ == '__main__':
    generate_orm()
