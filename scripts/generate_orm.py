import logging
import os
import sys
from sqlacodegen.generators import TablesGenerator
from sqlalchemy import create_engine
from sqlalchemy.orm import registry, Session
import pycram.datastructures.pose
import pycram.datastructures.grasp
import pycram.language
import pycram.plan
import pycram.designator
from ormatic.ormatic import logger, ORMatic
from pycram.orm.model import self_mapped_classes, DetectAction, TaskTreeNode, explicitly_mapped_classes
"""
List of standard classes that are to be mapped to the database.
"""
pycram.plan.ResolvedActionNode.__annotations__.update({"designator_ref": pycram.designator.ActionDescription})
pycram.plan.MotionNode.__annotations__.update({"designator_ref": pycram.designator.BaseMotion})


classes = [pycram.datastructures.grasp.GraspDescription,
           pycram.datastructures.pose.Vector3,
           pycram.datastructures.pose.Quaternion,
           pycram.datastructures.pose.Pose,
           pycram.datastructures.pose.Header,
           pycram.datastructures.pose.PoseStamped,
           pycram.datastructures.pose.Transform,
           pycram.datastructures.pose.TransformStamped,
           pycram.datastructures.pose.GraspPose,
           pycram.designator.ActionDescription,
           pycram.plan.PlanNode,
           pycram.language.SequentialNode,
           pycram.language.ParallelNode,
           ]


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
    ormatic = ORMatic(classes + self_mapped_classes + explicitly_mapped_classes, mapper_registry)

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