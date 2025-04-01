import logging
import os
import sys

from sqlacodegen.generators import TablesGenerator
from ormatic.ormatic import logger, ORMatic
from sqlalchemy import create_engine, select
from sqlalchemy.orm import registry, Session, clear_mappers

from pycram.datastructures.enums import Arms, TorsoState
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import ParkArmsAction, MoveTorsoAction, SetGripperAction
from pycram.orm.logging_hooks import ParkArmsActionSQL, \
    add_callbacks_for_mapped_classes, SetGripperActionSQL, MoveTorsoActionSQL, PoseSQL, PositionSQL, QuaternionSQL
from pycram.process_module import simulated_robot
from pycram.testing import BulletWorldTestCase

"""
WARNING: ORMatic only supports python 3.10. Make sure you are using the correct version when using with the ORM.
"""

def to_python_file(ormatic: ORMatic, mapper_registry: registry, session: Session, file: str):
    """
    Write the generated code to a file.
    :param ormatic: The ORMatic object.
    :param generator: The generator object.
    :param file: The file object to write to.
    """
    generator = TablesGenerator(mapper_registry.metadata, session.bind, [])

    path = os.path.abspath(os.path.join(os.getcwd(), '../src/pycram/orm/'))
    with open(os.path.join(path, file), 'w') as f:
        ormatic.to_python_file(generator, f)



# @unittest.skipIf(sys.version_info[:3] != (3.10), "ORMatic only supports python 3.10")
class ORMDesignatorTestCase(BulletWorldTestCase):
    def setUp(self):
        super().setUp()
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)

        self.mapper_registry = registry()
        engine = create_engine('sqlite:///:memory:')
        self.session = Session(engine)

    def tearDown(self):
        super().tearDown()
        self.mapper_registry.metadata.drop_all(self.session.bind)
        clear_mappers()
        self.session.close()

    def test_designator_creation(self):
        classes = [ParkArmsActionSQL, MoveTorsoActionSQL, PoseSQL, PositionSQL, QuaternionSQL]
        ormatic = ORMatic(classes, self.mapper_registry)
        ormatic.make_all_tables()
        to_python_file(ormatic, self.mapper_registry, self.session, 'ormatic_designator_interface.py')


        self.mapper_registry.metadata.create_all(self.session.bind)
        add_callbacks_for_mapped_classes(self.session)

        action = ParkArmsAction(Arms.BOTH)
        with simulated_robot:
            action.perform()
            MoveTorsoAction(TorsoState.HIGH).perform()

        result = self.session.scalars(select(ParkArmsActionSQL)).all()
        result2 = self.session.scalars(select(MoveTorsoActionSQL)).all()
        print(result)
        print(result2)
        self.assertEqual(len(result), 1)
        self.assertEqual(len(result2), 1)

