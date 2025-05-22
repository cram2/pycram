import random
import unittest

import numpy as np
import sqlalchemy.orm

from pycram.datastructures.pose import PoseStamped
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.action_designator import MoveAndPickUpActionDescription
from pycram.designators.specialized_designators.probabilistic.probabilistic_action import MoveAndPickUpParameterizer
from pycram.orm.logging_hooks import insert
from pycram.plan import Plan, ResolvedActionNode
from pycram.process_module import simulated_robot
from pycram.robot_description import RobotDescriptionManager, RobotDescription
from pycram.testing import EmptyBulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Robot, Milk
from pycram.orm.ormatic_interface import mapper_registry


class MoveAndPickUpTestCase(EmptyBulletWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        np.random.seed(69)
        random.seed(69)

        pycrorm_uri = "sqlite:///:memory:"
        # pycrorm_uri = "mysql+pymysql://" + "pycrorm@localhost:3306/pycrorm"
        engine = sqlalchemy.create_engine(pycrorm_uri)
        mapper_registry.metadata.create_all(bind=engine)
        cls.session = sqlalchemy.orm.sessionmaker(bind=engine)()

        rdm = RobotDescriptionManager()
        rdm.load_description("pr2")
        cls.milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([0, 1, 0.9]))
        cls.robot = Object(RobotDescription.current_robot_description.name, Robot,
                           RobotDescription.current_robot_description.name + cls.extension)

    def test_something(self):
        odd = ObjectDesignatorDescription(types=[Milk])
        mpa_description = MoveAndPickUpActionDescription(None, odd, None, None, None)
        mpa = MoveAndPickUpParameterizer(mpa_description.root).create_action()

        plan = Plan(ResolvedActionNode(designator_ref=mpa))

        with simulated_robot:
            try:
                plan.perform()
            except Exception as e:
                ...

        insert(plan, self.session)


if __name__ == '__main__':
    unittest.main()
