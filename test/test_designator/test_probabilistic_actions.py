import random
import unittest

import numpy as np
import sqlalchemy.orm
from krrood.ormatic.dao import to_dao
from sqlalchemy import select

from pycram.datastructures.enums import TaskStatus
from pycram.datastructures.pose import PoseStamped
from pycram.designator import ObjectDesignatorDescription
from pycram.language import SequentialPlan
from pycram.orm.ormatic_interface import Base, ResolvedActionNodeMappingDAO
from pycram.robot_plans import MoveAndPickUpActionDescription, MoveAndPickUpAction
from pycram.designators.specialized_designators.probabilistic.probabilistic_action import (
    MoveAndPickUpParameterizer,
)
from pycram.failures import PlanFailure
from pycram.plan import Plan, ResolvedActionNode, PlanNode
from pycram.process_module import simulated_robot
from pycram.robot_description import RobotDescriptionManager, RobotDescription
from pycram.testing import EmptyWorldTestCase, ApartmentWorldTestCase


class MoveAndPickUpTestCase(ApartmentWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        np.random.seed(69)
        random.seed(69)

        pycrorm_uri = "sqlite:///:memory:"
        engine = sqlalchemy.create_engine(pycrorm_uri)
        Base.metadata.create_all(bind=engine)
        cls.session = sqlalchemy.orm.sessionmaker(bind=engine)()

        # rdm = RobotDescriptionManager()
        # rdm.load_description("pr2")
        # cls.milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([0, 1, 0.9]))
        # cls.robot = Object(RobotDescription.current_robot_description.name, Robot,
        #                    RobotDescription.current_robot_description.name + cls.extension)
    @unittest.skip
    def test_orm(self):
        mpa_description = MoveAndPickUpActionDescription(
            None, [self.world.get_body_by_name("milk.stl")], None, None, None
        )
        plan = SequentialPlan(self.context, mpa_description)
        mpa = MoveAndPickUpParameterizer(
            mpa_description, world=self.world
        ).create_action()

        plan = Plan(
            ResolvedActionNode(
                designator_ref=mpa, kwargs={}, designator_type=MoveAndPickUpAction
            ),
            self.context,
        )

        with simulated_robot:
            try:
                plan.perform()
            except PlanFailure as e:
                ...

        dao = to_dao(plan)
        self.session.add(dao)
        self.session.commit()

        result = self.session.scalars(select(ResolvedActionNodeMappingDAO)).first()
        self.assertEqual(result.status, TaskStatus.SUCCEEDED)


if __name__ == "__main__":
    unittest.main()
