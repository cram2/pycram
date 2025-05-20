import random
import unittest
from time import sleep

import numpy as np
import sqlalchemy.orm
from random_events.variable import Continuous

from pycram.datastructures.dataclasses import BoundingBox
from pycram.datastructures.pose import PoseStamped
from pycram.designators.action_designator import NavigateAction, NavigateActionDescription, ParkArmsActionDescription, \
    PickUpActionDescription
from pycram.language import SequentialPlan
from pycram.parameterizer import Parameterizer, collision_free_event, update_variables_of_simple_event
from pycram.robot_description import RobotDescriptionManager, RobotDescription
from pycram.testing import EmptyBulletWorldTestCase
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Robot, Milk





class ProbabilisticActionTestCase(EmptyBulletWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        np.random.seed(69)
        random.seed(69)

        pycrorm_uri = "pycrorm@localhost:3306/pycrorm"
        pycrorm_uri = "mysql+pymysql://" + pycrorm_uri
        engine = sqlalchemy.create_engine(pycrorm_uri)
        cls.sessionmaker = sqlalchemy.orm.sessionmaker(bind=engine)()

        rdm = RobotDescriptionManager()
        rdm.load_description("pr2")
        cls.milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([0, 1, 0.9]))
        cls.robot = Object(RobotDescription.current_robot_description.name, Robot,
                           RobotDescription.current_robot_description.name + cls.extension)

    def move_and_pick_up_plan(self):
        return SequentialPlan(
        NavigateActionDescription(None),
        ParkArmsActionDescription(None),
        PickUpActionDescription(object_designator=self.milk)
        )

    def test_neem_generation(self):
        p = Parameterizer(self.move_and_pick_up_plan())
        condition = collision_free_event(self.world, search_space=BoundingBox(-10, -10, -10,
                                                                              10, 10, 10))
        x = condition.get_variable("x")
        y = condition.get_variable("y")
        z = condition.get_variable("z")

        new_variables = {
            x: Continuous("xa"),
            y: Continuous("ya"),
            z: Continuous("za"),
        }

        r = update_variables_of_simple_event(condition.simple_sets[0], new_variables)
        print(r)


if __name__ == '__main__':
    unittest.main()
