"""
This script generates the required database for the CI pipeline to execute notebooks.
This database is required for the execution of pycram/examples/improving_actions.ipynb.

For this script to work, you have to set a local variable in your environment called PYCRORM_CI_URI.
This variable contains the URI to the database is used for the CI pipeline with the credentials for an account that
has writing permissions to the database.

ONLY EXECUTE THIS IF YOU ARE SURE THAT YOU WANT TO DELETE THE DATABASE AND CREATE A NEW ONE.
"""

import os
import random
from datetime import timedelta

import numpy as np
import sqlalchemy.orm

from pycrap.ontologies import Robot, Milk

import pycram.orm.base
from pycram.designators.object_designator import ObjectDesignatorDescription
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import WorldMode, ApproachDirection, VerticalAlignment
from pycram.datastructures.pose import PoseStamped
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.process_module import ProcessModule, simulated_robot
from pycram.designators.specialized_designators.probabilistic.probabilistic_action import MoveAndPickUp, Arms, Grasp
from pycram.tasktree import task_tree
import pycram.orm.base


def main():
    np.random.seed(69)
    random.seed(69)

    pycrorm_uri = os.environ['PYCRORM_URI'] # os.environ['PYCRORM_CI_URI']
    pycrorm_uri = "mysql+pymysql://" + pycrorm_uri

    engine = sqlalchemy.create_engine(pycrorm_uri)
    session = sqlalchemy.orm.sessionmaker(bind=engine)()
    pycram.orm.base.Base.metadata.create_all(engine)

    world = BulletWorld(WorldMode.DIRECT)

    robot = Object("pr2", Robot, "pr2.urdf")
    milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([1.3, 1, 0.9]))
    viz_marker_publisher = VizMarkerPublisher()
    milk_description = ObjectDesignatorDescription(types=[Milk]).ground()

    fpa = MoveAndPickUp(milk_description, arms=[Arms.LEFT, Arms.RIGHT],
                        grasps=[ApproachDirection.FRONT.value, ApproachDirection.LEFT.value, ApproachDirection.RIGHT.value, VerticalAlignment.TOP.value])

    pycram.orm.base.ProcessMetaData().description = "Experimenting with Pick Up Actions"
    fpa.sample_amount = 100
    with simulated_robot:
        fpa.batch_rollout()
        task_tree.root.insert(session)
    session.commit()
    task_tree.reset_tree()
    world.exit()

if __name__ == "__main__":
    main()
