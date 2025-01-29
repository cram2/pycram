import time
import unittest

import networkx as nx
from matplotlib import pyplot as plt
from probabilistic_model.probabilistic_circuit.nx.helper import fully_factorized
from random_events.product_algebra import SimpleEvent
from sortedcontainers import SortedSet

from pycram.datastructures.mixins import ParameterInfo
from pycram.designator import ObjectDesignatorDescription
from pycram.designators import action_designator, object_designator
from pycram.designators.action_designator import MoveTorsoActionPerformable, PickUpActionPerformable, \
    NavigateActionPerformable, FaceAtPerformable, MoveTorsoAction
from pycram.local_transformer import LocalTransformer
from pycram.plan import Plan, ActionCore, parameters_from_path_dict
from pycram.robot_description import RobotDescription
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import ObjectType, Arms, GripperState, Grasp, DetectionTechnique, TorsoState
from pycram.testing import BulletWorldTestCase
import numpy as np
from pycrap.ontologies import Milk


class TestPlanGrounding(BulletWorldTestCase):
    """Testcase for the plan objects without worlds"""

    def test_plan_creation(self):
        plan = Plan()
        navigate = ActionCore(action_designator.NavigateActionPerformable, plan)

        n2 = ActionCore(action_designator.NavigateActionPerformable, plan)
        navigate.next_action(n2)

        variables = SortedSet(plan.variables())
        means = {variable: 0 for variable in variables}
        variances = {variable: 1 for variable in variables}

        model = fully_factorized(variables, means, variances)
        sample = model.sample(1)[0]

        parameters = dict()
        for action_core in plan.nodes:
            parameters[action_core] = {tuple(variable.path): ParameterInfo(None, None, value) for variable, value in zip(variables, sample)
                                       if variable.action_core == action_core}
            parameters[action_core] = parameters_from_path_dict(parameters[action_core])
        print(parameters)


        with simulated_robot:
            plan.perform(parameters)


        # plt.show()


