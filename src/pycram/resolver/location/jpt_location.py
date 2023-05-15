import dataclasses
import itertools
from typing import Optional, List, Tuple

import jpt
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import pybullet
import sqlalchemy.orm
import tf
import tqdm

import pycram.costmaps
import pycram.designators.location_designator
import pycram.task
from pycram.bullet_world import BulletWorld, Object as BulletWorldObject
from pycram.designators.action_designator import MoveTorsoAction, PickUpAction, NavigateAction
from pycram.designators.object_designator import ObjectDesignatorDescription
from pycram.plan_failures import PlanFailure
from pycram.process_module import simulated_robot
from pycram.costmaps import OccupancyCostmap
from sqlalchemy.ext.compiler import compiles
import mlflow


class JPTCostmapLocation(pycram.designators.location_designator.CostmapLocation):

    @dataclasses.dataclass
    class Location(pycram.designators.location_designator.LocationDesignatorDescription.Location):
        pose: Tuple[List[float], List[float]]
        reachable_arm: str
        torso_height: float
        grasp: str

    def __init__(self, target, reachable_for=None, reachable_arm=None,
                 model: Optional[jpt.trees.JPT] = None, path: Optional[str] = None, resolver=None):
        super().__init__(target, reachable_for, None, reachable_arm, resolver)

        if (not model and not path) or (model and path):
            raise ValueError("Either model or path must be set.")

        if model:
            self.model = model

        if path:
            self.model = jpt.trees.JPT.load(path)

        self.visual_ids: List[int] = []

    def create_evidence(self, use_success=True):
        evidence = dict()

        if self.reachable_for:
            evidence["robot_type"] = {self.reachable_for.type}

        evidence["object_type"] = {self.target.type}

        if use_success:
            evidence["success"] = {"True"}

        return self.model.bind(evidence)

    def sample(self, amount: int = 1):
        """Sample from the locations that fit the CostMap."""

        # cm = OccupancyCostmap(distance_to_obstacle=0.4, from_ros=False, size=100, resolution=0.02,
        #                       origin=self.target.pose)

        evidence = self.create_evidence()

        try:
            mpes, likelihood = self.model.mpe(evidence, fail_on_unsatisfiability=True)
        except jpt.trees.Unsatisfiability as e:
            raise PlanFailure(str(e))

        state = mpes[0]

        conditional_model = self.model.conditional_jpt(state)

        return conditional_model.sample(amount)

    def sample_to_location(self, sample: np.ndarray) -> Location:
        sample_dict = {variable.name: value for variable, value in zip(self.model.variables, sample)}

        angle = np.arctan2(sample_dict["y_position"] - self.target.pose[1], sample_dict["x_position"] -
                           self.target.pose[0]) + np.pi

        target_x, target_y, target_z = self.target.pose
        pose = [target_x - sample_dict["x_position"], target_y - sample_dict["y_position"], 0]
        orientation = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
        torso_height = target_z - sample_dict["z_position"]
        result = self.Location((pose, orientation), sample_dict["arm"], torso_height, sample_dict["grasp"])
        return result

    def __iter__(self):
        samples = self.sample(200)

        for sample in samples:
            yield self.sample_to_location(sample)

    def visualize(self):
        """Plot the possible areas to stand in the BulletWorld. The opacity is the probability of success."""
        evidence = self.create_evidence(use_success=False)

        conditional_model = self.model.conditional_jpt(evidence)

        for leaf in conditional_model.leaves.values():

            success = leaf.distributions["success"].p({"True"})

            if success == 0:
                continue

            x_intervals = leaf.distributions["x_position"].cdf.intervals
            y_intervals = leaf.distributions["y_position"].cdf.intervals

            x_range = np.array([x_intervals[0].upper, x_intervals[-1].lower])
            y_range = np.array([y_intervals[0].upper, y_intervals[-1].lower])

            center = np.array([sum(x_range) / 2, sum(y_range) / 2])

            visual = pybullet.createVisualShape(pybullet.GEOM_BOX,
                                                halfExtents=[(x_range[1] - x_range[0]) / 2,
                                                             (y_range[1] - y_range[0]) / 2, 0.001],
                                                rgbaColor=[1, 0, 0, success],
                                                visualFramePosition=[*center, 0])

            self.visual_ids.append(visual)

        for id_list in np.array_split(np.array(self.visual_ids), np.ceil(len(self.visual_ids)/127)):

            # Dummy paramater since these are needed to spawn visual shapes as a multibody.
            link_poses = [[0, 0, 0] for c in id_list]
            link_orientations = [[0, 0, 0, 1] for c in id_list]
            link_masses = [1.0 for c in id_list]
            link_parent = [0 for c in id_list]
            link_joints = [pybullet.JOINT_FIXED for c in id_list]
            link_collision = [-1 for c in id_list]
            link_joint_axis = [[1, 0, 0] for c in id_list]

            # The position at which the multibody will be spawned. Offset such that
            # the origin referes to the centre of the costmap.
            origin_pose = self.target.get_position_and_orientation()
            base_position = list(origin_pose[0])
            base_position[2] = 0

            map_obj = pybullet.createMultiBody(baseVisualShapeIndex=-1, linkVisualShapeIndices=id_list,
                                               basePosition=base_position, baseOrientation=origin_pose[1],
                                               linkPositions=link_poses,
                                               linkMasses=link_masses, linkOrientations=link_orientations,
                                               linkInertialFramePositions=link_poses,
                                               linkInertialFrameOrientations=link_orientations,
                                               linkParentIndices=link_parent,
                                               linkJointTypes=link_joints, linkJointAxis=link_joint_axis,
                                               linkCollisionShapeIndices=link_collision)
            self.visual_ids.append(map_obj)

    def close_visualization(self) -> None:
        for id in self.visual_ids:
            pybullet.removeBody(id)
        self.visual_ids = []
