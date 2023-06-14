import dataclasses
import time
from typing import Optional, List, Tuple

import jpt
import numpy as np
import pybullet
import tf

import pycram.designators.location_designator
import pycram.task
from pycram.costmaps import OccupancyCostmap, plot_grid
from pycram.plan_failures import PlanFailure


class JPTCostmapLocation(pycram.designators.location_designator.CostmapLocation):
    """Costmap Locations using Joint Probability Trees (JPTs).
    JPT costmaps are trained to model the dependency with a robot position relative to the object, the robots type,
    the objects type, the robot torso height, and the grasp parameters.
    Solutions to the problem definitions are chosen in such a way that the success probability is highest.
    """

    @dataclasses.dataclass
    class Location(pycram.designators.location_designator.LocationDesignatorDescription.Location):
        pose: Tuple[List[float], List[float]]
        reachable_arm: str
        torso_height: float
        grasp: str

    def __init__(self, target, reachable_for=None, reachable_arm=None,
                 model: Optional[jpt.trees.JPT] = None, path: Optional[str] = None, resolver=None):
        """
        Create a JPT Costmap

        :param target: The target object
        :param reachable_for: The robot to grab the object with
        :param reachable_arm: The arm to use
        :param model: The JPT model as a loaded tree in memory, either model or path must be set
        :param path: The path to the JPT model, either model or path must be set
        """
        super().__init__(target, reachable_for, None, reachable_arm, resolver)

        # check if arguments are plausible
        if (not model and not path) or (model and path):
            raise ValueError("Either model or path must be set.")

        # set model
        if model:
            self.model = model

        # load model from path
        if path:
            self.model = jpt.trees.JPT.load(path)

        # initialize member for visualized objects
        self.visual_ids: List[int] = []

    def evidence_from_occupancy_costmap(self) -> List[jpt.variables.LabelAssignment]:
        """
        Create a list of boxes that can be used as evidences for a jpt. The list of boxes describe areas where the
        robot can stand.

        :return: List of evidences describing the found boxes
        """

        # create Occupancy costmap for the target object
        position, orientation = self.target.get_position_and_orientation()
        position = list(position)
        position[-1] = 0

        ocm = OccupancyCostmap(distance_to_obstacle=0.3, from_ros=False, size=200, resolution=0.02,
                               origin=(position, orientation))
        # ocm.visualize()

        # working on a copy of the costmap, since found rectangles are deleted
        map = np.copy(ocm.map)

        # initialize result
        queries = []

        origin = np.array([ocm.height/2, ocm.width/2])

        # for every index pair (i, j) in the occupancy map
        for i in range(0, map.shape[0]):
            for j in range(0, map.shape[1]):

                # if this index has not been used yet
                if map[i][j] > 0:

                    # get consecutive box
                    width = ocm._find_consectuive_line((i, j), map)
                    height = ocm._find_max_box_height((i, j), width, map)

                    # mark box as used
                    map[i:i+height, j:j+width] = 0

                    # calculate to coordinates relative to the objects pose
                    pose = np.array([i, j])
                    lower_corner = (pose - origin) * ocm.resolution
                    upper_corner = (pose - origin + np.array([height, width])) * ocm.resolution
                    rectangle = np.array([lower_corner, upper_corner]).T

                    # transform to jpt query
                    query = self.model.bind({"x": list(rectangle[0]), "y": list(rectangle[1])})
                    queries.append(query)

        return queries

    def create_evidence(self, use_success=True) -> jpt.variables.LabelAssignment:
        """
        Create evidence usable for JPTs where type and status are set if wanted.

        :param use_success: Rather to set success or not
        :return: The usable label-assignment
        """
        evidence = dict()

        evidence["type"] = {self.target.type}

        if use_success:
            evidence["status"] = {"SUCCEEDED"}

        return self.model.bind(evidence)

    def sample(self, amount: int = 1) -> np.ndarray:
        """
        Sample from the locations that fit the CostMap and are not occupied.

        :param amount: The amount of samples to draw
        :return: A numpy array containing the samples drawn from the tree.
        """
        evidence = self.create_evidence()

        locations = self.evidence_from_occupancy_costmap()
        solutions = []

        for location in locations:
            for variable, value in evidence.items():
                location[variable] = value

            for leaf in self.model.apply(location):
                if leaf.probability(location) == 0:
                    continue
                altered_leaf = leaf.conditional_leaf(location)
                success_probability = altered_leaf.probability(location)

                _, mpe_state = altered_leaf.mpe(self.model.minimal_distances)
                location["grasp"] = mpe_state["grasp"]
                location["arm"] = mpe_state["arm"]
                location["relative torso height"] = mpe_state["relative torso height"]
                location["x"] = mpe_state["x"]
                location["y"] = mpe_state["y"]
                solutions.append((location, success_probability, leaf.prior))

        solutions = sorted(solutions, key=lambda x: x[1], reverse=True)
        best_solution = solutions[0]
        conditional_model = self.model.conditional_jpt(best_solution[0])

        # conditional_model.plot(plotvars=conditional_model.variables)
        return conditional_model.sample(amount)

    def sample_to_location(self, sample: np.ndarray) -> Location:
        """
        Convert a numpy array sampled from the JPT to a costmap-location

        :param sample: The drawn sample
        :return: The usable costmap-location
        """
        sample_dict = {variable.name: value for variable, value in zip(self.model.variables, sample)}
        target_x, target_y, target_z = self.target.pose
        pose = [target_x + sample_dict["x"], target_y + sample_dict["y"], 0]

        angle = np.arctan2(pose[1] - target_y, pose[0] - target_x) + np.pi

        orientation = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
        torso_height = np.clip(target_z - sample_dict["relative torso height"], 0, 0.33)
        result = self.Location((pose, orientation), sample_dict["arm"], torso_height, sample_dict["grasp"])
        return result

    def __iter__(self):
        samples = self.sample(200)
        for sample in samples:
            yield self.sample_to_location(sample)

    def visualize(self):
        """
        Plot the possible areas to stand in the BulletWorld. The opacity is the probability of success.

        """

        evidence = self.create_evidence(use_success=False)

        conditional_model = self.model.conditional_jpt(evidence)

        for leaf in conditional_model.leaves.values():

            success = leaf.distributions["status"].p({"SUCCEEDED"})

            if success == 0:
                continue

            x_intervals = leaf.distributions["x"].cdf.intervals
            y_intervals = leaf.distributions["y"].cdf.intervals

            x_range = np.array([x_intervals[0].upper, x_intervals[-1].lower])
            y_range = np.array([y_intervals[0].upper, y_intervals[-1].lower])

            center = np.array([sum(x_range) / 2, sum(y_range) / 2])

            visual = pybullet.createVisualShape(pybullet.GEOM_BOX,
                                                halfExtents=[(x_range[1] - x_range[0]) / 2,
                                                             (y_range[1] - y_range[0]) / 2, 0.001],
                                                rgbaColor=[1, 0, 0, success],
                                                visualFramePosition=[*center, 0])

            self.visual_ids.append(visual)

        for id_list in np.array_split(np.array(self.visual_ids), np.ceil(len(self.visual_ids) / 127)):
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
        """
        Close all plotted objects.

        """
        for id in self.visual_ids:
            pybullet.removeBody(id)
        self.visual_ids = []
