import glob
import os
import random
import shutil
import time

from pycram.datastructures.enums import WorldMode
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces.procthor import ProcTHORInterface
from pycram.language import ParallelPlan, CodePlan
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.process_module import simulated_robot
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Robot, Apartment, Milk, Cereal, Bowl, Spoon

procthor_interface = ProcTHORInterface(base_url="https://user.informatik.uni-bremen.de/~luc_kro/procthor_environments/")
resource_path, sampled_world = procthor_interface.sample_environment(keep_environment=True)
loginfo(f'Processing apartment:{sampled_world}')

np.random.seed(420)
extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)
world.add_resource_path(resource_path)


viz = VizMarkerPublisher()
viz2 = VizMarkerPublisher(use_prospection_world=True)

robot = Object("pr2", Robot, f"pr2{extension}", pose=PoseStamped.from_list([1, 2, 0]))

milk = Object("milk", Milk, "milk.stl", pose=PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1]),
              color=Color(1, 0, 0, 1))
cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                pose=PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1]), color=Color(0, 1, 0, 1))
spoon = Object("spoon", Spoon, "spoon.stl", pose=PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1]),
               color=Color(0, 0, 1, 1))
bowl = Object("bowl", Bowl, "bowl.stl", pose=PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1]),
              color=Color(1, 1, 0, 1))

apartment = Object(sampled_world, Apartment, sampled_world+extension)
robot_desig_resolved = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()

# substrings that are used to identify potential surfaces in our world. This should in the future be replaced by a more
# robust way of identifying surfaces, e.g. by using RippleDownRules
apartment_links = apartment.link_names
potential_surfaces_procthor = ["table_", "countertop_", "desk_", 'deskgustav_', 'tvstand_', 'dresser_']
known_obj_names_procthor = ['wall_', 'kitchen_', 'bathroom_', 'livingroom_', 'bedroom_'] + potential_surfaces_procthor
potential_surfaces = ['table_area_main', 'kitchen_island', 'coffee_table', 'bedside_table', 'island_countertop'] + potential_surfaces_procthor
available_surfaces = [link for link in apartment_links if any(surf in link.lower() for surf in potential_surfaces)]
used_surfaces = random.sample(available_surfaces, k=min(len(available_surfaces), 3))
loginfo(f"Used surfaces in {sampled_world}: {used_surfaces}")

start_time = time.time_ns()

try:
    if not available_surfaces:
        raise ValueError(f"No available surfaces found in apartment links for apartment '{sampled_world}'.")

    prob_costmap = ProbabilisticSemanticLocation(used_surfaces, apartment, for_object=milk, link_is_center_link=True)

    prob_costmap_iter = iter(prob_costmap)

    with simulated_robot:
        ParallelPlan(MoveTorsoActionDescription(TorsoState.MID),
                     ParkArmsActionDescription(Arms.BOTH)).perform()

        milk.set_pose(next(prob_costmap_iter))
        cereal.set_pose(next(prob_costmap_iter))
        spoon.set_pose(next(prob_costmap_iter))
        bowl.set_pose(next(prob_costmap_iter))

        sp = SequentialPlan(
            TransportActionDescription(ResolutionStrategyObject(
                strategy=SearchActionDescription(CodePlan(milk.get_pose).perform(), Milk)),
                prob_costmap_iter, place_rotation_agnostic=True),
            ParallelPlan(MoveTorsoActionDescription(TorsoState.MID),
                         ParkArmsActionDescription(Arms.BOTH)),
            TransportActionDescription(ResolutionStrategyObject(
                strategy=SearchActionDescription(CodePlan(cereal.get_pose).perform(), Cereal)),
                prob_costmap_iter,place_rotation_agnostic=True),
            ParallelPlan(MoveTorsoActionDescription(TorsoState.MID),
                         ParkArmsActionDescription(Arms.BOTH)),
            TransportActionDescription(ResolutionStrategyObject(
                strategy=SearchActionDescription(CodePlan(spoon.get_pose).perform(), Spoon)),
                prob_costmap_iter),
            ParallelPlan(MoveTorsoActionDescription(TorsoState.MID),
                         ParkArmsActionDescription(Arms.BOTH)),
            TransportActionDescription(ResolutionStrategyObject(
                strategy=SearchActionDescription(CodePlan(bowl.get_pose).perform(), Bowl)),
                prob_costmap_iter, place_rotation_agnostic=True),
            ParallelPlan(MoveTorsoActionDescription(TorsoState.MID),
                         ParkArmsActionDescription(Arms.BOTH)),
        )
        sp.perform()
        loginfo("ProcTHOR demo completed successfully.")
except Exception as e:
    logerr(f"The ProcTHOR demo failed with the following error: {e}")

loginfo(f"Time taken to complete: {(time.time_ns() - start_time) / 1e9} seconds")

[shutil.rmtree(d) for d in glob.glob("../../tmp/dataset_house_*") if os.path.isdir(d)]

viz._stop_publishing()
viz2._stop_publishing()
world.exit()
