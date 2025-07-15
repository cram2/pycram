# Procthor Interface - Creating a Manager for testing multible robots in mutliple enviromnents

This Notebook aims to provide an overview of the ProcThor Interface, by giving an easy-to-understand example on how to 
use it. We will be executing a simple Transporting Plan in a randomly chosen environment with a PR2 robot.

We will start to set up our ProcThorInterface.
The base URL is the server where the environments are stored. The ProcTHOR interface expects the environments to be
stored as tar.gz files, where each file contains a single environment with all its assets.
When sampling an environment, we can choose to keep the environment or not. If we keep it, it will be stored in the
"resources/procthor_environments" folder, otherwise it will be stored in the "tmp" folder and deleted after the run.

The "resources/procthor_environments" folder is excluded from the git repository, since hosting all environments in this
repository would be too large.

```python
from pycram.external_interfaces.procthor import ProcTHORInterface

procthor_interface = ProcTHORInterface(base_url="https://user.informatik.uni-bremen.de/~luc_kro/procthor_environments/")
resource_path, sampled_world = procthor_interface.sample_environment(keep_environment=True)

print(f"Resource path: {resource_path}")
print(f"Sampled world: {sampled_world}")
```

Next, we will set up our simulation environment using the BulletWorld. To ensure our sampled environment is loaded correctly,
we need to add the resource path to our bullet world.

```python
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.worlds.bullet_world import BulletWorld
from pycram.datastructures.enums import WorldMode
from pycram.world_concepts.world_object import Object
from pycrap.ontologies import Robot, Apartment, Cereal
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.dataclasses import Color
from pycram.designators.object_designator import BelieveObject
from pycram.robot_description import RobotDescription

extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.GUI)
world.add_resource_path(resource_path)

robot = Object("pr2", Robot, f"pr2{extension}", pose=PoseStamped.from_list([1, 2, 0]))

cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                pose=PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1]), color=Color(0, 1, 0, 1))
apartment = Object(sampled_world, Apartment, sampled_world+extension)
robot_desig_resolved = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()

```

Afterwards, we will define a few known substrings that are used to identify potential surfaces in our world. 
In the future this will be replaced by a more robust way of identifying surfaces, e.g. by using RippleDownRules.

For large environments, we won't use all surfaces, since the computation load would be too high. Instead, we will 
sample a maximum of 3 surfaces.

```python
import random
potential_surfaces_procthor = ["table_", "countertop_", "desk_", 'deskgustav_', 'tvstand_', 'dresser_']
known_obj_names_procthor = ['wall_', 'kitchen_', 'bathroom_', 'livingroom_', 'bedroom_'] + potential_surfaces_procthor
potential_surfaces = ['table_area_main', 'kitchen_island', 'coffee_table', 'bedside_table', 'island_countertop'] + potential_surfaces_procthor

apartment_links = apartment.link_names
available_surfaces = [link for link in apartment_links if any(surf in link.lower() for surf in potential_surfaces)]
used_surfaces = random.sample(available_surfaces, k=min(len(available_surfaces), 3))
```

Now we will create ProbabilisticSemanticLocation, which will sample poses for the used surfaces. We then access its iterator
and sample a starting pose for our cereal object. 

The `link_is_center_link` parameter is set to 'True', since ProcTHOR environments currently don"t have any links on the
surfaces of objects, so we need to use the center link of the object as the bases for our calculations.

```python
from pycram.designators.location_designator import ProbabilisticSemanticLocation

prob_costmap = ProbabilisticSemanticLocation(used_surfaces, apartment, for_object=cereal, link_is_center_link=True)

prob_costmap_iter = iter(prob_costmap)

cereal.set_pose(next(prob_costmap_iter))
```

Lastly we will execute a simple Transporting Plan with the PR2 robot in our randomly chosen environment.

```python
from pycram.robot_plans.actions import *
from pycram.language import ParallelPlan, CodePlan, SequentialPlan
from pycram.datastructures.enums import TorsoState, Arms
from pycram.designators.object_designator import ResolutionStrategyObject
from pycram.process_module import simulated_robot

with simulated_robot:
    sp = SequentialPlan(
        ParallelPlan(MoveTorsoActionDescription(TorsoState.MID),
                     ParkArmsActionDescription(Arms.BOTH)).perform(),
        TransportActionDescription(ResolutionStrategyObject(
            strategy=SearchActionDescription(CodePlan(cereal.get_pose).perform(), Cereal)),
            prob_costmap_iter,place_rotation_agnostic=True)
    )
    sp.perform()

```

