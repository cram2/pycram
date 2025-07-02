---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.16.4
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# Graph of Convex Sets

This notebook walks through the way PyCRAM navigates spaces that are challenging to navigate.
Usually, navigation is done in spaces that are convex, meaning that every point is reachable via a straight line without collision.
Unfortunately, the real world is not like this.

PyCRAM internally represents the objects in the world using some implementation of a scene description format. These formats include collision information for every object.
The collision information is often approximated using a set of boxes.

These collision boxes are converted to their algebraic representation using the [random-events](https://github.com/tomsch420/random-events) package.
This allows the free space to be formulated as the complement of the belief state collision boxes.
The complement itself is a finite collection of (possible infinitely big) boxes that do not intersect.
These boxes, however, have surfaces that are adjacent.
Representing this adjacency is done using a Graph of Convex Sets (GCS) where every node is a box, 
and every edge means that these boxes are adjacent.
Navigating the free space is then possible using path finding algorithms on the graph.

You can read more about GCS [here](https://arxiv.org/abs/2101.11565).

+++

Let's get hands on! First, we need to create an object that makes navigation non-trivial.

```{code-cell} ipython2

from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import WorldMode
from pycrap.ontologies import PhysicalObject

from pycram.worlds.bullet_world import BulletWorld
from pycram.object_descriptors.generic import ObjectDescription as GenericObjectDescription
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher

world = BulletWorld(mode=WorldMode.DIRECT)
viz_marker_publisher = VizMarkerPublisher()
obstacle = Object("obstacle", concept=PhysicalObject, description=GenericObjectDescription("obstacle", [0,0,0.5], [0.5, 0.5, 0.5]))
```

Next, we create a connectivity graph of the space so we can solve navigation problems.
To visualize the result in a better way, we limit the search space to a finite set around the box. Furthermore, we constraint the robot to be unable to fly by constraining the z-axis. Otherwise, he would get the idea
to go over the box, which is not a good idea.

```{code-cell} ipython2
from random_events.interval import SimpleInterval
from pycram.graph_of_convex_sets import GraphOfConvexSets
from pycram.datastructures.dataclasses import BoundingBox

search_space = BoundingBox(min_x=-1, max_x=1,
                           min_y=-1, max_y=1,
                           min_z=0.1, max_z=0.2).as_collection()
gcs = GraphOfConvexSets.free_space_from_world(world, search_space=search_space)
```

Let's have a look at the free space constructed. We can see that it is a rectangular catwalk around the obstacle.

```{code-cell} ipython2
import plotly
plotly.offline.init_notebook_mode()
import plotly.graph_objects as go

fig = go.Figure(gcs.plot_free_space())
fig.show()
```

Looking at the connectivity graph, we can see that it is still possible to go from one side of the box to the other, just not directly. Intuitively, we can see that we just have to go around the obstacle.

```{code-cell} ipython2
import matplotlib.pyplot as plt
import networkx as nx
nx.draw(gcs, with_labels=True, font_size=8)
```

Let's use graph theory to find a path!

```{code-cell} ipython2
from pycram.datastructures.pose import PoseStamped

start = PoseStamped.from_list([-0.75, 0, 0.15])
goal = PoseStamped.from_list([0.75, 0, 0.15])
path = gcs.path_from_to(start, goal)
print("A potential path is", [(point.position.x, point.position.y) for point in path])
```

This minimal example demonstrates a concept that can be applied to the entire belief state of the robot. Let's load a more complex environment and look at the connectivity of it.

```{code-cell} ipython2
from pycrap.ontologies import Kitchen

kitchen = Object("kitchen", Kitchen, "kitchen.urdf")

search_space = BoundingBox(min_x=-2, max_x=2,
                           min_y=-2, max_y=2,
                           min_z=0., max_z=2).as_collection()
gcs = GraphOfConvexSets.free_space_from_world(world, search_space=search_space)
```

We can now see the algebraic representation of the occupied and free space. The free space is the complement of the occupied space.

```{code-cell} ipython2

from plotly.subplots import make_subplots

fig = make_subplots(rows=1, cols=2,  specs=[[{'type': 'surface'}, {'type': 'surface'}]], subplot_titles=["Occupied Space", "Free Space"])

occupied_traces = gcs.plot_occupied_space()
fig.add_traces(occupied_traces, rows=[1 for _ in occupied_traces], cols=[1 for _ in occupied_traces])
free_traces = gcs.plot_free_space()
fig.add_traces(free_traces, rows=[1 for _ in free_traces], cols=[2 for _ in free_traces])
fig.show()
```

Now let's look at the connectivity of the entire world!

```{code-cell} ipython2
import networkx as nx
nx.draw(gcs, node_size=10)
```

We can see that all spaces are somehow reachable from everywhere besides one isolated region! Amazing!
This allows the accessing of locations using a sequence of local problems put together in an overarching trajectory!
Finally, let's find a way from here to there:

```{code-cell} ipython2
start = PoseStamped.from_list([-0.75, 0, 0.15])
goal = PoseStamped.from_list([0.75, 0, 0.15])
path = gcs.path_from_to(start, goal)
print("A potential path is", [(point.position.x, point.position.y, point.position.z) for point in path])
```

Known limitations and potential improvements are:
- The connectivity graph currently calculates its edges by using an approximation to adjacent surfaces. This can be improved by an exact calculation. 
- The path is generated through the center points of the connection boxes. This is perhaps not optimal
- The path is chosen by taking the shortest (meaning the least amount of edges) path. This is not necessarily the best path