![](doc/images/new_pycram_logo_clean.svg#gh-light-mode-only)
![](doc/images/new_pycram_logo_clean_dark.svg#gh-dark-mode-only)


# PyCRAM — High‑level feature highlights 

[![Python Tests](https://github.com/cram2/pycram/actions/workflows/new-pycram-ci.yml/badge.svg)](https://github.com/cram2/pycram/actions/workflows/new-pycram-ci.yml/badge.svg)
[![Example Tests](https://github.com/cram2/pycram/actions/workflows/notebook-test-ci.yml/badge.svg)](https://github.com/cram2/pycram/actions/workflows/notebook-test-ci.yml/badge.svg)


## Key Features

- Intent-based task specification: express “what” to do using designators; the framework decides the concrete “how” at run time.
- Late binding and adaptability: defer grounding of actions, motions, objects, and locations until execution to match the current world and robot state.
- Composable plans: build behaviors as trees of plan nodes with clear control-flow constructs (sequential, parallel, retry, repeat, monitor).
- Introspection and observability: plans track node status, timing, and context for debugging, visualization, and monitoring.
- ORM-backed persistence: automatically capture plans, actions, spatial data, and outcomes in a relational schema for analysis and reproducibility.
- Reusable across robots: separate intent from embodiment so the same high-level logic can target different platforms.
- Motion building blocks: ready-to-use motion designators (e.g., TCP and gripper control) compose into higher-level actions like pick-up and transport.
- World model integration: operate over bodies, links, transforms, and semantic annotations; reason about objects and places, not only coordinates.
- Simulation-first workflow: run end-to-end plans in simulation for rapid iteration, with optional visualization.
- Clear validation points: action and motion steps can declare pre/post conditions to verify success (e.g., object grasped).
- Data-driven improvement: stored execution traces support benchmarking, failure analysis, and experiment tracking.
- Modular and extensible: define custom designators, actions, and plan patterns while leveraging the common orchestration and logging.
- Tested examples and demos: runnable scenarios (e.g., PR2 pick-and-place) illustrate typical workflows and best practices.

## Why it matters
- ✅ Fewer brittle hacks, more reusable intent.
- ✅ Faster iteration in simulation, safer rollouts on real robots.
- ✅ Robust control that anticipates uncertainty—and recovers when reality disagrees.

## Installation

The recommended installation method is via `pip`:

```bash
pip install pycram-robotics
```

While this works out-of-the-box to execute the examples or tests PyCRAM needs a ROS installation to load URDFs or use the 
visualization with RViz2. Look at the detailed installation instructions for more infos.  

Detailed installation instructions and manual setup guides are available [here](https://pycram.readthedocs.io/en/latest/installation.html).

## Live Demonstration in the Virtual Research Building

Test PyCRAM directly in your browser via our [Virtual Research Building](https://vib.ai.uni-bremen.de/page/fallschool/).
Explore a variety of labs and demonstrations showcasing PyCRAM's capabilities on the [Labs page](https://vib.ai.uni-bremen.de/page/labs/) of our virtual building.


### Setting Up Your Own Lab

To create a custom lab in the virtual building, consult the `vrb` branch of this repository, which includes detailed setup instructions and templates.


## Documentation

The full documentation is maintained at [Read the Docs](https://pycram.readthedocs.io/en/latest/index.html).

Source documentation is located in the `doc` directory. Instructions for building and viewing the documentation can be found in the corresponding `README` file.

## Examples

Comprehensive examples are provided as Jupyter Notebooks in the `examples` folder and documented in the [Examples section](https://pycram.readthedocs.io/en/latest/notebooks/intro.html). Refer to the examples folder's `README` for instructions on executing these notebooks.


