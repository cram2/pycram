![](doc/images/new_pycram_logo_clean.svg#gh-light-mode-only)
![](doc/images/new_pycram_logo_clean_dark.svg#gh-dark-mode-only)


# PyCRAM — High‑level feature highlights 

[![Python Tests](https://github.com/cram2/pycram/actions/workflows/new-pycram-ci.yml/badge.svg)](https://github.com/cram2/pycram/actions/workflows/new-pycram-ci.yml/badge.svg)
[![Example Tests](https://github.com/cram2/pycram/actions/workflows/notebook-test-ci.yml/badge.svg)](https://github.com/cram2/pycram/actions/workflows/notebook-test-ci.yml/badge.svg)



## Key Features

- 🧠⚡ Plan smarter, faster: compose and execute high‑level robot tasks with clear structure.
- 🛡️ Resilient execution: built‑in monitoring, introspection, and recovery workflows.
- 🎲 Embrace uncertainty: defer key choices to run time based on what the robot perceives.
- 🔁 Always have a plan B: automatic fallbacks when the first strategy doesn’t work.
- 🧰📐 Portable by design: separate intent from implementation so plans move across robots and setups.
- ✍️→⚙️ Intent vs. execution: describe what to achieve; swap how it’s achieved without rewriting plans.
- 🧪⚡ Iterate rapidly: simulate scenarios quickly, with or without graphics, before touching hardware.
- 👀📡 Understand at a glance: visualization that reveals goals, state, and environment clearly.
- 🤝 Flexible integration: connect to common robotics ecosystems when you need hardware and sensors.
- 🧠📚 Semantic reasoning: work with concepts and relationships, not just numbers and coordinates.
- 🧭🦾 Motion building blocks: ready‑to‑use navigation and manipulation skills to assemble bigger behaviors.
- ✅🧪 Continuous verification: tests and runnable examples to keep changes reliable.
- 🚀📗 Learn by doing: curated demos, notebooks, and virtual labs for fast onboarding.
- 📊🗃️ Improve with data: log outcomes, analyze performance, and refine strategies over time.
- 1️⃣➡️🤖🤖 One plan, many robots: run the same high‑level logic on different platforms.

## Why it matters
- ✅ Fewer brittle hacks, more reusable intent.
- ✅ Faster iteration in simulation, safer rollouts on real robots.
- ✅ Robust control that anticipates uncertainty—and recovers when reality disagrees.

## Installation

The recommended installation method is via `pip`:

```bash
pip install pycram-robotics
```

For an alternative installation from source, use the automated script:

```bash
curl -s https://raw.githubusercontent.com/cram2/pycram/dev/scripts/install.sh | bash
```

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


