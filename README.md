![](doc/images/pycram_logo.png)

# PyCRAM

[![Python Tests](https://github.com/cram2/pycram/actions/workflows/new-pycram-ci.yml/badge.svg)](https://github.com/cram2/pycram/actions/workflows/new-pycram-ci.yml/badge.svg)
[![Example Tests](https://github.com/cram2/pycram/actions/workflows/notebook-test-ci.yml/badge.svg)](https://github.com/cram2/pycram/actions/workflows/notebook-test-ci.yml/badge.svg)

PyCRAM is a plan executive framework for cognitive robotics that enables robust execution of high-level robot plans in partially observable environments. It provides modular, extensible tools for designing, implementing, and executing robot plans, facilitating integration of new functionalities and heterogeneous robot platforms.

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

## Live Demonstration

Test PyCRAM directly in your browser via our [Virtual Research Building](https://vib.ai.uni-bremen.de/page/fallschool/).

## Cross-Platform Plan Execution Example

PyCRAM supports executing identical high-level plans on different robot platforms. Below is a demonstration of the same plan running on the PR2 and IAI's Boxy:

|             Boxy             |             PR2            |
| :--------------------------: | :------------------------: |
| ![Boxy](doc/images/boxy.gif) | ![PR2](doc/images/pr2.gif) |

## Documentation

The full documentation is maintained at [Read the Docs](https://pycram.readthedocs.io/en/latest/index.html).

Source documentation is located in the `doc` directory. Instructions for building and viewing the documentation can be found in the corresponding `README` file.

## Examples

Comprehensive examples are provided as Jupyter Notebooks in the `examples` folder and documented in the [Examples section](https://pycram.readthedocs.io/en/latest/notebooks/intro.html). Refer to the examples folder's `README` for instructions on executing these notebooks.

## Virtual Building

Explore a variety of labs and demonstrations showcasing PyCRAM's capabilities on the [Labs page](https://vib.ai.uni-bremen.de/page/labs/) of our virtual building.

## Setting Up Your Own Lab

To create a custom lab in the virtual building, consult the `vrb` branch of this repository, which includes detailed setup instructions and templates.
