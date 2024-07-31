---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.2
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

# Robokudo interface in PyCRAM

This notebook should give you an example on how the RoboKudo interface in PyCRAM works. We will go over how to use the
interface, how it is implemented and what can be extended.

First, you need to install RoboKudo by following the installation
instructions [here](https://robokudo.ai.uni-bremen.de/installation.html).

RoboKudo depends on a pipline of so-called annotators to process images, depending on your use-case the used annotators
will change. But for this simple example we can use the demo pipeline from
the [tutorial](https://robokudo.ai.uni-bremen.de/tutorials/run_pipeline.html). You can start RoboKudo by calling

```
rosrun robokudo main.py _ae=query
```

To get a stream of images to process you need the test bag file,
from [here](https://robokudo.ai.uni-bremen.de/_downloads/6cd3bff02fd0d7a3933348060faa42fc/test.bag). You can run this
bag file with the following command in the directory where the bag file is.

```
rosbag play test.bag --loop
```

There should now be two windows which show you the result of the annotators. You switch between different annotators by
using the arrow keys.

## How to use the RoboKudo interface in PyCRAM

Everything related to the RoboKudo interface can be found in the file {class}`pycram.external_interfaces.robokudo`. The
most important method of this file is {meth}`~pycram.external_interfaces.robokudo.query` which takes a PyCRAM object designator and calls RoboKudo to try to
find a fitting object in the camera view. The other methods are just helper for constructing messages.

Since we are only working with the demo pipeline we will only see how the interface functions but not actually perceive
objects in the images.

```python
from pycram.external_interfaces import robokudo
from pycram.designators.object_designator import *
from pycram.enums import ObjectType

object_desig_desc = ObjectDesignatorDescription(types=[ObjectType.BOWL])
robokudo.query(object_desig_desc)
```

There was no object detected since the pipline we are using for this example only returns an empty message. However,
this should give you an impression on how the interface works.

## How the RoboKudo interface in PyCRAM works

The interface to RoboKudo is designed around the ROS service that RoboKudo provides. The interface takes an
ObjectDesignatorDescription which is PyCRAMs symbolic representation of objects and converts it to a RoboKudo
ObjectDesignator, the RoboKudo ObjectDesignator is then send to RoboKudo.

The result from this is a list of RoboKudo ObjectDesignators which are possible matches that were found in the camera
FOV. Each of these ObjectDesignators has a list of possible poses that are the result of different pose estimators (
currently PyCRAM picks the pose from 'ClusterPoseBBAnnotator' from the list of possible poses).
PyCRAM then transforms all possible poses for the found Objects to 'map' frame and returns them as a dictionary.

When using the interface the decorator {meth}`~pycram.external_interfaces.robokudo.init_robokudo_interface` should be added to all methods that want to send
queries to RoboKudo. This decorator makes sure that RoboKudo is running and creates an action client which can be used
via the global variable {attr}`~pycram.external_interfaces.robokudo.robokudo_action_client`.

## How to extend the RoboKudo interface in PyCRAM

At the moment the RoboKudo interface is tailored towards a specific scenario, in which only two types of objects need to
be detected. The distinction is mainly made by the difference in color, which is written in the RoboKudo
ObjectDesignator depending on the ObjectType of the PyCRAM ObjectDesignator.

The main point for extension would be to make the interface more universal and extend it to work with other pipelines
for example for human detection.
