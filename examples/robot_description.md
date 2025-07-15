---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.3
  kernelspec:
    display_name: Python 3 (ipykernel)
    language: python
    name: python3
---

# Robot Description

(robot_description_header)=
The robot description contains semantic information about the robot which can not be extracted from the URDF in a
general way. This includes kinematic chains, end-effectors, cameras and their parameter, etc.

In general a Robot Description consists a number of different descriptions, these are:

* RobotDescription
* KinematicChainDescription
* EndEffectorDescription
* CameraDescription

In this example we will create a robot description step-by-step and describe the different components on the way. The
robot we will use as an example will be the PR2, the complete PR2 description can also be seen in
{meth}`pycram.robot_descriptions.pr2_description`.

## Robot Description Class

We start by creating an instance of the {class}`~pycram.robot_description.RobotDescription` class, this will serve as a
the main component to which all other descriptions will be added.

To initialize a {class}`~pycram.robot_description.RobotDescription` we need a few parameter which are:

* Name
* base_link
* torso_link
* torso_joint
* Path to a URDF file

```python
from pycram.robot_description import RobotDescription
from pycram.ros import get_ros_package_path

filename = get_ros_package_path('pycram') + '/resources/robots/' + "pr2" + '.urdf'
pr2_description = RobotDescription("pr2_example", "base_link", "torso_lift_link", "torso_lift_joint", filename)
```

## Kinematic Chain Description

The kinematic chain description describes a chain of links and joints of the robot which might be interesting when
working with the robot. An example of such a chain would be the arm of the robot, when programming for the robot it is
important to know which links and joints exactly make up the arm, however, these can not be extracted from the URDF
automatically.

The kinematic chain is based upon the URDF, meaning when initializing the description one only needs to specify the
first and last link of the chain.

We will now create the kinematic chain description for the right arm of the PR2. For initializing
the {class}`~pycram.robot_description.KinematicChainDescription` the following parameter are needed:

* Name
* first link
* last link
* URDF object
* Arm type

The arm type specifies which arm this kinematic chain describes, this is needed when one wants to access only the arms
of the robot.

```python
from pycram.robot_description import KinematicChainDescription
from pycram.datastructures.enums import Arms

right_arm = KinematicChainDescription("right", "torso_lift_link", "r_wrist_roll_link",
                                      pr2_description.urdf_object, arm_type=Arms.RIGHT)
```

The created {class}`~pycram.robot_description.KinematicChainDescription` can now be added to the robot description.

```python
pr2_description.add_kinematic_chain_description(right_arm)
```

## End Effector Description

Since kinematic chains only describe a moveable chain of links and joints like arms these do not represent end-effectors
which can be used to do manipulation tasks.

To represent end-effectors we will create an {class}`~pycram.robot_description.EndEffectorDescription` which contains the information of the respective
end-effector. When creating an {class}`~pycram.robot_description.EndEffectorDescription` we need the following parameter:

* Name
* first link
* tool_frame
* URDF object

You might have noticed that the end-efftor only has a first link but no last link, this is the case since end-effectors
are at the end of the arms. Therefore, all links and joints below a certain link can be seen as part of the
end-effector.

```python
from pycram.robot_description import EndEffectorDescription

right_gripper = EndEffectorDescription("right_gripper", "r_gripper_palm_link", "r_gripper_tool_frame",
                                       pr2_description.urdf_object)
```

The gripper can no be added to the previously created {class}`~pycram.robot_description.KinematicChainDescription`.

```python
right_arm.end_effector = right_gripper
```

## Camera Description

The camera description contains all parameters of a camera, which is mounted on the robot. The parameter for
the {class}`~pycram.robot_description.CameraDescription` are:

* Name
* Link name
* minimal height
* maximal height
* horizontal angle
* vertical angle

```python
from pycram.robot_description import CameraDescription
from pycram.datastructures.enums import Grasp

camera = CameraDescription("kinect_camera", "wide_stereo_optical_frame", 1.27,
                           1.60, 0.99483, 0.75049)
```

The finished camera description can now be added to the robot description.

```python
pr2_description.add_camera_description(camera)
```

## Grasps

Grasps define how a robot interacts with objects. The grasps defined in the robot description are set for each end-effector
individually. The predefined grasp used is the ApproachDirection.FRONT grasp of the robot. Based on this grasp, all other grasps are
generated.

```python
right_gripper.update_all_grasp_orientations([0, 0, 0, 1])

```

## Register Robot Description

Lastly, you need to register the robot description to the {class}`~pycram.robot_description.RobotDescriptionManager`. As you can see the code to
register the robot description has to be executed at the start of PyCRAM, if you put your file with the robot
description in the {class}`pycram.robot_descriptions` directory it will be executed upon the start of PyCRAM.

```python
from pycram.robot_description import RobotDescriptionManager

rdm = RobotDescriptionManager()
rdm.register_description(pr2_description)
```
