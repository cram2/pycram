from pycram.language import SequentialPlan---
jupyter:
  jupytext:
    text_representation:
      extension: .md
      format_name: markdown
      format_version: '1.3'
      jupytext_version: 1.16.3
  kernelspec:
    display_name: Python 3
    language: python
    name: python3
---

# Motion Designator

Motion designators are similar to action designators, but unlike action designators, motion designators represent atomic
low-level motions. Motion designators only take the parameter that they should execute and not a list of possible
parameters, like the other designators. Like action designators, motion designators can be performed. Performing a motion
designator verifies the parameter and passes the designator to the respective process module.

Since motion designators perform a motion on the robot, we need a robot which we can use. Therefore, we will create a
BulletWorld as well as a PR2 robot.

```python
from pycram.testing import setup_world
from pycram.datastructures.dataclasses import Context
from semantic_digital_twin.robots.pr2 import PR2


world = setup_world()
pr2_view = PR2.from_world(world)

context = Context(world, pr2_view)
```

## Move

Move is used to let the robot drive to the given target pose. Motion designator are used in the same way as the other
designator, first create a description then resolve it to the actual designator and lastly, perform the resolved
designator.

```python
from pycram.datastructures.pose import PoseStamped
from pycram.robot_plans.motions import MoveMotion
from pycram.process_module import simulated_robot
from pycram.language import SequentialPlan

with simulated_robot:
    motion_description = MoveMotion(target=PoseStamped.from_list([1, 0, 0], [0, 0, 0, 1]))

    SequentialPlan(context, motion_description).perform()
```

## MoveTCP

MoveTCP is used to move the tool center point (TCP) of the given arm to the target position specified by the parameter.
Like any designator we start by creating a description and then resolving and performing it.

```python
from pycram.robot_plans.motions import MoveTCPMotion
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import Arms

with simulated_robot:
    motion_description = MoveTCPMotion(target=PoseStamped.from_list([0.5, 0.6, 0.6], [0, 0, 0, 1], world.root), arm=Arms.LEFT)

    SequentialPlan(context, motion_description).perform()
```

## Looking

Looking motion designator adjusts the robot state such that the cameras point towards the target pose. Although this
motion designator takes the target as position and orientation, in reality only the position is used.

```python
from pycram.robot_plans.motions import LookingMotion
from pycram.process_module import simulated_robot

with simulated_robot:
    motion_description = LookingMotion(target=PoseStamped.from_list([1, 1, 1], [0, 0, 0, 1], world.root))

    SequentialPlan(context, motion_description).perform()
```

## Move Gripper

Move gripper moves the gripper of an arm to one of two states. The states can be {attr}`~pycram.datastructures.enums.GripperState.OPEN`  and {attr}`~pycram.datastructures.enums.GripperState.CLOSE`, which open
and close the gripper respectively.

```python
from pycram.robot_plans.motions import MoveGripperMotion
from pycram.process_module import simulated_robot
from pycram.datastructures.enums import Arms, GripperState

with simulated_robot:
    motion_description = MoveGripperMotion(motion=GripperState.OPEN, gripper=Arms.LEFT)

    SequentialPlan(context, motion_description).perform()
```

## Detecting

This is the motion designator implementation of detecting, if an object with the given object type is in the field of
view (FOV) this motion designator will return a list of  object designators describing the objects. It is important to specify the 
technique and state of the detection. You can also optional specify a region in which the object should be detected.


Since we need an object that we can detect, we will spawn a milk for this.

```python
# from pycram.robot_plans.motions import DetectingMotion, LookingMotion
# from pycram.process_module import simulated_robot
# from pycram.datastructures.pose import PoseStamped
# from pycram.datastructures.enums import DetectionTechnique, DetectionState
# from pycram.designators.object_designator import BelieveObject
# 
# with simulated_robot:
#     LookingMotion(target=PoseStamped.from_list([1.5, 0, 1], [0, 0, 0, 1])).perform()
# 
#     motion_description = DetectingMotion(technique=DetectionTechnique.TYPES,
#                                          state=DetectionState.START,
#                                          object_designator_description=BelieveObject(types=[Milk]).resolve(),
#                                          region=None)
# 
#     obj = motion_description.perform()
# 
#     print(obj[0])
```

## Move Arm Joints

This motion designator moves one or both arms. Movement targets are a dictionary with joint name as key and target pose
as value.

```python
from pycram.robot_plans.motions import MoveArmJointsMotion
from pycram.process_module import simulated_robot

with simulated_robot:
    motion_description = MoveArmJointsMotion(right_arm_poses={"r_shoulder_pan_joint": -0.7})

    SequentialPlan(context, motion_description).perform()
```

## Move Joints

Move joints can move any number of joints of the robot, the designator takes two lists as parameter. The first list are
the names of all joints that should be moved and the second list are the positions to which the joints should be moved.

```python
from pycram.robot_plans.motions import MoveJointsMotion
from pycram.process_module import simulated_robot

with simulated_robot:
    motion_description = MoveJointsMotion(names=["torso_lift_joint", "r_shoulder_pan_joint"], positions=[0.2, -1.2])

    SequentialPlan(context, motion_description).perform()
```