import logging
from datetime import timedelta

import rospy
from tf.transformations import quaternion_from_euler

from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, Arms
from pycram.datastructures.pose import Pose
from pycram.datastructures.world import UseProspectionWorld, World
from pycram.designators.action_designator import ParkArmsAction, MoveTorsoAction, TransportAction, NavigateAction, \
    LookAtAction, DetectAction
from pycram.designators.object_designator import BelieveObject
from pycram.failures import PerceptionObjectNotFound, NavigationGoalNotReachedError
from pycram.process_module import simulated_robot, with_simulated_robot, real_robot
from pycram.ros_utils.robot_state_updater import WorldStateUpdater
from pycram.world_concepts.world_object import Object
from pycram.worlds.multiverse import Multiverse


rospy_logger = logging.getLogger('rosout')
rospy_logger.setLevel(logging.DEBUG)


@with_simulated_robot
def move_and_detect(obj_type: ObjectType, pick_pose: Pose):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig


world = Multiverse()
robot = Object('pr2', ObjectType.ROBOT, f'pr2.urdf', pose=Pose([1.3, 2.6, 0.01]))
WorldStateUpdater(tf_topic="/tf", joint_state_topic="/real/pr2/joint_states", update_rate=timedelta(seconds=2),
                  world=world)
apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment.urdf")
milk = Object("milk", ObjectType.MILK, f"milk.xml", pose=Pose([0.4, 2.6, 1.34],
                                                              [1, 0, 0, 0]),
              color=Color(1, 0, 0, 1))

# apartment.set_joint_position("fridge_door1_joint", 1.5707963267948966)

fridge_base_pose = apartment.get_link_pose("fridge_base")
fridge_base_pose.position.z -= 0.12
fridge_base_pose.position.x += 0.16
fridge_base_pose.position.y += -0.1
milk.set_pose(fridge_base_pose, base=True)


robot_desig = BelieveObject(names=[robot.name])
apartment_desig = BelieveObject(names=[apartment.name])


with real_robot:

    # Transport the milkMoveGripperMotion
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.2]).resolve().perform()

    NavigateAction(target_locations=[Pose([1.4, 3.15, 0.01], quaternion_from_euler(0, 0, 3.14))]).resolve().perform()

    LookAtAction(targets=[Pose(milk.get_position_as_list())]).resolve().perform()

    milk_desig = DetectAction(BelieveObject(types=[milk.obj_type])).resolve().perform()

    TransportAction(milk_desig, [Arms.LEFT], [Pose([2.4, 3, 1.02])]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

world.exit()
