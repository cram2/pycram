from pycram.ros import ros_tools
from pycram.designators.action_designator MoveTorsoAction, PickUpAction, \
    NavigateAction
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.enums import Arms, Grasp, GripperState, WorldMode, ObjectType
from pycram.world_concepts.world_object import Object
from pycram.process_module import simulated_robot
import pycram.tasktree
from pycram.tasktree import with_tree
import pycram.failures
from pycram.designators import object_designator, action_designator
from anytree import RenderTree
from pycram.robot_description import RobotDescription,RobotDescriptionManager

from pycram.worlds.bullet_world import BulletWorld


rdm = RobotDescriptionManager()
rdm.load_description("pr2")
world = BulletWorld(WorldMode.GUI)
robot_obj = Object(RobotDescription.current_robot_description.name, ObjectType.ROBOT, RobotDescription.current_robot_description.name +".urdf", pose=PoseStamped.from_list([1, 2, 0]))
milk_obj = Object("milk", ObjectType.MILK,"milk.stl", pose=PoseStamped.from_list([1.3, 1, 0.9]))
@with_tree
def plan():
    object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
    description = action_designator.PlaceAction(object_description, [PoseStamped.from_list([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
    with simulated_robot:
        NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
        MoveTorsoActionPerformable(0.3).perform()
        PickUpAction(object_description.resolve(), Arms.LEFT, Grasp.FRONT).perform()
        description.resolve().perform()


plan()
tt = pycram.tasktree.task_tree.root

print(RenderTree(tt))

