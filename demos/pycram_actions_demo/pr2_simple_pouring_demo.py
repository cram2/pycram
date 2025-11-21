from pycram.helper import perform, an
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.process_module import simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.robot_plans *


extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.GUI)
robot = Object("pr2", Robot, f"pr2{extension}", pose=PoseStamped.from_list([1, 2, 0]))
apartment = Object("apartment", Apartment, f"apartment{extension}")
robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])
VizMarkerPublisher()

pose = PoseStamped.from_list([2.4, 2, 1.0], [0, 0, 0, 1])
obj_tool_ = Object("jeroen_cup", PouringTool, "jeroen_cup.stl", pose=pose)
obj_target_ = Object("bowl", Bowl, "bowl.stl", pose=pose)

action_map = {"pouring": PouringAction}

# this is just bc we dont want to pick up for the demonstration of cutting and mixing
#obiously we need to change this here for different robots
tool_pose = PoseStamped.from_list([2.0449586673391935, 1.5384467778416917, 1.09705326966067], [0, 0, 0, 1])
obj_tool_.pose = tool_pose
location_pose = PoseStamped.from_list([1.7, 2, 0])
looking_pose = PoseStamped.from_list([2.5, 2, 0.97])
generic_obj_BO = BelieveObject(names=[obj_target_.name]).resolve()
tool_BO = BelieveObject(names=[obj_tool_.name]).resolve()

with simulated_robot:
    perform(an(ParkArmsAction([Arms.BOTH])))
    perform(an(NavigateAction([location_pose])))
    perform(an(MoveTorsoAction([TorsoState.HIGH])))

    #attach tool to robot
    tool_frame = RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame()
    World.current_world.robot.attach(child_object=obj_tool_, parent_link=tool_frame)

    perform(an(LookAtAction([looking_pose])))
    perform(an(PouringAction(generic_obj_BO, tool_BO, [Arms.RIGHT], None, angle=90)))
    perform(an(ParkArmsAction([Arms.BOTH])))