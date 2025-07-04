from pycram.datastructures.enums import Arms, WorldMode
from pycram.datastructures.pose import PoseStamped
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.robot_plans.actions.transport import TransportingAction
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld
from pycrap.ontologies import Cup, Robot

extension = ObjectDescription.get_file_extension()


world = BulletWorld(WorldMode.GUI)
robot = Object("pr2", Robot, f"pr2{extension}", pose=PoseStamped.from_list([1, 2, 0]))

my_object = Object("jeroen_cup", Cup, "jeroen_cup.stl", pose=PoseStamped.from_list([3.4, 2, 1.0], [0, 0, 0, 1]))
my_arm = Arms.RIGHT
target_pose = PoseStamped.from_list([0.5, 0.2, 0.0], [0, 0, 0, 1])

# Create and execute action
transport_action = TransportingAction(
    object_=my_object,
    arm=my_arm,
    target_location=target_pose
)

# Perform the full transport plan
try:
    transport_action.perform()
except Exception as e:
    print(f"Transport failed: {e}")
