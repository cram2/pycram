import pybullet as p
import rospy
import pycram.ik as ik
from pycram import bullet_world, bullet_world_reasoning
from pycram.robot_description import InitializedRobotDescription as robot_description
from demos.pycram_bullet_world_demo import pepper_process_modules
from demos.pycram_bullet_world_demo import motion_designators
from demos.pycram_bullet_world_demo import available_process_modules
from pycram.designator import MotionDesignator
from pycram.process_module import ProcessModule
from pycram.helper import _apply_ik


world = bullet_world.BulletWorld()

robot = bullet_world.Object("pepper", "robot", "resources/pepper.urdf", [0, 0, 1], [0, 0, 0, 1])
floor = bullet_world.Object("floor", "environment", "resources/plane.urdf")
#room = bullet_world.Object("dm_room", "environment", "resources/dm_room.urdf")
#box = bullet_world.Object("box", "object", "box.urdf",[3, -0.5, 1.5])
bullet_world.BulletWorld.robot = robot

def get_inv(target, arm):
    joints = robot_description.i._safely_access_chains(arm).joints
    return ik.request_ik('base_link', robot_description.i.get_tool_frame(arm), [target, [0, 0, 0, 1]], robot, joints )

get_inv([0.2, 0.3, 0.9], 'left')
