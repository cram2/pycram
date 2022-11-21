import rospy

from .robot_description import InitializedRobotDescription as robot_description
from .bullet_world import BulletWorld
from giskardpy.python_interface import GiskardWrapper
from geometry_msgs.msg import PoseStamped
from giskard_msgs.srv import UpdateWorldRequest, WorldBody


giskard_wrapper = GiskardWrapper()
giskard_update_service = rospy.ServiceProxy("/giskard/update_world", UpdateWorld)


""" Belive state managment betwenn pycram and giskard """

def initial_adding_objects():
    groups = giskard_wrapper.get_group_names()
    for obj in BulletWorld.current_bullet_world.objects:
        name = obj.name + "_" + str(id)
        if name not in groups:
            spawn_object(obj)

def sync_worlds():
    initial_adding_objects()
    for obj in BulletWorld.current_bullet_world.objects:
        if obj == BulletWorld.robot:
            continue
        update_pose(obj)

def update_pose(object):
    return giskard_wrapper.update_group_pose(objet.name + "_" + str(object.id), \
            make_pose_stamped(object.get_position_and_orientation()))

def spawn_object(object):
    spawn_urdf(object.name + "_" + str(object.id), object.path, object.get_position_and_orientation())

def spawn_urdf(name, urdf_path, pose):
    urdf_string = ""
    with open(urdf_path) as f:
        urdf_string = f.read()
    pose_stamped = make_pose_stamped(pose)

    return giskard_wrapper.add_urdf(name, urdf_sting, pose_stamped, "map")

""" Sending Goals to Giskard """

def achive_joint_goal(joint, position):
    sync_worlds()
    giskard_wrapper.set_joint_goal({joint: position})
    return giskard_wrapper.plan_and_exeute()

def achive_cartisan_goal(goal_pose, tip_link, root_link):
    sync_worlds()
    giskard_wrapper.set_cart_goal(make_pose_stamped(goal_pose), tip_link, root_link)
    return giskard_wrapper.plan_and_exeute()


""" Creating ROS messages """

def make_world_body(object):
    urdf_string = ""
    with open(object.path) as f:
        urdf_sting = f.read()
    urdf_body = WorldBody()
    urdf_body.type = WorldBody.URDF_BODY
    urdf_body.urdf = urdf_string

    return urdf_body

def make_pose_stamped(position_and_orientation):
    po, qu = position_and_orientation

    pose = PoseStamped()
    pose.header.stamp = rospy.Time().now()
    pose.header.frame_id = "map"

    pose.pose.position.x = po[0]
    pose.pose.position.y = po[1]
    pose.pose.position.z = po[2]

    pose.orientation.x = qu[0]
    pose.orientation.y = qu[1]
    pose.orientation.z = qu[2]
    pose.orientation.w = qu[3]

    return pose
