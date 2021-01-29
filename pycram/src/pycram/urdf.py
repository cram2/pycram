import rospkg
import rospy
import sys


def correct_urdf(urdf_name):
    urdf_string = rospy.get_param(urdf_name)
    new_urdf_string = ""
    for line in urdf_string.split('\n'):
        if "package://" in line:
            s = line.split('//')
            s1 = s[1].split('/')
            path = r.get_path(s1[0])
            line = line.replace("package://" + s1[0], path)
        new_urdf_string += line + '\n'

    return new_urdf_string


if __name__ == "__main__":
    r = rospkg.RosPack()
    node = rospy.init_node('listen', anonymous=True)
    urdf_name = sys.argv[1]
    save_location = sys.argv[2]
    new_urdf = correct_urdf(urdf_name)
    f = open(save_location, "w")
    f.write(new_urdf)
    f.close()
