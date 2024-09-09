from pycram.utils import hacky_urdf_parser_fix
# Path to the URDF file
urdf_file_path = "/home/vee/robocup_workspaces/pycram_ws/src/pycram/resources/robots/turtlebot.urdf"

# Read the URDF file content as a string
with open(urdf_file_path, 'r') as urdf_file:
    urdf_content = urdf_file.read()

# Call hacky_urdf_parser_fix with the URDF content as a string
# Assuming hacky_urdf_parser_fix expects URDF content as a string and processes it
fixed_urdf_content = hacky_urdf_parser_fix(urdf_content)
f = open("../../resources/robots/turtlebot.urdf", "w")
f.write(fixed_urdf_content)