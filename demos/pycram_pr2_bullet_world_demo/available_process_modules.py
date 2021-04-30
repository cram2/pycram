from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.process_module import ProcessModule
from pr2_process_modules import PR2ProcessModules
from boxy_process_modules import BoxyProcessModules
from donbot_process_modules import DonbotProcessModules
from hsr_process_modules import HSRProcessModules
from rospy import logerr


def available_process_modules(desig):
    robot_name = robot_description.i.name
    if robot_name is 'pr2':  # todo rmv hardcoded robot names
        return PR2ProcessModules().initialized.available_process_modules(desig)
    elif robot_name is 'boxy':  # todo rmv hardcoded robot names
        return BoxyProcessModules().initialized.available_process_modules(desig)
    elif robot_name is 'donbot':  # todo rmv hardcoded robot names
        return DonbotProcessModules().initialized.available_process_modules(desig)
    elif robot_name is 'hsr': # todo rmv hardcoded robot names
        return HSRProcessModules().initialized.available_process_modules(desig)
    else:
        logerr('No Process Modules found for robot %s.', robot_name)


ProcessModule.resolvers.append(available_process_modules)
