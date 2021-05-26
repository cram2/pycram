from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.process_module import ProcessModule
from pr2_process_modules import PR2ProcessModulesSimulated, PR2ProcessModulesReal
from boxy_process_modules import BoxyProcessModules
from donbot_process_modules import DonbotProcessModules
from hsr_process_modules import HSRProcessModules
from rospy import logerr


def available_process_modules(desig):
    robot_name = robot_description.i.name
    robot_type = ProcessModule.robot_type

    if robot_name == 'pr2':  # todo rmv hardcoded robot names
        if robot_type == 'simulated':
            return PR2ProcessModulesSimulated().initialized.available_process_modules(desig)
        else:
            return PR2ProcessModulesReal().initialized.available_process_modules(desig)
    elif robot_name == 'boxy':  # todo rmv hardcoded robot names
        if robot_type == 'simulated':
            return BoxyProcessModules().initialized_simulated.available_process_modules(desig)
        else:
            return BoxyProcessModules().initialized_real.available_process_modules(desig)
    elif robot_name == 'donbot':  # todo rmv hardcoded robot names
        if robot_type == 'simulated':
            return DonbotProcessModules().initialized_simulated.available_process_modules(desig)
        else:
            return DonbotProcessModules().initialized_real.available_process_modules(desig)
    elif robot_name == 'hsr': # todo rmv hardcoded robot names
        if robot_type == 'simulated':
            return HSRProcessModules().initialized_simulated.available_process_modules(desig)
        else:
            return HSRProcessModules().initialized_real.available_process_modules(desig)
    else:
        logerr('No Process Modules found for robot %s.', robot_name)


ProcessModule.resolvers.append(available_process_modules)
