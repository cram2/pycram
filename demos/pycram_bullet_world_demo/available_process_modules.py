from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.process_module import ProcessModule
from .pr2_process_modules import PR2ProcessModulesSimulated, PR2ProcessModulesReal
from .boxy_process_modules import BoxyProcessModulesSimulated, BoxyProcessModulesReal
from .donbot_process_modules import DonbotProcessModulesSimulated, DonbotProcessModulesReal
from .hsr_process_modules import HSRProcessModulesSimulated, HSRProcessModulesReal
from rospy import logerr


def available_process_modules(desig):
    robot_name = robot_description.i.name
    robot_type = ProcessModule.robot_type
    type = desig.prop_value('type')

    if robot_name == 'pr2':
        if robot_type == 'simulated':
            return PR2ProcessModulesSimulated[type]
        elif robot_type == 'real':
            return PR2ProcessModulesReal[type]
        elif robot_type == "":
            logerr(f"No robot_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
        else:
            logerr(f"No Process Module could be found for robot {robot_name}")


    if robot_name == 'boxy':
        if robot_type == 'simulated':
            return BoxyProcessModulesSimulated[type]
        elif robot_type == 'real':
            return BoxyProcessModulesReal[type]
        elif robot_type == "":
            logerr(f"No robot_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
        else:
            logerr(f"No Process Module could be found for robot {robot_name}")


    if robot_name == 'donbot':
        if robot_type == 'simulated':
            return DonbotProcessModulesSimulated[type]
        elif robot_type == 'real':
            return DonbotProcessModulesReal[type]
        elif robot_type == "":
            logerr(f"No robot_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
        else:
            logerr(f"No Process Module could be found for robot {robot_name}")


    if robot_name == 'hsr':
        if robot_type == 'simulated':
            return HSRProcessModulesSimulated[type]
        elif robot_type == 'real':
            return HSRProcessModulesReal[type]
        elif robot_type == "":
            logerr(f"No robot_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
        else:
            logerr(f"No Process Module could be found for robot {robot_name}")



ProcessModule.resolvers.append(available_process_modules)
