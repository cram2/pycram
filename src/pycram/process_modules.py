from .robot_description import InitializedRobotDescription as robot_description




class ProcessModules():

    def __init__(self, navigation_PM, pick_up_PM, place_PM, accessing_PM, park_arms_PM,
                 looking_PM, opening_gripper_PM, closing_gripper_PM, detecting_PM, move_tcp_PM,
                 move_joints_PM, world_state_detecting_PM):
        self.navigation = navigation_PM
        self.pick_up = pick_up_PM
        self.place = place_PM
        self.accessing = accessing_PM
        self.park_arms = park_arms_PM
        self.move_head = looking_PM
        self.opening_gripper = opening_gripper_PM
        self.closing_gripper = closing_gripper_PM
        self.detecting = detecting_PM
        self.move_tcp = move_tcp_PM
        self.move_joints = move_joints_PM
        self.world_state_detecting = world_state_detecting_PM

    def available_process_modules(self, desig):
        """
        This method chooses the right process module for the given designator and returns it.
        :param desig: The designator for which a process module should be choosen.
        :return: The choosen process module
        """
        if desig.check_constraints([('type', 'moving')]):
            return self.navigation

        if desig.check_constraints([('type', 'pick-up')]):
            return self.pick_up

        if desig.check_constraints([('type', 'place')]):
            return self.place

        if desig.check_constraints([('type', 'accessing')]):
            return self.accessing

        if desig.check_constraints([('type', 'park-arms')]):
            return self.park_arms

        if desig.check_constraints([('type', 'looking')]):
            return self.move_head

        if desig.check_constraints([('type', 'opening-gripper')]):
            return self.opening_gripper

        if desig.check_constraints([('type', 'closing-gripper')]):
            return self.closing_gripper

        if desig.check_constraints([('type', 'detecting')]):
            return self.detecting

        if desig.check_constraints([('type', 'move-tcp')]):
            return self.move_tcp

        if desig.check_constraints([('type', 'move-arm-joints')]):
            return self.move_joints

        if desig.check_constraints([('type', 'world-state-detecting')]):
            return self.world_state_detecting
