
class PinocchioConfig:
    error_threshold: float = 5e-4
    """
    The error threshold which needs to be reached for the goal to be considered as reached.
    """
    max_iterations: int = 5000
    """
    The maximum number of iterations for the inverse kinematics computation.
    """
    time_step: float = 1e-1
    """
    The time step for the integration of the joint configuration.
    """
    dampening_factor:float = 1e-12
    """
    The dampening factor for the inverse kinematics computation.
    """