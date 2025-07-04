"""
The `actions` package provides task-level behaviors and plans for the robot.

Structure:
- `core/` contains fundamental actions that directly use motion primitives or low-level control.
  These actions may also coordinate small helper actions.
- `composite/` contains high-level plans that compose core or other composite actions to achieve
  complex tasks. Composite actions do not directly use motion primitives.

Typical usage:
    from actions import transporting, pick_and_place
    from actions import cutting, placing
"""

# Expose composite actions at top level
from .composite.transporting import TransportActionDescription
from .core.grasping import PickUpActionDescription
from core.grasping import GraspingActionDescription


# # Expose core actions at top level
# from .core.cutting import CuttingAction as cutting
# from .core.placing import PlaceAction as placing
# from .core.grasping import GraspingAction as grasping
# from .core.pouring import PouringAction as pouring
# from .core.mixing import MixingAction as mixing
# from .core.wiping import WipingAction as wiping
# from .core.navigation import NavigationAction as navigation
# from .core.container import OpenAction as open_container, CloseAction as close_container
