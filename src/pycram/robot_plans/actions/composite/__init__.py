"""
The `composite` package contains high-level plans that compose other actions.

These actions:
- Combine core actions and/or other composite actions.
- Do not directly control motions or low-level controllers.
- Represent complete task strategies (e.g., transporting, pick and place).

Example actions in this package:
- TransportingAction
- PickAndPlaceAction
"""

# from .transporting import TransportingAction
# from .pick_and_place import PickAndPlaceAction