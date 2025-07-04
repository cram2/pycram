import os
import re

SOURCE_FILE = "action_designator.py"
OUTPUT_DIR = "./actions_refactored"

# Grouping map: action class → target file
GROUPS = {
    "MoveTorsoAction": "robot_body.py",
    "SetGripperAction": "robot_body.py",
    "ParkArmsAction": "robot_body.py",
    "CarryAction": "robot_body.py",

    "GripAction": "grasping.py",
    "GraspingAction": "grasping.py",
    "ReachToPickUpAction": "grasping.py",
    "PickUpAction": "grasping.py",
    "ReleaseAction": "grasping.py",

    "PlaceAction": "placing.py",
    "MoveAndPlaceAction": "placing.py",

    "TransportAction": "transporting.py",
    "PickAndPlaceAction": "transporting.py",

    "NavigateAction": "navigation.py",
    "FaceAtAction": "navigation.py",
    "LookAtAction": "navigation.py",
    "SearchAction": "navigation.py",

    "OpenAction": "container.py",
    "CloseAction": "container.py",
}

# Prepare output
os.makedirs(OUTPUT_DIR, exist_ok=True)
for target in set(GROUPS.values()):
    with open(os.path.join(OUTPUT_DIR, target), 'w') as f:
        f.write("# Auto-generated split from action_designator.py\n\n")

# Read source
with open(SOURCE_FILE, 'r') as f:
    content = f.read()

# Find all @dataclass classes
pattern = re.compile(r"(@has_parameters\s*@dataclass\s*class\s+(\w+)[\s\S]*?)(?=@has_parameters|$)")
matches = pattern.findall(content)

# Sort and write
for full_def, class_name in matches:
    target_file = GROUPS.get(class_name)
    if target_file:
        with open(os.path.join(OUTPUT_DIR, target_file), 'a') as f:
            f.write(full_def.strip())
            f.write("\n\n")
    else:
        # For classes not mapped, optionally log or write to misc
        with open(os.path.join(OUTPUT_DIR, "misc.py"), 'a') as f:
            f.write(full_def.strip())
            f.write("\n\n")

print(f"✅ Refactor complete. Actions split into {OUTPUT_DIR}/")
