from typing import List


def knowrob_string_to_pose(pose_as_string: str) -> List[float]:
    reference_frame = ""
    for i, char in enumerate(pose_as_string[1:-1]):
        if char == ",":
            break
        reference_frame += char
    pos, ori = pose_as_string[1+i+2:-2].split("],[")
    xyz = list(map(float, pos.split(",")))
    qxyzw = list(map(float, ori.split(",")))
    return xyz + qxyzw
