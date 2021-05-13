from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
import rospy


def query(query, id, mode=1):
    rospy.wait_for_service('/rosprolog/query')
    service = rospy.ServiceProxy('/rosprolog/query', PrologQuery)
    response = service(mode, id, query)
    #print(response)

def solution(id):
    rospy.wait_for_service('/rosprolog/next_solution')
    service = rospy.ServiceProxy('/rosprolog/next_solution', PrologNextSolution)
    response = service(str(id))
    #print(response.solution)
    return response.solution

def finish(id):
    rospy.wait_for_service('/rosprolog/finish')
    service = rospy.ServiceProxy('/rosprolog/finish', PrologFinish)
    response = service(str(id))


def query_1(query_string):
    """
    Query which only returns the first result and then finishes the query.
    """
    query(query_string, "tmp")
    sol = solution("tmp")
    finish("tmp")
    return sol

def query_all(query_string):
    query(query_string, "tmp", 0)
    sol = solution("tmp")
    finish("tmp")
    return sol

def get_object_pose(object_class):
    query(f"owl_individual_of(Object, knowrob:'{object_class}'), current_object_pose(Object, Pose).",
        "object_pose")
    solution = solution("object_pose")
    return solution

def _make_dict_from_msg(msg):
    split = msg.split("\n")
    if split == [msg]:
        return _make_dict_from_msg_single(msg)
    dict = {}
    curr_elem = None
    for line in split:
        if ":" in line and "http" not in line:
            curr_elem = line.replace(":", "").replace("[", "")
            curr_elem = "".join(curr_elem.split()).replace("\"", "")
            dict[curr_elem] = []
        if "http" in line:
            dict[curr_elem].append("".join(line.split()).replace(",", "").replace("\"", ""))
    return dict

def _make_dict_from_msg_single(msg):
    split = msg.split("\"")
    dict = {}
    dict[split[1]] = split[3]
    return dict


def find_shelf_pose():
    shelves = query_1("get_all_shelves(A)")
    shelves = _make_dict_from_msg(shelves)['A']
    poses = {}
    for shelf in shelves:
        translation = query_1(f"holds('{shelf}', knowrob:pose, Trans)")
        translation = _make_dict_from_msg_single(translation)
        pose = query_1(f"holds('{translation['Trans']}', knowrob:translation, P)")
        pose = _make_dict_from_msg_single(pose)
        poses[shelf] = pose['P']
    return poses
