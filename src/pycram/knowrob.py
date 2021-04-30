from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
import rospy

id = 3

def query(query):
    rospy.wait_for_service('/rosprolog/query')
    service = rospy.ServiceProxy('/rosprolog/query', PrologQuery)
    global id
    response = service(1, str(id), query)
    id += 1
    print(response)

def solution(id):
    rospy.wait_for_service('/rosprolog/next_solution')
    service = rospy.ServiceProxy('/rosprolog/next_solution', PrologNextSolution)
    response = service(str(id))
    print(response)

def finish(id):
    rospy.wait_for_service('/rosprolog/finish')
    service = rospy.ServiceProxy('/rosprolog/finsh', PrologFinish)
    response = service(str(id))
