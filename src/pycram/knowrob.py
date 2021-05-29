from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish
from knowrob_refills.knowrob_wrapper import KnowRob
from rosprolog_client import Prolog, PrologException
import rospy
<<<<<<< HEAD
=======

def startup_knowrob():
    db = '~/workspace/whole_store_filled_neem/roslog'
    knowrob = KnowRob(initial_mongo_db=db,
                      clear_roslog=True,
                      republish_tf=True,
                      neem_mode=False)
    # knowrob = KnowRob(initial_mongo_db=None,
    #                   clear_roslog=False)
    shelf_ids = knowrob.get_shelf_system_ids(False)
    print(shelf_ids)
    for shelf_id in shelf_ids:
        print('shelf center frame id {}'.format(knowrob.get_object_frame_id(shelf_id)))
        print('shelf corner frame id {}'.format(knowrob.get_perceived_frame_id(shelf_id)))
    return knowrob

def get_all_shelves():
    prolog = Prolog()
    return prolog.once("get_all_shelves(A)")['A']

def find_shelf_pose(shelf):
    prolog = Prolog()
    return prolog.once(f"get_pose_in_desired_reference_frame('{shelf}', 'map', T, R)")
>>>>>>> 3a540e7... added rosprolog_client and refactored code
