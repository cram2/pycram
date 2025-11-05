
from __future__ import print_function

import sys
import json
import logging
from typing import Callable

from typing_extensions import Dict

from ..ros import get_node_names, wait_for_service, get_service_proxy, ServiceException
try:
    from iai_apartment_kitchen_msgs.srv import Authenticateuser
except ModuleNotFoundError as e:
    pass

logger = logging.getLogger(__name__)

def init_kitchen_interface(func: Callable) -> Callable:
    """
    Tries to import the messages and initialize interface.
    """
    def wrapper(*args, **kwargs):
        if "iai_apartment_kitchen_msgs" not in sys.modules:
            logger.warning("Could not initialize the Apartment kitchen interface since the iai_apartment_kitchen_msgs are not imported")
            return

        if "/blum_control_service_server" in get_node_names():
            logger.info("Successfully initialized Apartment kitchen interface")
        else:
            logger.warning("Apartment kitchen is not running, could not initialize Apartment kitchen interface")
            return
        authenticate_user()

        return func(*args, **kwargs)
    return wrapper


def authenticate_user():
    """
    Authenticates a user at the bridge
    """
    logger.info('Waiting for authentication')
    wait_for_service('/blum_kitchen_server')

    try:
        kitchen_service = get_service_proxy('/blum_kitchen_server', Authenticateuser)
        resp1 = kitchen_service('authenticate_chk', 'pycram')
        if resp1.resp == 'FALSE':
            logger.info('Please press the cyan button on the kitchen gateway')
            resp1 = kitchen_service('authenticate', 'pycram')
    except ServiceException as e:
        print(f"Service call failed: {e}")

@init_kitchen_interface
def call_kitchen_service(command, argument):
    logger.info('Waiting for kitchen service')
    wait_for_service('/blum_kitchen_server')

    try:
        kitchen_service = get_service_proxy('/blum_kitchen_server', Authenticateuser)
        resp1 = kitchen_service(command, argument)
        return resp1.resp
    except ServiceException as e:
        print(f"Service call failed: {e}")


@init_kitchen_interface
def open_cabinet(kitchen_element):
    """
    Opens a cabinet of the apartment kitchen

    :param kitchen_element: The cabinet which should be opened
    """
    call_kitchen_service('open', kitchen_element)


@init_kitchen_interface
def close_cabinet(kitchen_element):
    """
    Closes a cabinet of the apartment kitchen, currently this is only possible for the "oberschrank"

    :param kitchen_element: Cabinet which should be closed
    """
    call_kitchen_service('close', kitchen_element)

@init_kitchen_interface
def modules_show():
    """
    Shows all possible cabinet modules that can be called
    :return:
    """
    return call_kitchen_service('modules_show', ' ')


@init_kitchen_interface
def kitchen_state() -> Dict[str, Dict[str, str]]:
    """
    Returns the state of the whole kitchen as a dictionary. The mapping is "name": {"state", "type"}

    :return: A dict describing the kitchen state
    """
    state = call_kitchen_service("kitchen_state_json"," ")
    state_dict = json.loads(state.replace("'", '"'))
    return state_dict