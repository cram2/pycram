from logging import getLogger, Logger
from functools import lru_cache
import logging

logging.basicConfig(format='[%(filename) %(lineno)s %(funcName)s] %(message)s')
logger = getLogger(__name__)

def logwarn(msg: str):
    logger.warn(msg)

def logerr(msg: str):
    logger.error(msg)

def loginfo(msg: str):
    logger.info(msg)

def logdebug(msg: str):
    logger.debug(msg)

@lru_cache(None)
def logwarn_once(msg: str):
    logger.warn(msg)

@lru_cache(None)
def logerr_once(msg: str):
    logger.error(msg)

@lru_cache(None)
def loginfo_once(msg: str):
    logger.info(msg)

@lru_cache(None)
def logdebug_once(msg: str):
    logger.debug(msg)