from logging import getLogger, Logger
from functools import lru_cache
import logging

format = '%(filename)s %(lineno)s %(funcName)s %(message)s'
logging.basicConfig(format=format)
logger = getLogger(__name__)
#
# def warning(msg: str):
#     logger.warning(msg)
#
# def error(msg: str):
#     logger.error(msg)
#
# def info(msg: str):
#     logger.info(msg)
#
# def debug(msg: str):
#     logger.debug(msg)
#
# @lru_cache(None)
# def logwarn_once(msg: str):
#     logger.warning(msg)
#
# @lru_cache(None)
# def logerr_once(msg: str):
#     logger.error(msg)
#
# @lru_cache(None)
# def loginfo_once(msg: str):
#     logger.info(msg)
#
# @lru_cache(None)
# def logdebug_once(msg: str):
#     logger.debug(msg)
#
# def set_logger_level(level: int):
#     """
#     Set the logging level for the logger.
#     :param level: The logging level to set.
#     """
#     logger.setLevel(level)