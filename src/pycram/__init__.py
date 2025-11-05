__version__ = "2.0.0"


import logging

format = '%(levelname)s:%(filename)s::%(lineno)s %(funcName)s %(message)s'
logging.basicConfig(format=format)
logger = logging.Logger(__name__)
logger.setLevel(logging.INFO)