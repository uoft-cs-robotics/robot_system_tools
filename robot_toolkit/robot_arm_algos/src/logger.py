import logging

logger = logging.getLogger("my_module_name")
logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

# create formatter
formatter = logging.Formatter('%(levelname)-8s [%(filename)s:%(lineno)d] %(message)s')

# add formatter to ch
ch.setFormatter(formatter)
# add ch to logger
logger.addHandler(ch)

logger.propagate = False