import logging

log = logging.getLogger("navigator") # Set name of logger

logging.basicConfig( # Set config of logger
    filename='log.log',
    #encoding='utf-8',
    level=logging.INFO
)