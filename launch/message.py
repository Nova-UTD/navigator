import datetime
import time
from enum import IntEnum

from launch.text_colors import text_colors as colors
from launch.processes import change_process_status, get_node_from_process
from launch.logger import log

MessageLevels: dict = { # Dictionary to quickly assign level to int
    'INFO': 0,
    'WARN': 1,
    'WARNING': 2,
    'FATAL': 2,
    'ERROR': 2
}


class MessageLevel(IntEnum): 
    """
    Enumerated type to hold message level
    """
    INFO = 0
    WARN = 1
    FATAL = 2


class MessageType(IntEnum):
    """
    Enumerated type to hold what type of message a Message object is. 
    Each character represents a property inputted, as is below
    Process, Level, Timestamp, Node, Information
    """
    PLTNI = 0
    PTI = 1
    LPI = 2
    PI = 3


class Message(object):
    """
    Message object to hold console message properties
    """

    def print_items(self, is_output: bool) -> None:
        """
        Prints and logs formatted based on self data
        @param output[bool]     Boolean to output or not output to console
        """
        message_color: str = "" # Assign null color if nothing to change to
        if self.level == MessageLevel.INFO:
            change_process_status(self.process, "SUCCESS")
        elif self.level == MessageLevel.WARN:
            message_color = colors.WARNING
            change_process_status(self.process, "WARNING")
        elif self.level == MessageLevel.FATAL:
            message_color = colors.FAIL
            change_process_status(self.process, "FATAL")

        formatted_timestamp: datetime = str(datetime.datetime.fromtimestamp(self.timestamp).strftime('%H:%M:%S:%f')) # Get our message timestamp as a formatted date
        log_output: str =  f"[{str(self.level)[13:]}] [{formatted_timestamp}] [{self.process}]: {self.info}" # Holder for log output

        if is_output: # If we want to output then output to console
            print(
                f"{message_color} [{str(self.level)[13:]}] [{formatted_timestamp}] [{self.process}]: {self.info} {colors.ENDC}"
            )

        if self.level == MessageLevel.INFO:
            log.info(log_output)
        elif self.level == MessageLevel.WARN:
            log.warning(log_output)
        elif self.level == MessageLevel.FATAL:
            log.error(log_output)

    def confirm_level(self) -> None:
        """
        Processes data, to be called after data assignment is finished
        """
        int_level: int = MessageLevels[self.level] # Get level as int for intenum
        self.level = MessageLevel(int_level) # Set level to object from intenum
        self.process = get_node_from_process(self.process.split("-")[0]) # Set process to parsed process

    def __init__(self, msg_type: MessageType) -> None:
        """
        Initialization function for Message object
        @param msg_type[MessageType]    Message type to use for message
        """
        self.type: MessageType = msg_type
        self.process: str = None
        self.level: str = "INFO"
        self.timestamp = time.time() # Set timestamp to current time [close enough]
        self.node: str = None
        self.info: str = None