from launch.message import MessageType, Message

def parse_PLTNI(message: str) -> Message: # [ObstacleZonerLaunch-9] [INFO] [1666749993.288106915] [planning.obstacle_zoner]: Start ob
    """
    Parses string of pattern [PROCESS] [LEVEL] [TIMESTAMP] [NODE]: INFO
    @param message      Console message of type PLTNI to parse
    @return Message     Populated message object to return
    """
    # Parse message for information
    split_messages: list = message.split(' ', 4)
    process: str = split_messages[0][1:].split("]")[0]
    level: str = split_messages[1][1:].split("]")[0]
    timestamp: str = split_messages[2][1:].split("]")[0]
    node: str = split_messages[3][1:].split("]")[0]
    info: str = message.split(':')[1][1:]

    # Create message object from parsed information
    msg: Message = Message(MessageType.PLTNI)
    msg.process = process
    msg.level = level
    msg.node = node
    msg.info = info
    
    return msg


def parse_PTI(message: str) -> Message: # [static_transform_publisher-5] 1666749993.283289 [0] static_tra
    """
    Parses string of pattern [PROCESS] [TIMESTAMP]: INFO
    @param message      Console message of type PTI to parse
    @return Message     Populated message object to return
    """
    # Parse message for information
    split_messages: list = message.split(' ', 3)
    process: str = split_messages[0][1:].split("]")[0]
    timestamp: str = split_messages[1].split("]")[0]
    info: str = message.split(':')[1][1:]

    # Create message object from parsed information
    msg = Message(MessageType.PTI)
    msg.process = process
    msg.info = info
    
    return msg


def parse_LPI(message: str) -> Message: # [INFO] [ZoneFusionLaunch-8]: process starte
    """
    Parses string of pattern [LEVEL] [PROCESS]: INFO
    @param message      Console message of type LPI to parse
    @return Message     Populated message object to return
    """
    # Parse message for information
    split_messages: list = message.split(' ', 3)
    level: str = split_messages[0][1:].split("]")[0]
    process: str = split_messages[1][1:].split("]")[0]
    info: str = message.split(':')[1][1:]

    # Create message object from parsed information
    msg: Message = Message(MessageType.LPI)
    msg.level = level
    msg.process = process
    msg.info = info
    return msg


def parse_PI(message: str) -> Message: # [robot_state_publisher-6] Link ardu
    """
    Parses string of pattern [PROCESS]: INFO
    @param message      Console message of type PI to parse
    @return Message     Populated message object to return
    """
    # Parse message for information
    split_messages = message.split(' ', 2)
    process = split_messages[0][1:].split("]")[0]
    info = message.split('] ')[1]

    # Create message object from parsed information
    msg: Message = Message(MessageType.PI)
    msg.process = process
    msg.info = info
    return msg