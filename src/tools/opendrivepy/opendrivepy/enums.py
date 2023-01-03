from enum import Enum, auto

# https://www.asam.net/index.php?eID=dumpFile&t=f&f=4422&token=e590561f3c39aa2260e5442e29e93f6693d1cccd#top-EAID_9692E2F3_4895_4ce6_A84E_FB1297B0B58E


class LaneType(Enum):
    HOV = 1
    BIDIRECTIONAL = auto()
    BIKING = auto()
    BUS = auto()
    CURB = auto()
    DRIVING = auto()
    MEDIAN = auto()
    NONE = auto()
    PARKING = auto()
    RESTRICTED = auto()
    SHOULDER = auto()
    SIDEWALK = auto()
    STOP = auto()
    OTHER = auto()  # This list is missing a few types


class RoadType(Enum):
    BICYCLE = 1
    LOW_SPEED = auto()
    JUNCTION = auto()
    MOTORWAY = auto()
    PEDESTRIAN = auto()
    RURAL = auto()
    TOWN_ARTERIAL = auto()
    TOWN_COLLECTOR = auto()
    TOWN_EXPRESSWAY = auto()
    TOWN_LOCAL = auto()
    TOWN_PLAYSTREET = auto()
    TOWN_PRIVATE = auto()
    TOWN = auto()
    UNKNOWN = auto()
    NONE = auto
