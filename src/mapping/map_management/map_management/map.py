from xml.dom import minidom
from shapely.geometry import Point

import math
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData


class Map():

    def __init__(self, map_string: str, res: float = 0.4):
        '''
        :param map_string: The map data, encoded as an xml doc
        :param res: The side length of the map grid's (square) cells, in meters
        '''
        self.dom_tree = minidom.parseString(map_string)
        self.res = res

        # roads = dom_tree.documentElement.getElementsByTagName("road")
        self._parse_header_()
        self._generate_map_grid_()

    def _generate_map_grid_(self):
        # 1. Create map metadata
        metadata = MapMetaData()
        metadata.height = math.ceil((self.north-self.south) / self.res)
        metadata.width = math.ceil((self.east-self.west) / self.res)

        # From the metadata msg spec: The origin of the map [m, m, rad].
        # This is the real-world pose of the bottom left corner of cell (0,0) in the map.
        # Navigator uses the ENU coordinate system, where +x is east, +y in north, and +z is up
        origin = Pose()
        origin.position.x = self.west # The minimum east value
        origin.position.y = self.south # The minimum north value
        origin.position.z = 0.0 # Assume flat map. TODO: Is this dumb?
        # Default orientation is (x,y,z,w) = (0,0,0,1), which means "no rotation"
        metadata.origin = origin
        metadata.resolution = self.res

        self.grid = OccupancyGrid()
        self.grid.info = metadata
        self.grid.data = np.ones((metadata.height, metadata.width), dtype=int).flatten().tolist()

    def _parse_header_(self):
        header: minidom.Element = self.dom_tree.documentElement.getElementsByTagName("header")[
            0]

        self.north = float(header.getAttribute('north'))
        self.south = float(header.getAttribute('south'))
        self.east = float(header.getAttribute('east'))
        self.west = float(header.getAttribute('west'))

        # Find our geoReference tag, which has contents like this:
        # "<![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs ]]>"
        geo_reference: str = header.getElementsByTagName('geoReference')[
            0].childNodes[0].data
        geo_reference_parts = geo_reference.split('+')

        for part in geo_reference_parts:
            if part.startswith('lon_0'):
                self.lon0 = float(part[6:])
            elif part.startswith('lat_0'):
                self.lat0 = float(part[6:])
            elif part.startswith('x_0'):
                self.x0 = float(part[4:])
            elif part.startswith('y_0'):
                self.y0 = float(part[4:])
