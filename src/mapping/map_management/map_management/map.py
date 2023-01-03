from xml.dom import minidom
from shapely.geometry import *
import shapely

import math
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData

import matplotlib.pyplot as plt


class Map():

    def __init__(self, map_string: str, res: float = 0.4):
        '''
        :param map_string: The map data, encoded as an xml doc
        :param res: The side length of the map grid's (square) cells, in meters
        '''
        self.dom_tree: minidom.Document = minidom.parseString(map_string)
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
        origin.position.x = self.west  # The minimum east value
        origin.position.y = self.south  # The minimum north value
        origin.position.z = 0.0  # Assume flat map. TODO: Is this dumb?
        # Default orientation is (x,y,z,w) = (0,0,0,1), which means "no rotation"
        metadata.origin = origin
        metadata.resolution = self.res

        self.grid = OccupancyGrid()
        self.grid.info = metadata

        road_elements = self.dom_tree.documentElement.getElementsByTagName(
            "road")
        for road in road_elements:
            road: minidom.Element

            road_name = str(road.getAttribute("name"))

            # Look through planView -> geometry
            geometries = road.getElementsByTagName(
                "planView")[0].getElementsByTagName("geometry")
            centerline_pts = []

            for geometry in geometries:
                geometry: minidom.Element
                # Is this geometry a line?
                if len(geometry.getElementsByTagName("line")) > 0:
                    x = float(geometry.getAttribute('x'))
                    y = float(geometry.getAttribute('y'))
                    hdg = float(geometry.getAttribute('hdg'))
                    length = float(geometry.getAttribute('length'))
                    # Geometry is a line. Add the start and endpoints.
                    start_pt = (x, y)
                    end_pt = (x + length*math.cos(hdg),
                              y + length*math.sin(hdg))
                    centerline_pts.append(start_pt)
                    centerline_pts.append(end_pt)
                # TODO: Add support for arcs

            centerline = LineString(centerline_pts)
            plt.plot([point[0] for point in centerline.coords],
                     [point[1] for point in centerline.coords])

            # now find left width
            lane_sections = road.getElementsByTagName("laneSection")

            # Assume only one lane section. TODO: add support for multiple.
            lsec: minidom.Element = lane_sections[0]

            left_sections = lsec.getElementsByTagName("left")

            if len(left_sections) == 0:
                print(f"{road_name} has no left section!")
                left_width = 0.0
            else:
                left_lanes = left_sections[0].getElementsByTagName("lane")
                left_width = 0.0
                for lane in left_lanes:
                    lane: minidom.Element
                    width_elem: minidom.Element = lane.getElementsByTagName("width")[
                        0]
                    lane_width = float(width_elem.getAttribute('a'))
                    left_width += lane_width

            right_sections = lsec.getElementsByTagName("right")

            if len(right_sections) == 0:
                print(f"{road_name} has no right section!")
                right_width = 0.0
            else:
                right_lanes = right_sections[0].getElementsByTagName("lane")
                right_width = 0.0
                for lane in right_lanes:
                    lane: minidom.Element
                    width_elem: minidom.Element = lane.getElementsByTagName("width")[
                        0]
                    lane_width = float(width_elem.getAttribute('a'))
                    right_width += lane_width

            left_bound = centerline.parallel_offset(
                left_width, side='left', resolution=1)
            right_bound = centerline.parallel_offset(
                right_width, side='right', resolution=1)

            plt.plot([point[0] for point in left_bound.coords],
                     [point[1] for point in left_bound.coords])
            plt.plot([point[0] for point in right_bound.coords],
                     [point[1] for point in right_bound.coords])

        plt.show()

        self.grid.data = np.ones(
            (metadata.height, metadata.width), dtype=int).flatten().tolist()

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
