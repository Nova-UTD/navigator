from __future__ import annotations

import math
import numpy as np
from shapely.geometry import LineString, Point, Polygon, MultiPolygon
from shapely.prepared import prep, PreparedGeometry
from shapely.strtree import STRtree
from tqdm import tqdm, trange

import xml.etree.ElementTree as ET

# Strictly for testing/debug
import matplotlib.pyplot as plt

from .enums import LaneType, RoadType
from .header import Header
from .lane import Lane
from .lane_section import LaneSection
from .road import Road


class Map:
    """The root OpenDRIVE object.
    """

    def __init__(self, map_str: str, grid_resolution=1.0):
        """Initialize the map and add roads, lanes, etc to it

        Args:
            map_str (str): XML OpenDRIVE map data
        """
        self._root_ = ET.fromstring(map_str)

        self.header = self._parse_header_(self._root_)
        self.roads = self._parse_roads_(self._root_)
        self.road_tree = self._build_road_area_tree_()

        # p = Point(42, -220)
        # print(self.road_tree.intersects(p))

        self.road_grid = self._build_road_grid_(
            grid_resolution, self.road_tree)

        # result = self.road_tree.query(
        #     Point(42, -220), predicate="within").tolist()
        # result_shape = self.road_tree.geometries.take(result).tolist()
        # print(self.road_tree.geometries)

        self.controllers = []
        self.junctions = []

    def _build_road_grid_(self, cell_size: float, tree: STRtree) -> np.array:
        width_m = self.header.east_bound - self.header.west_bound
        width = math.ceil(width_m/cell_size)

        height_m = self.header.north_bound - self.header.south_bound
        height = math.ceil(height_m/cell_size)

        x0 = self.header.west_bound
        x1 = self.header.east_bound
        y0 = self.header.south_bound
        y1 = self.header.north_bound

        Y, X = np.mgrid[y0:y1:height*1j, x0:x1:width*1j]
        points = []
        X = X.flatten()
        Y = Y.flatten()

        tqdm_range = trange(0, len(X))
        tqdm_range.set_description("Creating Points")
        for i in tqdm_range:
            points.append(Point(X[i], Y[i]))

        plt.show()

        hits = []

        progress = tqdm(points)
        progress.set_description("Querying Points")
        for point in progress:
            res = tree.query(point, predicate="within").tolist()
            if len(res) == 0:
                hits.append(False)
            else:
                hits.append(True)

        result = np.array(hits,
                          dtype=np.int8).reshape(height, width)

        # 100 = completely occupied. Only affects occupied cells.
        result *= 100

        # plt.imshow(result)
        plt.show()

        self.header.grid_height = height
        self.header.grid_width = width
        self.header.grid_resolution = cell_size

        return result

    def _build_road_area_tree_(self) -> PreparedGeometry:
        geoms = []
        for road in self.roads:
            for lsec in road.sections:
                for lane in lsec.lanes:
                    if lane.type == LaneType.DRIVING:
                        geoms.append(lane.shape)

        return STRtree(geoms)

    def _parse_header_(self, root: ET.Element) -> Header:
        header = root.find('header')

        north = float(header.attrib['north'])
        south = float(header.attrib['south'])
        east = float(header.attrib['east'])
        west = float(header.attrib['west'])

        # Find our geoReference tag, which has contents like this:
        # "<![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs ]]>"
        geo_reference: str = header.find('geoReference').text
        geo_reference_parts = geo_reference.split('+')

        for part in geo_reference_parts:
            if part.startswith('lon_0'):
                lon0 = float(part[6:])
            elif part.startswith('lat_0'):
                lat0 = float(part[6:])
            elif part.startswith('x_0'):
                x0 = float(part[4:])
            elif part.startswith('y_0'):
                y0 = float(part[4:])

        return Header(north, south, east, west, lat0, lon0, x0, y0)

    def _parse_roads_(self, root: ET.Element) -> list[Road]:
        roads = []

        road_xmls = list()
        progress = tqdm(list(root.iter('road')))
        progress.set_description("Parsing roads")
        for road_xml in progress:
            attrs = road_xml.attrib

            # Begin construction
            road = Road(
                name=attrs['name'],
                id=int(attrs['id']),
                length=float(attrs['length']),
                junction=int(attrs['junction'])
            )

            type_xml = road_xml.find('type')
            if road.junction != -1:
                road.type = RoadType.JUNCTION
                road.speed_limit_ms = 10
            elif type_xml is None:
                road.type = RoadType.NONE
                road.speed_limit_ms = 10
                print(f"Warning: Road type for #{road.id} not found")
            else:
                if type_xml.attrib['type'] == 'town':
                    road.type = RoadType.TOWN
                else:  # TODO: Add other cases
                    road.type = RoadType.UNKNOWN
                speed_xml = type_xml.find('speed')
                if (speed_xml.attrib['unit'] == 'mph'):
                    road.speed_limit_ms = int(
                        int(speed_xml.attrib['max']) * 0.447)
                elif (speed_xml.attrib['unit'] == 'km/h'):
                    road.speed_limit_ms = int(
                        int(speed_xml.attrib['max']) * 0.2778)
                elif (speed_xml.attrib['unit'] == 'm/s'):
                    road.speed_limit_ms = int(int(speed_xml.attrib['max']))
                else:
                    raise RuntimeError('Invalid speed unit')

            road.lane_offset = float(road_xml.find(
                'lanes').find('laneOffset').attrib['a'])

            self._generate_refline_(road_xml, road)
            self._parse_lane_sections_(road_xml, road)
            roads.append(road)

        return roads

    def _generate_refline_(self, road_xml: ET.Element, road: Road):
        points = []

        planview_xml = road_xml.find('planView')
        geom_count = len(list(planview_xml.iter('geometry')))
        for idx, geom_xml in enumerate(planview_xml.iter('geometry')):
            hdg = float(geom_xml.attrib['hdg'])
            length = float(geom_xml.attrib['length'])

            if geom_xml.find('line') is not None:
                # We have a straight line
                x = float(geom_xml.attrib['x'])
                y = float(geom_xml.attrib['y'])

                start_pt = (x, y)
                points.append(start_pt)

                # If this is the last geometry, we need to add an endpoint
                if (idx == geom_count-1):
                    end_pt = (x + length*math.cos(hdg),
                              y + length*math.sin(hdg))
                    points.append(end_pt)
            elif geom_xml.find('arc') is not None:
                arc_xml = geom_xml.find('arc')
                # TODO: We need to implement arc interpolation... Yikes
                x = float(geom_xml.attrib['x'])
                y = float(geom_xml.attrib['y'])
                curvature = float(arc_xml.attrib['curvature'])
                r = 1/curvature
                # Find center point of arc
                # A positive curvature means a left/CCW arc
                # The radius vector is normal to the heading
                center_x = x - r*np.sin(hdg)  # -13.3
                center_y = y + r*np.cos(hdg)  # -116

                circle_pts = []

                for s in np.linspace(0, length, 10):
                    angle = s * curvature - np.pi / 2
                    xs = r * (np.cos(hdg + angle) - np.sin(hdg)) + x
                    ys = r * (np.sin(hdg + angle) + np.cos(hdg)) + y
                    points.append((xs, ys))

                start_pt = (x, y)
                # points.append(start_pt)

        road.refline = LineString(points)
        # plt.plot([point[0] for point in road.refline.coords],
        #          [point[1] for point in road.refline.coords], linewidth=8.0)

    def _rotate_point_(self, origin_x, origin_y, theta, x, y):
        x_ = x-origin_x
        y_ = y-origin_y

        x_new = x_ * np.cos(theta) - y_ * np.sin(theta)
        y_new = x_ * np.sin(theta) + y_ * np.cos(theta)

        return (x_new + origin_x, y_new + origin_y)

    def _parse_lane_sections_(self, road_xml: ET.Element, road: Road):
        sections = []
        for lsec_xml in road_xml.find('lanes').iter('laneSection'):
            attrs = lsec_xml.attrib
            lsec = LaneSection(float(attrs['s']), road=road)

            self._parse_lanes_(lsec_xml, lsec)

            sections.append(lsec)

        road.sections = sections

    def _parse_lanes_(self, lsec_xml: ET.Element, lsec: LaneSection):
        lsec.lanes = []
        width_dict = {}

        whitelist = [146, 9, 10, 133]

        # Iterate through <left>, <center>, and <right>
        for subsec_xml in lsec_xml.iter():
            for lane_xml in subsec_xml.iter('lane'):
                attrs = lane_xml.attrib
                lane = Lane(
                    lsec=lsec,
                    road=lsec.road,
                    id=int(attrs['id'])
                )

                if attrs['type'] == 'driving':
                    lane.type = LaneType.DRIVING
                elif attrs['type'] == 'sidewalk':
                    lane.type = LaneType.SIDEWALK
                elif attrs['type'] == 'shoulder':
                    lane.type = LaneType.SHOULDER
                elif attrs['type'] == 'none':
                    lane.type = LaneType.NONE
                else:
                    lane.type = LaneType.OTHER  # TODO: Add the other types
                lsec.lanes.append(lane)
                width_xml = lane_xml.find('width')
                if lane.id == 0:
                    width_dict[lane.id] = 0.0
                elif width_xml is None:
                    print(f"Road {lane.road.id} has no width")
                    width_dict[lane.id] = 0.0
                else:
                    width_dict[lane.id] = float(
                        lane_xml.find('width').attrib['a'])

        # Now add geometry to each lane
        # Procedure:
        # 1. Start with lane #0 and iterate up

        lane_offset = lane.road.lane_offset

        # Left (positive) lanes
        i = 1
        right_bound: LineString = lsec.road.refline
        if lsec.road.refline is None:
            print(f"Road {lsec.road.id} has no refline!")

        # Keep going left until we exit the lane ("None")
        while lsec.findLane(i) is not None:
            lane = lsec.findLane(i)
            lane_width = width_dict[i]
            lane_offset = lane.road.lane_offset
            left_bound = right_bound.parallel_offset(
                lane_width + lane_offset, 'left')
            lane.left_bound = left_bound
            lane.right_bound = right_bound

            shape_pts = []
            shape_pts += list(lane.left_bound.coords)
            right_bound_coords = list(lane.right_bound.coords)
            right_bound_coords.reverse()
            shape_pts += right_bound_coords
            lane.shape = Polygon(shape_pts)

            plt.fill([point[0] for point in lane.shape.exterior.coords],
                     [point[1] for point in lane.shape.exterior.coords], "paleturquoise")
            plt.plot([point[0] for point in lane.shape.exterior.coords],
                     [point[1] for point in lane.shape.exterior.coords], 'k')

            i += 1
            right_bound = left_bound

        # Right (positive) lanes
        i = -1

        # TODO: Fix this. Does not work with arcs with laneOffset
        if lane_offset > 0.05:
            left_bound = lsec.road.refline.parallel_offset(
                -1*lane_offset, 'right')
        else:
            left_bound: LineString = lsec.road.refline

        # Keep going right until we exit the lane ("None")
        while lsec.findLane(i) is not None:
            lane = lsec.findLane(i)
            lane_width = width_dict[i]

            right_bound = left_bound.parallel_offset(
                lane_width, 'right')
            lane.left_bound = left_bound
            lane.right_bound = right_bound

            shape_pts = []
            shape_pts += list(lane.left_bound.coords)
            right_bound_coords = list(lane.right_bound.coords)
            right_bound_coords.reverse()
            shape_pts += right_bound_coords
            lane.shape = Polygon(shape_pts)

            plt.fill([point[0] for point in lane.shape.exterior.coords],
                     [point[1] for point in lane.shape.exterior.coords], "lightseagreen")

            plt.plot([point[0] for point in lane.shape.exterior.coords],
                     [point[1] for point in lane.shape.exterior.coords], 'k')
            i -= 1
            left_bound = right_bound

    def get_route():
        raise NotImplementedError
