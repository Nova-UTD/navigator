from __future__ import annotations

import math
from shapely.geometry import LineString, Polygon
import xml.etree.ElementTree as ET

# Strictly for testing/debug
import matplotlib.pyplot as plt

from .enums import RoadType
from .header import Header
from .lane import Lane
from .lane_section import LaneSection
from .road import Road


class Map:
    def __init__(self, map_str: str):
        self._root_ = ET.fromstring(map_str)

        self.header = self._parse_header_(self._root_)
        self.roads = self._parse_roads_(self._root_)

        self.shapes = []
        self.roads = []
        self.controllers = []
        self.junctions = []

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
        for road_xml in root.iter('road'):
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

            self._generate_refline_(road_xml, road)
            self._parse_lane_sections_(road_xml, road)

        plt.show()

    def _generate_refline_(self, road_xml: ET.Element, road: Road):
        points = []

        planview_xml = road_xml.find('planView')
        geom_count = len(list(planview_xml.iter('geometry')))
        for idx, geom_xml in enumerate(planview_xml.iter('geometry')):
            if geom_xml.find('line') is not None:
                # We have a straight line
                x = float(geom_xml.attrib['x'])
                y = float(geom_xml.attrib['y'])

                start_pt = (x, y)
                points.append(start_pt)

                # If this is the last geometry, we need to add an endpoint
                if (idx == geom_count-1):
                    hdg = float(geom_xml.attrib['hdg'])
                    length = float(geom_xml.attrib['length'])
                    end_pt = (x + length*math.cos(hdg),
                              y + length*math.sin(hdg))
                    points.append(end_pt)
            elif geom_xml.find('arc') is not None:
                # TODO: We need to implement arc interpolation... Yikes
                x = float(geom_xml.attrib['x'])
                y = float(geom_xml.attrib['y'])

                start_pt = (x, y)
                points.append(start_pt)

        road.refline = LineString(points)
        # plt.plot([point[0] for point in road.refline.coords],
        #          [point[1] for point in road.refline.coords], linewidth=8.0)

    def _parse_lane_sections_(self, road_xml: ET.Element, road: Road):
        for lsec_xml in road_xml.find('lanes').iter('laneSection'):
            attrs = lsec_xml.attrib
            lsec = LaneSection(float(attrs['s']), road=road)
            self._parse_lanes_(lsec_xml, lsec)

    def _parse_lanes_(self, lsec_xml: ET.Element, lsec: LaneSection):
        lsec.lanes = []
        width_dict = {}

        # Iterate through <left>, <center>, and <right>
        for subsec_xml in lsec_xml.iter():
            for lane_xml in subsec_xml.iter('lane'):
                attrs = lane_xml.attrib
                lane = Lane(
                    lsec=lsec,
                    road=lsec.road,
                    id=int(attrs['id']),
                    type=attrs['type']
                )
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

        # Left (positive) lanes
        i = 1
        right_bound: LineString = lsec.road.refline
        if lsec.road.refline is None:
            print(f"Road {lsec.road.id} has no refline!")

        # Keep going left until we exit the lane ("None")
        while lsec.findLane(i) is not None:
            lane = lsec.findLane(i)
            lane_width = width_dict[i]
            left_bound = right_bound.parallel_offset(lane_width, 'left')
            lane.left_bound = left_bound
            lane.right_bound = right_bound

            shape_pts = []
            shape_pts += list(lane.left_bound.coords)
            right_bound_coords = list(lane.right_bound.coords)
            right_bound_coords.reverse()
            shape_pts += right_bound_coords
            lane.shape = Polygon(shape_pts)

            whitelist = [1, 12, 16]
            if 1:
                plt.fill([point[0] for point in lane.shape.exterior.coords],
                         [point[1] for point in lane.shape.exterior.coords], "r")

            i += 1
            right_bound = left_bound

        # Right (positive) lanes
        i = -1
        left_bound: LineString = lsec.road.refline

        # Keep going right until we exit the lane ("None")
        while lsec.findLane(i) is not None:
            lane = lsec.findLane(i)
            lane_width = width_dict[i]

            right_bound = left_bound.parallel_offset(lane_width, 'right')
            lane.left_bound = left_bound
            lane.right_bound = right_bound

            shape_pts = []
            shape_pts += list(lane.left_bound.coords)
            right_bound_coords = list(lane.right_bound.coords)
            right_bound_coords.reverse()
            shape_pts += right_bound_coords
            lane.shape = Polygon(shape_pts)
            whitelist = [1, 12, 16]
            if 1:
                plt.fill([point[0] for point in lane.shape.exterior.coords],
                         [point[1] for point in lane.shape.exterior.coords], "b")
                print(f"{lane.road.id}: {lane_width}")
            i -= 1
            left_bound = right_bound

    def get_route():
        raise NotImplementedError
