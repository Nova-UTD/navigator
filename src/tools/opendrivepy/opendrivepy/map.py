from __future__ import annotations

import xml.etree.ElementTree as ET

from .enums import RoadType
from .header import Header
from .road import Road


class Map:
    def __init__(self, map_str: str):
        self._root_ = ET.fromstring(map_str)

        for road in self._root_.iter('road'):
            print(road.attrib)

        self.header = self._parse_header_(self._root_)
        self.roads = self._parse_roads_(self._root_)

        self.shapes = []
        self.roads = []
        self.controllers = []
        self.junctions = []

    def _parse_header_(self, root: ET.Element) -> Header:
        header = root.find('header')
        print(header.attrib)

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

            print(road)

            # Add speed limit

    def getRoute():
        raise NotImplementedError
