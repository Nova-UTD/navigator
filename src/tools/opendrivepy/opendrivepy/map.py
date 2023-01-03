import xml.etree.ElementTree as ET
from .header import Header


class Map:
    def __init__(self, map_str: str):
        self._root_ = ET.fromstring(map_str)

        for road in self._root_.iter('road'):
            print(road.attrib)

        self.header = self._parse_header_(self._root_)

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

    def _get_roads(self, root: ET.Element) ->

    def getRoute():
        raise NotImplementedError
