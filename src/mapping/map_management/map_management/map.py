from xml.dom import minidom
from shapely.geometry import Point

class Map():

    def __init__(self, map_string: str):
        self.dom_tree = minidom.parseString(map_string)
        
        # roads = dom_tree.documentElement.getElementsByTagName("road")
        self._parse_header_()

    def _parse_header_(self):
        header: minidom.Element = self.dom_tree.documentElement.getElementsByTagName("header")[0]

         # Find our geoReference tag, which has contents like this:
        # "<![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs ]]>"
        geo_reference: str = header.getElementsByTagName('geoReference')[0].childNodes[0].data
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
                