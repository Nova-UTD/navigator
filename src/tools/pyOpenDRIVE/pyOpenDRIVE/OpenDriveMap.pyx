# distutils: language=c++

from cython.operator cimport dereference, postincrement

from pyOpenDRIVE.OpenDriveMap cimport OpenDriveMap

from pyOpenDRIVE.Road import PyRoad
from pyOpenDRIVE.Road cimport PyRoad

from pyOpenDRIVE.Junction import PyJunction
from pyOpenDRIVE.Junction cimport PyJunction

from pyOpenDRIVE.RoadNetworkMesh import PyRoadNetworkMesh
from pyOpenDRIVE.RoadNetworkMesh cimport PyRoadNetworkMesh

from pyOpenDRIVE.RoutingGraph import PyRoutingGraph
from pyOpenDRIVE.RoutingGraph cimport PyRoutingGraph

from pyOpenDRIVE.XmlNode import Py_xml_document
from pyOpenDRIVE.XmlNode cimport Py_xml_document

cdef class PyOpenDriveMap:
    def __cinit__(self, const string& xodr_file, bool center_map = False, bool with_road_objects = True, bool with_lateral_profile = True, bool with_lane_height = True, bool abs_z_for_for_local_road_obj_outline = False, bool fix_spiral_edge_cases = True, bool with_road_signals = True):
        self.c_self = make_shared[OpenDriveMap](xodr_file, center_map, with_road_objects, with_lateral_profile, with_lane_height, abs_z_for_for_local_road_obj_outline, fix_spiral_edge_cases, with_road_signals)

    def get_roads(self):
        out_arr = []
        cdef vector[Road] c_objs = self.unwrap().get_roads()
        for i in range(c_objs.size()):
            out_val = PyRoad.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    def get_junctions(self):
        out_arr = []
        cdef vector[Junction] c_objs = self.unwrap().get_junctions()
        for i in range(c_objs.size()):
            out_val = PyJunction.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    def get_road_network_mesh(self, const double eps):
        return PyRoadNetworkMesh.wrap(self.unwrap().get_road_network_mesh(eps))

    def get_routing_graph(self):
        return PyRoutingGraph.wrap(self.unwrap().get_routing_graph())

    @property
    def proj4(self):
        return self.unwrap().proj4

    @property
    def x_offs(self):
        return self.unwrap().x_offs

    @property
    def y_offs(self):
        return self.unwrap().y_offs

    @property
    def xodr_file(self):
        return self.unwrap().xodr_file

    @property
    def xml_doc(self):
        return Py_xml_document.wrap(self.unwrap().xml_doc)
    
    @property
    def id_to_road(self):
        out_dict = {}
        cdef map[string, Road] c_map = self.unwrap().id_to_road
        cdef map[string, Road].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyRoad.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict

    @property
    def id_to_junction(self):
        out_dict = {}
        cdef map[string, Junction] c_map = self.unwrap().id_to_junction
        cdef map[string, Junction].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyJunction.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict