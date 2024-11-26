# distutils: language=c++

cdef extern from "../src/OpenDriveMap.cpp":
    pass
cdef extern from "../src/Geometries/Spiral.cpp":
    pass
cdef extern from "../src/Geometries/Spiral/odrSpiral.cpp":
    pass
cdef extern from "../src/Geometries/ParamPoly3.cpp":
    pass
cdef extern from "../src/Geometries/Arc.cpp":
    pass
cdef extern from "../src/Geometries/Line.cpp":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

from pyOpenDRIVE.XmlNode cimport XmlNode, xml_document
from pyOpenDRIVE.Road cimport Road
from pyOpenDRIVE.Junction cimport Junction
from pyOpenDRIVE.RoadNetworkMesh cimport RoadNetworkMesh
from pyOpenDRIVE.RoutingGraph cimport RoutingGraph

cdef extern from "OpenDriveMap.h" namespace "odr":
    cdef cppclass OpenDriveMap:
        OpenDriveMap(const string& xodr_file, const bool center_map, const bool with_road_objects, const bool with_lateral_profile, const bool with_lane_height, const bool abs_z_for_for_local_road_obj_outline, const bool fix_spiral_edge_cases, const bool with_road_signals) except +

        vector[Road] get_roads() const
        vector[Junction] get_junctions() const

        RoadNetworkMesh get_road_network_mesh(const double eps) const
        RoutingGraph get_routing_graph() const

        string proj4
        double x_offs
        double y_offs
        const string xodr_file
        xml_document xml_doc

        map[string, Road] id_to_road
        map[string, Junction] id_to_junction

cdef class PyOpenDriveMap:
    cdef inline OpenDriveMap* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[OpenDriveMap] c_self