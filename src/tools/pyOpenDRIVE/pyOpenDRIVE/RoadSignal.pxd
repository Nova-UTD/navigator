# distutils: language=c++

cdef extern from "../src/RoadSignal.cpp" namespace "odr":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

from pyOpenDRIVE.XmlNode cimport XmlNode
from pyOpenDRIVE.Mesh cimport Mesh3D
from pyOpenDRIVE.LaneValidityRecord cimport LaneValidityRecord

cdef extern from "RoadSignal.h" namespace "odr":
    cdef cppclass RoadSignal(XmlNode):
        RoadSignal(string road_id, string id, string name, double s0, double t0, bool is_dynamic, double zOffset, double value, double height, double width, double hOffset, double pitch, double roll, string orientation, string country, string type, string subtype, string unit, string text) except +

        @staticmethod
        Mesh3D get_box(const double width, const double length, const double height)

        string road_id
        string id
        string name
        double s0
        double t0
        bool is_dynamic
        double zOffset
        double value
        double height
        double width
        double hOffset
        double pitch
        double roll
        string orientation
        string country
        string type
        string subtype
        string unit
        string text

        vector[LaneValidityRecord] lane_validities

cdef class PyRoadSignal:
    @staticmethod
    cdef inline PyRoadSignal wrap(const RoadSignal& c_obj):
        temp = PyRoadSignal()
        temp.c_self = make_shared[RoadSignal](c_obj)
        return temp

    cdef inline RoadSignal* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[RoadSignal] c_self