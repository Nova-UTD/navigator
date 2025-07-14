# distutils: language=c++

cdef extern from "../src/RoadMark.cpp":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set as libcpp_set # Alias to prevent name collision with Python's set()
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

from pyOpenDRIVE.XmlNode cimport XmlNode, PyXmlNode

cdef extern from "RoadMark.h" namespace "odr":
    cdef cppclass RoadMarksLine(XmlNode):
        RoadMarksLine(string road_id, double lanesection_s0, int lane_id, double group_s0, double width, double length, double space, double t_offset, double s_offset, string name, string rule) except +

        string road_id
        double lanesection_s0
        int lane_id
        double group_s0

        double width
        double length
        double space
        double t_offset
        double s_offset

        string name
        string rule
        
    cdef cppclass RoadMarkGroup(XmlNode):
        RoadMarkGroup(string road_id, double lanesection_s0, int lane_id, double width, double height, double s_offset, string type, string weight, string color, string material, string lane_change) except +

        string road_id
        double lanesection_s0
        int lane_id

        double width
        double height
        double s_offset

        string type
        string weight
        string color
        string material
        string lane_change

        libcpp_set[RoadMarksLine] roadmark_lines

    cdef cppclass RoadMark:
        RoadMark(string road_id, double lanesection_s0, int lane_id, double group_s0, double s_start, double s_end, double t_offset, double width, string type) except +

        string road_id
        double lanesection_s0
        int lane_id
        double group_s0

        double s_start
        double s_end
        double t_offset
        double width

        string type

cdef class PyRoadMark:
    @staticmethod
    cdef inline PyRoadMark wrap(const RoadMark& c_obj):
        temp = PyRoadMark()
        temp.c_self = make_shared[RoadMark](c_obj)
        return temp

    cdef inline RoadMark* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[RoadMark] c_self

cdef class PyRoadMarkGroup(PyXmlNode):
    @staticmethod
    cdef inline PyRoadMarkGroup wrap(const RoadMarkGroup& c_obj):
        temp = PyRoadMarkGroup()
        temp.c_self = make_shared[RoadMarkGroup](c_obj)
        return temp

    cdef inline RoadMarkGroup* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[RoadMarkGroup] c_self

cdef class PyRoadMarksLine(PyXmlNode):
    @staticmethod
    cdef inline PyRoadMarksLine wrap(const RoadMarksLine& c_obj):
        temp = PyRoadMarksLine()
        temp.c_self = make_shared[RoadMarksLine](c_obj)
        return temp

    cdef inline RoadMarksLine* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[RoadMarksLine] c_self