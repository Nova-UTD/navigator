# distutils: language=c++

cdef extern from "../src/Lane.cpp":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set as libcpp_set # Alias to prevent name collision with Python's set()
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

from pyOpenDRIVE.CubicSpline cimport CubicSpline
from pyOpenDRIVE.RoadMark cimport RoadMark, RoadMarkGroup

cdef extern from "Lane.h" namespace "odr":
    cdef cppclass HeightOffset:
        HeightOffset(double inner, double outer) except +

        double inner
        double outer

    cdef cppclass LaneKey:
        LaneKey(string road_id, double lanesection_s0, int lane_id) except +
        string to_string() const

        string road_id
        double lanesection_s0
        int lane_id

    cdef cppclass Lane:
        Lane(string road_id, double lanesection_s0, int id, bool level, string type) except +

        vector[RoadMark] get_roadmarks(const double s_start, const double s_end) const

        LaneKey key
        int id
        bool level
        int predecessor
        int successor
        string type

        CubicSpline lane_width
        CubicSpline outer_border
        CubicSpline inner_border

        map[double, HeightOffset] s_to_height_offset
        libcpp_set[RoadMarkGroup] roadmark_groups

cdef class PyLane:
    @staticmethod
    cdef inline PyLane wrap(const Lane& c_obj):
        temp = PyLane()
        temp.c_self = make_shared[Lane](c_obj)
        return temp

    cdef inline Lane* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[Lane] c_self

cdef class PyHeightOffset:
    @staticmethod
    cdef inline PyHeightOffset wrap(const HeightOffset& c_obj):
        temp = PyHeightOffset()
        temp.c_self = make_shared[HeightOffset](c_obj)
        return temp

    cdef inline HeightOffset* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[HeightOffset] c_self

cdef class PyLaneKey:
    @staticmethod
    cdef inline PyLaneKey wrap(const LaneKey& c_obj):
        temp = PyLaneKey()
        temp.c_self = make_shared[LaneKey](c_obj)
        return temp

    cdef inline LaneKey* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[LaneKey] c_self