# distutils: language=c++

cdef extern from "../src/Road.cpp":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

cdef extern from "<cstdint>" namespace "std":
    cdef cppclass uint32_t

from pyOpenDRIVE.Lane cimport Lane
from pyOpenDRIVE.LaneSection cimport LaneSection
from pyOpenDRIVE.CubicSpline cimport CubicSpline
from pyOpenDRIVE.Array cimport array
from pyOpenDRIVE.Mesh cimport Mesh3D
from pyOpenDRIVE.RoadMark cimport RoadMark
from pyOpenDRIVE.Math cimport Vec3D, Line3D
from pyOpenDRIVE.RoadObject cimport RoadObject
from pyOpenDRIVE.RoadSignal cimport RoadSignal
from pyOpenDRIVE.RefLine cimport RefLine

cdef extern from "Road.h" namespace "odr":
    cdef cppclass Crossfall:
        enum Side:
            Side_Both,
            Side_Left,
            Side_Right

        double get_crossfall(const double s, const bool on_left_side) const
        map[double, Side] sides

    cdef cppclass RoadLink:
        enum ContactPoint:
            ContactPoint_None,
            ContactPoint_Start,
            ContactPoint_End

        enum Type:
            Type_None,
            Type_Road,
            Type_Junction

        RoadLink()
        RoadLink(string id, Type type, ContactPoint contact_point)

        string id
        Type type
        ContactPoint contact_point
    
    cdef cppclass RoadNeighbor:
        RoadNeighbor(string id, string side, string direction)

        string id
        string side
        string direction

    cdef cppclass SpeedRecord:
        SpeedRecord(string max, string unit)

        string max
        string unit

    cdef cppclass Road:
        Road(string id, double length, string junction, string name, bool left_hand_traffic) except +

        vector[LaneSection] get_lanesections() const
        vector[RoadObject]  get_road_objects() const
        vector[RoadSignal]  get_road_signals() const

        double      get_lanesection_s0(const double s) const
        LaneSection get_lanesection(const double s) const

        double get_lanesection_end(const LaneSection& lanesection) const
        double get_lanesection_end(const double lanesection_s0) const
        double get_lanesection_length(const LaneSection& lanesection) const
        double get_lanesection_length(const double lanesection_s0) const

        Vec3D get_xyz(const double s, const double t, const double h, Vec3D* e_s, Vec3D* e_t, Vec3D* e_h) const
        Vec3D get_surface_pt(double s, const double t, Vec3D* vn) const

        Line3D get_lane_border_line(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer) const
        Line3D get_lane_border_line(const Lane& lane, const double eps, const bool outer) const

        Mesh3D get_lane_mesh(const Lane& lane, const double s_start, const double s_end, const double eps, vector[uint32_t]* outline_indices) const
        Mesh3D get_lane_mesh(const Lane& lane, const double eps, vector[uint32_t]* outline_indices) const

        Mesh3D get_roadmark_mesh(const Lane& lane, const RoadMark& roadmark, const double eps) const
        Mesh3D get_road_signal_mesh(const RoadSignal& road_signal) const
        Mesh3D get_road_object_mesh(const RoadObject& road_object, const double eps) const

        set[double] approximate_lane_border_linear(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer) const
        set[double] approximate_lane_border_linear(const Lane& lane, const double eps, const bool outer) const

        double length
        string id
        string junction
        string name
        bool left_hand_traffic

        RoadLink predecessor
        RoadLink successor
        vector[RoadNeighbor] neighbors

        CubicSpline lane_offset
        CubicSpline superelevation
        Crossfall   crossfall
        RefLine     ref_line

        map[double, LaneSection]     s_to_lanesection
        map[double, string]     s_to_type
        map[double, SpeedRecord]     s_to_speed
        map[string, RoadObject] id_to_object
        map[string, RoadSignal] id_to_signal

cdef class PyCrossfall:
    @staticmethod
    cdef inline PyCrossfall wrap(const Crossfall& c_obj):
        temp = PyCrossfall()
        temp.c_self = make_shared[Crossfall](c_obj)
        return temp

    cdef inline Crossfall* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[Crossfall] c_self

cdef class PyRoadLink:
    @staticmethod
    cdef inline PyRoadLink wrap(const RoadLink& c_obj):
        temp = PyRoadLink()
        temp.c_self = make_shared[RoadLink](c_obj)
        return temp

    cdef inline RoadLink* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[RoadLink] c_self

cdef class PyRoadNeighbor:
    @staticmethod
    cdef inline PyRoadNeighbor wrap(const RoadNeighbor& c_obj):
        temp = PyRoadNeighbor()
        temp.c_self = make_shared[RoadNeighbor](c_obj)
        return temp

    cdef inline RoadNeighbor* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[RoadNeighbor] c_self

cdef class PySpeedRecord:
    @staticmethod
    cdef inline PySpeedRecord wrap(const SpeedRecord& c_obj):
        temp = PySpeedRecord()
        temp.c_self = make_shared[SpeedRecord](c_obj)
        return temp

    cdef inline SpeedRecord* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[SpeedRecord] c_self

cdef class PyRoad:
    @staticmethod
    cdef inline PyRoad wrap(const Road& c_obj):
        temp = PyRoad()
        temp.c_self = make_shared[Road](c_obj)
        return temp

    cdef inline Road* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[Road] c_self