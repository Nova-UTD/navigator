# distutils: language=c++

cdef extern from "../src/RoadObject.cpp" namespace "odr":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

from pyOpenDRIVE.Math cimport Vec3D
from pyOpenDRIVE.XmlNode cimport XmlNode
from pyOpenDRIVE.Mesh cimport Mesh3D
from pyOpenDRIVE.LaneValidityRecord cimport LaneValidityRecord

cdef extern from "RoadObject.h" namespace "odr":
    cdef cppclass RoadObjectRepeat(XmlNode):
        RoadObjectRepeat(double s0, double length, double distance, double t_start, double t_end, double width_start, double width_end, double height_start, double height_end, double z_offset_start, double z_offset_end) except +

        double s0
        double length
        double distance
        double t_start
        double t_end
        double width_start
        double width_end
        double height_start
        double height_end
        double z_offset_start
        double z_offset_end

    cdef cppclass RoadObjectCorner(XmlNode):
        enum Type:
            Type_Local_RelZ,
            Type_Local_AbsZ,
            Type_Road

        RoadObjectCorner(int id, Vec3D pt, double height, Type type) except +

        int id
        Vec3D pt
        double height
        Type type

    cdef cppclass RoadObjectOutline(XmlNode):
        RoadObjectOutline(int id, string fill_type, string lane_type, bool outer, bool closed) except +

        int id
        string fill_type
        string lane_type
        bool outer
        bool closed

        vector[RoadObjectCorner] outline
    
    cdef cppclass RoadObject(XmlNode):
        RoadObject(string road_id, string id, double s0, double t0, double z0, double length, double valid_length, double width, double radius, double height, double hdg, double pitch, double roll, string type, string name, string orientation, string subtype, bool is_dynamic) except +

        @staticmethod
        Mesh3D get_cylinder(const double eps, const double radius, const double height)
        @staticmethod
        Mesh3D get_box(const double width, const double length, const double height)

        string road_id

        string id
        string type
        string name
        string orientation
        string subtype

        double s0
        double t0
        double z0
        double length
        double valid_length
        double width
        double radius
        double height
        double hdg
        double pitch
        double roll
        bool is_dynamic

        vector[RoadObjectRepeat] repeats
        vector[RoadObjectOutline] outlines
        vector[LaneValidityRecord] lane_validities

cdef class PyRoadObject:
    @staticmethod
    cdef inline PyRoadObject wrap(const RoadObject& c_obj):
        temp = PyRoadObject()
        temp.c_self = make_shared[RoadObject](c_obj)
        return temp

    cdef inline RoadObject* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[RoadObject] c_self