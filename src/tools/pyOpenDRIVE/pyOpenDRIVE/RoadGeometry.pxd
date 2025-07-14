# distutils: language=c++

cdef extern from "../src/Geometries/RoadGeometry.cpp":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr, unique_ptr

from pyOpenDRIVE.XmlNode cimport XmlNode
from pyOpenDRIVE.Math cimport Vec2D

cdef extern from "Geometries/RoadGeometry.h" namespace "odr":
    cdef enum GeometryType:
        GeometryType_Line,
        GeometryType_Spiral,
        GeometryType_Arc,
        GeometryType_ParamPoly3

    # NOTE: This class is abstract in the original src; do NOT try to instantiate it directly!
    cdef cppclass RoadGeometry(XmlNode):
        RoadGeometry(double s0, double x0, double y0, double hdg0, double length, GeometryType type) except +

        unique_ptr[RoadGeometry] clone() const

        Vec2D get_xy(double s) const
        Vec2D get_grad(double s) const

        set[double] approximate_linear(double eps) const

        double s0
        double x0
        double y0
        double hdg0
        double length
        GeometryType type