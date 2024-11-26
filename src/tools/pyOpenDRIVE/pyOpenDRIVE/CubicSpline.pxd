# distutils: language=c++

cdef extern from "../src/Geometries/CubicSpline.cpp":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

cdef extern from "Geometries/CubicSpline.h" namespace "odr":
    cdef cppclass Poly3:
        Poly3() except +
        Poly3(double s0, double a, double b, double c, double d) except +

        double get(double s) const
        double get_grad(double s) const
        double get_max(double s_start, double s_end) const
        void   negate()
        bool   isnan() const

        set[double] approximate_linear(double eps, double s_start, double s_end) const

        double a
        double b
        double c
        double d

    cdef cppclass CubicSpline:
        CubicSpline() except +

        double get(double s, double default_val, bool extend_start)
        double get_grad(double s, double default_val, bool extend_start) const
        double get_max(double s_start, double s_end) const
        Poly3  get_poly(double s, bool extend_start) const

        bool        empty() const
        size_t size() const
        CubicSpline negate() const
        CubicSpline add(const CubicSpline& other) const

        set[double] approximate_linear(double eps, double s_start, double s_end) const

        map[double, Poly3] s0_to_poly

cdef class PyCubicSpline:
    @staticmethod
    cdef inline PyCubicSpline wrap(const CubicSpline& c_obj):
        temp = PyCubicSpline()
        temp.c_self = make_shared[CubicSpline](c_obj)
        return temp

    cdef inline CubicSpline* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[CubicSpline] c_self