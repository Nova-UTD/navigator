# distutils: language=c++

from libcpp.memory cimport make_shared, shared_ptr
from libcpp.vector cimport vector
from pyOpenDRIVE.Array cimport array

cdef extern from "Math.hpp" namespace "odr":
    ctypedef int dim_3 "3"
    ctypedef int dim_2 "2"

    ctypedef array[double, dim_2] Vec2D
    ctypedef array[double, dim_3] Vec3D
    ctypedef vector[Vec3D] Line3D

cdef class PyVec2D:
    @staticmethod
    cdef inline PyVec2D wrap(const Vec2D& c_obj):
        temp = PyVec2D()
        temp.c_self = make_shared[Vec2D](c_obj)
        return temp

    cdef inline Vec2D* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[Vec2D] c_self

cdef class PyVec3D:
    @staticmethod
    cdef inline PyVec3D wrap(const Vec3D& c_obj):
        temp = PyVec3D()
        temp.c_self = make_shared[Vec3D](c_obj)
        return temp

    cdef inline Vec3D* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[Vec3D] c_self

cdef class PyLine3D:
    @staticmethod
    cdef inline PyLine3D wrap(const Line3D& c_obj):
        temp = PyLine3D()
        temp.c_self = make_shared[Line3D](c_obj)
        return temp

    cdef inline Line3D* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[Line3D] c_self