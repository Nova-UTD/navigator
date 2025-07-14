# distutils: language=c++

from cython.operator cimport dereference

from pyOpenDRIVE.Math cimport dim_2, dim_3, Vec2D, Vec3D, Line3D
from pyOpenDRIVE.Array cimport array

cdef class PyVec2D:
    def __cinit__(self):
        self.c_self = make_shared[Vec2D]()

    @property
    def array(self):
        out_arr = []
        cdef int *internal_array = <int*>self.unwrap().data()
        for i in range(self.unwrap().size()):
            out_val = internal_array[i]
            out_arr.append(out_val)
        return out_arr

cdef class PyVec3D:
    def __cinit__(self):
        self.c_self = make_shared[Vec3D]()

    @property
    def array(self):
        out_arr = []
        cdef int *internal_array = <int*>self.unwrap().data()
        for i in range(self.unwrap().size()):
            out_val = internal_array[i]
            out_arr.append(out_val)
        return out_arr

cdef class PyLine3D:
    def __cinit__(self):
        self.c_self = make_shared[Line3D]()

    @property
    def array(self):
        out_arr = []
        cdef vector[Vec3D] *c_objs = <vector[Vec3D]*>self.unwrap()
        for i in range(c_objs.size()):
            out_val = PyVec3D.wrap(dereference(c_objs)[i])
            out_arr.append(out_val)
        return out_arr