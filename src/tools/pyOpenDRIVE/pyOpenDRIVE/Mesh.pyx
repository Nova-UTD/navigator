# distutils: language=c++

from pyOpenDRIVE.Mesh cimport Mesh3D

from pyOpenDRIVE.Math cimport PyVec3D, PyVec2D
from pyOpenDRIVE.Math import PyVec3D, PyVec2D

cdef class PyMesh3D:
    def __cinit__(self):
        self.c_self = make_shared[Mesh3D]()

    def add_mesh(self, PyMesh3D other):
        self.unwrap().add_mesh(other.unwrap()[0])

    def get_obj(self):
        return self.unwrap().get_obj()

    @property
    def vertices(self):
        out_arr = []
        cdef vector[Vec3D] c_objs = self.unwrap().vertices
        for i in range(c_objs.size()):
            out_val = PyVec3D.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    @property
    def indices(self):
        out_arr = []
        cdef vector[uint32_t] c_objs = self.unwrap().indices
        for i in range(c_objs.size()):
            out_val = <int>c_objs[i]
            out_arr.append(out_val)
        return out_arr

    @property
    def normals(self):
        out_arr = []
        cdef vector[Vec3D] c_objs = self.unwrap().normals
        for i in range(c_objs.size()):
            out_val = PyVec3D.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    @property
    def st_coordinates(self):
        out_arr = []
        cdef vector[Vec2D] c_objs = self.unwrap().st_coordinates
        for i in range(c_objs.size()):
            out_val = PyVec2D.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr