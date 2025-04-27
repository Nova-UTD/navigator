# distutils: language=c++

from cython.operator cimport dereference, postincrement

from pyOpenDRIVE cimport RefLine

from pyOpenDRIVE.Math import PyVec3D, PyLine3D
from pyOpenDRIVE.Math cimport PyVec3D, PyLine3D

from pyOpenDRIVE.CubicSpline import PyCubicSpline
from pyOpenDRIVE.CubicSpline cimport PyCubicSpline

#from pyOpenDRIVE.RoadGeometry import PyRoadGeometry
#from pyOpenDRIVE.RoadGeometry cimport PyRoadGeometry

cdef class PyRefLine:
    def __cinit__(self, string road_id = '', double length = 0):
        if road_id != '':
            self.c_self = make_shared[RefLine](road_id, length)
    
    def __cinit__(self, PyRefLine other = None):
        if other != None:
            self.c_self = make_shared[RefLine](other.c_self.get()[0])

    def get_geometry_s0(self, double s):
        return self.unwrap().get_geometry_s0(s)

    def get_xyz(self, double s):
        return PyVec3D.wrap(self.unwrap().get_xyz(s))

    def get_grad(self, double s):
        return PyVec3D.wrap(self.unwrap().get_grad(s))

    def get_line(self, double s_start, double s_end, double eps):
        return PyLine3D.wrap(self.unwrap().get_line(s_start, s_end, eps))

    def match(self, double x, double y):
        return self.unwrap().match(x, y)

    def approximate_linear(self, double eps, double s_start, double s_end):
        out_set = set()
        cdef libcpp_set[double] c_set = self.unwrap().approximate_linear(eps, s_start, s_end)
        cdef libcpp_set[double].iterator iter = c_set.begin()
        while iter != c_set.end():
            out_set.add(dereference(iter))
            postincrement(iter)
        return out_set

    @property
    def road_id(self):
        return self.unwrap().road_id

    @property
    def length(self):
        return self.unwrap().length

    @property
    def elevation_profile(self):
        return PyCubicSpline.wrap(self.unwrap().elevation_profile)

    #@property
    #def s0_to_geometry(self):
    #    out_dict = {}
    #    cdef map[string, unique_ptr[RoadGeometry]] c_map = self.unwrap().s0_to_geometry
    #    cdef map[string, unique_ptr[RoadGeometry]].iterator iter = c_map.begin()
    #    while iter != c_map.end():
    #        out_dict[dereference(iter).first] = PyRoadGeometry.wrap(dereference(dereference(iter).second))
    #        postincrement(iter)
    #    return out_dict
