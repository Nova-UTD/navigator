# distutils: language=c++

from cython.operator cimport dereference, postincrement

from pyOpenDRIVE cimport LaneSection

from pyOpenDRIVE.Lane cimport PyLane
from pyOpenDRIVE.Lane import PyLane

cdef class PyLaneSection:
    def __cinit__(self, string road_id = "", double s0 = 0):
        if road_id != "":
            self.c_self = make_shared[LaneSection](road_id, s0)
    
    def get_lanes(self):
        out_arr = []
        cdef vector[Lane] c_objs = self.unwrap().get_lanes()
        for i in range(c_objs.size()):
            out_val = PyLane.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    def get_lane_id(self, const double s, const double t):
        return self.unwrap().get_lane_id(s, t)

    def get_lane(self, const double s, const double t):
        return PyLane.wrap(self.unwrap().get_lane(s, t))

    @property
    def road_id(self):
        return self.unwrap().road_id

    @property
    def s0(self):
        return self.unwrap().s0

    @property 
    def id_to_lane(self):
        out_dict = {}
        cdef map[int, Lane] c_map = self.unwrap().id_to_lane
        cdef map[int, Lane].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyLane.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict