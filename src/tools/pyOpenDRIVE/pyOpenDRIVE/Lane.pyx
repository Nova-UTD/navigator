# distutils: language=c++

from cython.operator cimport dereference, postincrement

from pyOpenDRIVE cimport Lane

from pyOpenDRIVE.CubicSpline cimport PyCubicSpline
from pyOpenDRIVE.CubicSpline import PyCubicSpline

from pyOpenDRIVE.RoadMark cimport PyRoadMark, PyRoadMarkGroup, PyRoadMarksLine
from pyOpenDRIVE.RoadMark import PyRoadMark, PyRoadMarkGroup, PyRoadMarksLine

cdef class PyHeightOffset:
    def __cinit__(self, double inner = 0, double outer = 0):
        self.c_self = make_shared[HeightOffset](inner, outer)

    @property
    def inner(self):
        return self.unwrap().inner

    @property
    def outer(self):
        return self.unwrap().outer

cdef class PyLaneKey:
    def __cinit__(self, string road_id = "", double lanesection_s0 = 0, int lane_id = 0):
        if road_id != "":
            self.c_self = make_shared[LaneKey](road_id, lanesection_s0, lane_id)
    
    def to_string(self):
        return self.unwrap().to_string()

    @property
    def road_id(self):
        return self.unwrap().road_id

    @property
    def lanesection_s0(self):
        return self.unwrap().lanesection_s0

    @property
    def lane_id(self):
        return self.unwrap().lane_id

cdef class PyLane:
    def __cinit__(self, string road_id = "", double lanesection_s0 = 0, int id = 0, bool level = False, string type = ""):
        if road_id != "":
            self.c_self = make_shared[Lane](road_id, lanesection_s0, id, level, type)

    def get_roadmarks(self, const double s_start, const double s_end):
        out_arr = []
        cdef vector[RoadMark] c_objs = self.unwrap().get_roadmarks(s_start, s_end)
        for i in range(c_objs.size()):
            out_val = PyRoadMark.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    @property
    def key(self):
        return PyLaneKey.wrap(self.unwrap().key)
    
    @property
    def id(self):
        return self.unwrap().id

    @property
    def level(self):
        return self.unwrap().level

    @property
    def predecessor(self):
        return self.unwrap().predecessor

    @property
    def successor(self):
        return self.unwrap().successor

    @property
    def type(self):
        return self.unwrap().type

    @property
    def lane_width(self):
        return PyCubicSpline.wrap(self.unwrap().lane_width)

    @property
    def outer_border(self):
        return PyCubicSpline.wrap(self.unwrap().outer_border)

    @property
    def inner_border(self):
        return PyCubicSpline.wrap(self.unwrap().inner_border)

    @property
    def s_to_height_offset(self):
        out_dict = {}
        cdef map[double, HeightOffset] c_map = self.unwrap().s_to_height_offset
        cdef map[double, HeightOffset].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyHeightOffset.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict

    @property
    def roadmark_groups(self):
        out_set = set()
        cdef libcpp_set[RoadMarkGroup] c_set = self.unwrap().roadmark_groups
        cdef libcpp_set[RoadMarkGroup].iterator iter = c_set.begin()
        while iter != c_set.end():
            out_set.add(PyRoadMarkGroup.wrap(dereference(iter)))
            postincrement(iter)
        return out_set