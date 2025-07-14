# distutils: language=c++

from cython.operator cimport dereference, postincrement

from pyOpenDRIVE cimport RoadMark

cdef class PyRoadMarksLine(PyXmlNode):
    def __cinit__(self, string road_id = "", double lanesection_s0 = 0, int lane_id = 0, double group_s0 = 0, double width = 0, double length = 0, double space = 0, double t_offset = 0, double s_offset = 0, string name = "", string rule = ""):
        if road_id != "":
            self.c_self = make_shared[RoadMarksLine](road_id, lanesection_s0, lane_id, group_s0, width, length, space, t_offset, s_offset, name, rule)

    @property
    def road_id(self):
        return self.unwrap().road_id

    @property
    def lanesection_s0(self):
        return self.unwrap().lanesection_s0

    @property
    def lane_id(self):
        return self.unwrap().lane_id

    @property
    def group_s0(self):
        return self.unwrap().group_s0

    @property
    def width(self):
        return self.unwrap().width

    @property
    def length(self):
        return self.unwrap().length

    @property
    def space(self):
        return self.unwrap().space

    @property
    def t_offset(self):
        return self.unwrap().t_offset

    @property
    def s_offset(self):
        return self.unwrap().s_offset

    @property
    def name(self):
        return self.unwrap().name

    @property
    def rule(self):
        return self.unwrap().rule

cdef class PyRoadMarkGroup(PyXmlNode):
    def __cinit__(self, string road_id = "", double lanesection_s0 = 0, int lane_id = 0, double width = 0, double height = 0, double s_offset = 0, string type = "", string weight = "", string color = "", string material = "", string lane_change = ""):
        if road_id != "":
            self.c_self = make_shared[RoadMarkGroup](road_id, lanesection_s0, lane_id, width, height, s_offset, type, weight, color, material, lane_change)

    @property
    def road_id(self):
        return self.unwrap().road_id

    @property
    def lanesection_s0(self):
        return self.unwrap().lanesection_s0

    @property
    def lane_id(self):
        return self.unwrap().lane_id

    @property
    def width(self):
        return self.unwrap().width

    @property
    def height(self):
        return self.unwrap().height

    @property
    def s_offset(self):
        return self.unwrap().s_offset

    @property
    def type(self):
        return self.unwrap().type

    @property
    def weight(self):
        return self.unwrap().weight

    @property
    def color(self):
        return self.unwrap().color

    @property
    def material(self):
        return self.unwrap().material

    @property
    def lane_change(self):
        return self.unwrap().lane_change

    @property
    def roadmark_lines(self):
        out_set = set()
        cdef libcpp_set[RoadMarksLine] c_set = self.unwrap().roadmark_lines
        cdef libcpp_set[RoadMarksLine].iterator iter = c_set.begin()
        while iter != c_set.end():
            out_set.add(PyRoadMarksLine.wrap(dereference(iter)))
            postincrement(iter)
        return out_set

cdef class PyRoadMark:
    def __cinit__(self, string road_id = "", double lanesection_s0 = 0, int lane_id = 0, double group_s0 = 0, double s_start = 0, double s_end = 0, double t_offset = 0, double width = 0, string type = ""):
        if road_id != "":
            self.c_self = make_shared[RoadMark](road_id, lanesection_s0, lane_id, group_s0, s_start, s_end, t_offset, width, type)

    @property
    def road_id(self):
        return self.unwrap().road_id

    @property
    def lanesection_s0(self):
        return self.unwrap().lanesection_s0

    @property
    def lane_id(self):
        return self.unwrap().lane_id

    @property
    def group_s0(self):
        return self.unwrap().group_s0

    @property
    def s_start(self):
        return self.unwrap().s_start

    @property
    def s_end(self):
        return self.unwrap().s_end

    @property
    def t_offset(self):
        return self.unwrap().t_offset

    @property
    def width(self):
        return self.unwrap().width

    @property
    def type(self):
        return self.unwrap().type