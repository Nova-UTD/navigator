# distutils: language=c++

from cython.operator cimport dereference, postincrement

from pyOpenDRIVE cimport Junction

cdef class PyJunctionLaneLink:
    def __cinit__(self, int frm = 0, int to = 0):
        self.c_self = make_shared[JunctionLaneLink](frm, to)

    @property
    def frm(self):
        return self.unwrap().frm

    @property
    def to(self):
        return self.unwrap().to

cdef class PyJunctionConnection:
    def __cinit__(self, string id = "", string incoming_road = "", string connecting_road = "", int contact_point = 0):
        if id != "":
            self.c_self = make_shared[JunctionConnection](id, incoming_road, connecting_road, <JunctionConnection.ContactPoint>contact_point)

    @property
    def id(self):
        return self.unwrap().id

    @property
    def incoming_road(self):
        return self.unwrap().incoming_road

    @property
    def connecting_road(self):
        return self.unwrap().connecting_road

    @property
    def contact_point(self):
        return self.unwrap().contact_point

    @property
    def lane_links(self):
        out_set = set()
        cdef libcpp_set[JunctionLaneLink] c_set = self.unwrap().lane_links
        cdef libcpp_set[JunctionLaneLink].iterator iter = c_set.begin()
        while iter != c_set.end():
            out_set.add(PyJunctionLaneLink.wrap(dereference(iter)))
            postincrement(iter)
        return out_set

cdef class PyJunctionPriority:
    def __cinit__(self, string high = "", string low = ""):
        if high != "":
            self.c_self = make_shared[JunctionPriority](high, low)

    @property
    def high(self):
        return self.unwrap().high

    @property
    def low(self):
        return self.unwrap().low

cdef class PyJunctionController:
    def __cinit__(self, string id = "", string type = "", int sequence = 0):
        if id != "":
            self.c_self = make_shared[JunctionController](id, type, sequence)

    @property
    def id(self):
        return self.unwrap().id

    @property
    def type(self):
        return self.unwrap().type

    @property
    def sequence(self):
        return <int>self.unwrap().sequence

cdef class PyJunction(PyXmlNode):
    def __cinit__(self, string name = "", string id = ""):
        if name != "":
            self.c_self = make_shared[Junction](name, id)

    @property
    def name(self):
        return self.unwrap().name

    @property
    def id(self):
        return self.unwrap().id

    @property
    def id_to_connection(self):
        out_dict = {}
        cdef map[string, JunctionConnection] c_map = self.unwrap().id_to_connection
        cdef map[string, JunctionConnection].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyJunctionConnection.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict

    @property
    def id_to_controller(self):
        out_dict = {}
        cdef map[string, JunctionController] c_map = self.unwrap().id_to_controller
        cdef map[string, JunctionController].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyJunctionController.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict

    @property
    def priorities(self):
        out_set = set()
        cdef libcpp_set[JunctionPriority] c_set = self.unwrap().priorities
        cdef libcpp_set[JunctionPriority].iterator iter = c_set.begin()
        while iter != c_set.end():
            out_set.add(PyJunctionPriority.wrap(dereference(iter)))
            postincrement(iter)
        return out_set