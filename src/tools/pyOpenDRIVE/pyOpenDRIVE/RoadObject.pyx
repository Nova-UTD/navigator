# distutils: language=c++

from pyOpenDRIVE cimport RoadObject

cdef class PyRoadObject:
    def __cinit__(self, string road_id = "", string id = "", double s0 = 0, double t0 = 0, double z0 = 0, double length = 0, double valid_length = 0, double width = 0, double radius = 0, double height = 0, double hdg = 0, double pitch = 0, double roll = 0, string type = "", string name = "", string orientation = "", string subtype = "", bool is_dynamic = False):
        if road_id != "":
            self.c_self = make_shared[RoadObject](road_id, id, s0, t0, z0, length, valid_length, width, radius, height, hdg, pitch, roll, type, name, orientation, subtype, is_dynamic)