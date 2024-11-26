# distutils: language=c++

from pyOpenDRIVE cimport RoadSignal

cdef class PyRoadSignal:
    def __cinit__(self, string road_id = "", string id = "", string name = "", double s0 = 0, double t0 = 0, bool is_dynamic = False, double zOffset = 0, double value = 0, double height = 0, double width = 0, double hOffset = 0, double pitch = 0, double roll = 0, string orientation = "", string country = "", string type = "", string subtype = "", string unit = "", string text = ""):
        if road_id != "":
            self.c_self = make_shared[RoadSignal](road_id, id, name, s0, t0, is_dynamic, zOffset, value, height, width, hOffset, pitch, roll, orientation, country, type, subtype, unit, text)