# distutils: language=c++

from pyOpenDRIVE cimport LaneValidityRecord

cdef class PyLaneValidityRecord:
    def __cinit__(self, int from_lane, int to_lane):
        self.c_self = make_shared[LaneValidityRecord](from_lane, to_lane)