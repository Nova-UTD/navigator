# distutils: language=c++

from pyOpenDRIVE cimport RefLine

cdef class PyRefLine:
    def __cinit__(self, string road_id, double length):
        self.c_self = make_shared[RefLine](road_id, length)
    
    def __cinit__(self, PyRefLine other):
        self.c_self = make_shared[RefLine](other.c_self.get()[0])