# distutils: language=c++

from pyOpenDRIVE cimport RoutingGraph

cdef class PyRoutingGraph:
    def __cinit__(self):
        self.c_self = make_shared[RoutingGraph]()