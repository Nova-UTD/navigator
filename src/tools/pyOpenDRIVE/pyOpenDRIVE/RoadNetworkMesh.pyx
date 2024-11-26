# distutils: language=c++

from pyOpenDRIVE cimport RoadNetworkMesh

cdef class PyRoadNetworkMesh:
    def __cinit__(self):
        self.c_self = make_shared[RoadNetworkMesh]()