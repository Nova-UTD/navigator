# distutils: language=c++

#cdef extern from "pugixml/pugixml.cpp" namespace "pugi":
#    pass

from libcpp.memory cimport make_shared, shared_ptr

from pyOpenDRIVE.XmlNode cimport XmlNode

cdef extern from "LaneValidityRecord.h" namespace "odr":
    cdef cppclass LaneValidityRecord(XmlNode):
        LaneValidityRecord(int from_lane, int to_lane) except +

        int from_lane
        int to_lane

cdef class PyLaneValidityRecord:
    cdef shared_ptr[LaneValidityRecord] c_self