# distutils: language=c++

cdef extern from "../src/Junction.cpp":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set as libcpp_set # Alias to prevent name collision with Python's set()
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

from pyOpenDRIVE.XmlNode cimport XmlNode, PyXmlNode

cdef extern from "<cstdint>" namespace "std":
    cdef cppclass uint32_t

cdef extern from "Junction.h" namespace "odr":

    cdef cppclass JunctionLaneLink:
        JunctionLaneLink(int f, int to) except +

        int frm "from" # Workaround here, since having a field or variable named "from" causes cython to throw syntax errors...
        int to

    cdef cppclass JunctionConnection:
        enum ContactPoint:
            ContactPoint_None,
            ContactPoint_Start,
            ContactPoint_End
        
        JunctionConnection(string id, string incoming_road, string connecting_road, ContactPoint contact_point) except +

        string id
        string incoming_road
        string connecting_road
        ContactPoint contact_point

        libcpp_set[JunctionLaneLink] lane_links

    cdef cppclass JunctionPriority:
        JunctionPriority(string high, string low) except +

        string high
        string low
    
    cdef cppclass JunctionController:
        JunctionController(string id, string type, uint32_t sequence) except +

        string id
        string type
        uint32_t sequence

    cdef cppclass Junction(XmlNode):
        Junction(string name, string id) except +

        string name
        string id

        map[string, JunctionConnection] id_to_connection
        map[string, JunctionController] id_to_controller
        libcpp_set[JunctionPriority] priorities

cdef class PyJunction(PyXmlNode):
    @staticmethod
    cdef inline PyJunction wrap(const Junction& c_obj):
        temp = PyJunction()
        temp.c_self = make_shared[Junction](c_obj)
        return temp

    cdef inline Junction* unwrap(this):
        return this.c_self.get()
        
    cdef shared_ptr[Junction] c_self

cdef class PyJunctionLaneLink:
    @staticmethod
    cdef inline PyJunctionLaneLink wrap(const JunctionLaneLink& c_obj):
        temp = PyJunctionLaneLink()
        temp.c_self = make_shared[JunctionLaneLink](c_obj)
        return temp

    cdef inline JunctionLaneLink* unwrap(this):
        return this.c_self.get()
        
    cdef shared_ptr[JunctionLaneLink] c_self

cdef class PyJunctionConnection:
    @staticmethod
    cdef inline PyJunctionConnection wrap(const JunctionConnection& c_obj):
        temp = PyJunctionConnection()
        temp.c_self = make_shared[JunctionConnection](c_obj)
        return temp

    cdef inline JunctionConnection* unwrap(this):
        return this.c_self.get()
        
    cdef shared_ptr[JunctionConnection] c_self

cdef class PyJunctionPriority:
    @staticmethod
    cdef inline PyJunctionPriority wrap(const JunctionPriority& c_obj):
        temp = PyJunctionPriority()
        temp.c_self = make_shared[JunctionPriority](c_obj)
        return temp

    cdef inline JunctionPriority* unwrap(this):
        return this.c_self.get()
        
    cdef shared_ptr[JunctionPriority] c_self

cdef class PyJunctionController:
    @staticmethod
    cdef inline PyJunctionController wrap(const JunctionController& c_obj):
        temp = PyJunctionController()
        temp.c_self = make_shared[JunctionController](c_obj)
        return temp

    cdef inline JunctionController* unwrap(this):
        return this.c_self.get()
        
    cdef shared_ptr[JunctionController] c_self