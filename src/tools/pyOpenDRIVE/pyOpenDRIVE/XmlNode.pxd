# distutils: language=c++

cdef extern from "pugixml/pugixml.cpp" namespace "pugi":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

cdef extern from "pugixml/pugixml.hpp" namespace "pugi":
    cdef cppclass char_t

    cdef enum xml_node_type:
        node_null,
        node_document,
        node_element,
        node_pcdata,
        node_cdata,
        node_comment,
        node_pi,
        node_declaration,
        node_doctype

    # NOTE: Partially wrapped, may not need to be wrapped fully
    cdef cppclass xml_node:
        xml_node() except +

        bool operator==(const xml_node& r) const
        bool operator!=(const xml_node& r) const
        bool operator<(const xml_node& r) const
        bool operator>(const xml_node& r) const
        bool operator<=(const xml_node& r) const
        bool operator>=(const xml_node& r) const
        
        bool empty() const

        xml_node_type type() const
        const char_t* name() const
        const char_t* value() const

    # This likely doesn't need to be wrapped, just included to prevent type error
    cdef cppclass xml_document(xml_node):
        xml_document() except +

        void reset()
        void reset(const xml_document& proto)

cdef extern from "XmlNode.h" namespace "odr":
    cdef cppclass XmlNode:
        xml_node node "xml_node"

# Wrapper for raw pugixml xml_node type
cdef class Py_xml_node:
    @staticmethod
    cdef inline Py_xml_node wrap_node(const xml_node& c_obj):
        temp = Py_xml_node()
        temp.node_ptr = make_shared[xml_node](c_obj)
        return temp

    cdef inline xml_node* unwrap_node(this):
        return this.node_ptr.get()

    cdef shared_ptr[xml_node] node_ptr

# Wrapper for pyOpenDRIVE XmlNode type which is wrapper for pugixml xml_node
cdef class PyXmlNode:
    @staticmethod
    cdef inline PyXmlNode wrap_node(const XmlNode& c_obj):
        temp = PyXmlNode()
        temp.node_ptr = make_shared[XmlNode](c_obj)
        return temp

    cdef inline XmlNode* unwrap_node(this):
        return this.node_ptr.get()

    cdef shared_ptr[XmlNode] node_ptr

# Wrapper for raw pugixml xml_document type
cdef class Py_xml_document(Py_xml_node):
    @staticmethod
    cdef inline Py_xml_document wrap(const xml_document& c_obj):
        temp = Py_xml_document()
        temp.c_self = make_shared[xml_document]()
        temp.unwrap().reset(c_obj)
        return temp

    cdef inline xml_document* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[xml_document] c_self