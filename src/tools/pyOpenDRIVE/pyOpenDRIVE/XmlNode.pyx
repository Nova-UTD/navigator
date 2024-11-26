# distutils: language=c++

from pyOpenDRIVE cimport XmlNode

cdef class Py_xml_node:
    def __cinit__(self):
        self.node_ptr = make_shared[xml_node]()

cdef class PyXmlNode:
    def __cinit__(self):
        self.node_ptr = make_shared[XmlNode]()

    @property
    def xml_node(self):
        return Py_xml_node.wrap_node(self.unwrap_node().node)

cdef class Py_xml_document(Py_xml_node):
    def __cinit__(self):
        self.c_self = make_shared[xml_document]()

    def empty(self):
        return self.unwrap().empty();

    def type(self):
        return self.unwrap().type();
    def name(self):
        return <string>self.unwrap().name();
    def value(self):
        return <string>self.unwrap().value();