# distutils: language=c++

from cython.operator cimport dereference, postincrement

from pyOpenDRIVE cimport XmlNode

# TODO: Implement PyPoly3?

cdef class PyCubicSpline:
    def __cinit__(self):
        self.c_self = make_shared[CubicSpline]()

    def get(self, double s, double default_val = 0, bool extend_start = True):
        return self.unwrap().get(s, default_val, extend_start)

    def get_grad(self, double s, double default_val = 0.0, bool extend_start = True):
        return self.unwrap().get_grad(s, default_val, extend_start)

    def get_max(self, double s_start, double s_end):
        return self.unwrap().get_max(s_start, s_end)

    # TODO: Implement get_poly?

    def empty(self):
        return self.unwrap().empty()

    def size(self):
        return self.unwrap().size()

    # TODO: Implement negate() and add()?

    def approximate_linear(self, double eps, double s_start, double s_end):
        out_set = set()
        cdef libcpp_set[double] c_set = self.unwrap().approximate_linear(eps, s_start, s_end)
        cdef libcpp_set[double].iterator iter = c_set.begin()
        while iter != c_set.end():
            out_set.add(dereference(iter))
            postincrement(iter)
        return out_set

    # TODO: Implement s0_to_poly?