# distutils: language=c++

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

cdef extern from "<array>" namespace "std":
    cdef cppclass array[T, N]:
        ctypedef T value_type
        ctypedef T* pointer
        ctypedef const T* const_pointer
        ctypedef T& reference
        ctypedef const T& const_reference
        ctypedef size_t size_type
        ctypedef ptrdiff_t difference_type
        
        cppclass iterator:
            T operator*()
            iterator operator++()
            bint operator==(iterator)
            bint operator!=(iterator)

        void fill(const T& u)
        void swap(array&)

        iterator begin()
        iterator end()

        size_type size() const
        size_type max_size() const

        reference operator[](size_type n)
        #const_reference operator[](size_type n) const
        reference at(size_type n)
        #const_reference at(size_type n) const
        reference front()
        #const_reference front() const
        reference back()
        #const_reference back() const

        pointer data()
        #const_pointer data() const