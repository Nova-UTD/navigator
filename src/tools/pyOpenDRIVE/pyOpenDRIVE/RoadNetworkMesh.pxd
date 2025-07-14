# distutils: language=c++

cdef extern from "../src/RoadNetworkMesh.cpp":
    pass

from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.set cimport set
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

from pyOpenDRIVE.Mesh cimport Mesh3D
from pyOpenDRIVE.Array cimport array
from pyOpenDRIVE.Math cimport dim_2

cdef extern from "RoadNetworkMesh.h" namespace "odr":
    cdef cppclass RoadsMesh(Mesh3D):
        RoadsMesh() except +

        string get_road_id(const size_t vert_idx) const
        array[size_t, dim_2] get_idx_interval_road(const size_t vert_idx) const

        map[size_t, string]  road_start_indices

    cdef cppclass LanesMesh(RoadsMesh):
        LanesMesh() except +

        double get_lanesec_s0(const size_t vert_idx) const
        int get_lane_id(const size_t vert_idx) const

        array[size_t, dim_2] get_idx_interval_lanesec(const size_t vert_idx) const
        array[size_t, dim_2] get_idx_interval_lane(const size_t vert_idx) const

        vector[size_t] get_lane_outline_indices() const

        map[size_t, double] lanesec_start_indices
        map[size_t, int] lane_start_indices

    cdef cppclass RoadmarksMesh(LanesMesh):
        RoadmarksMesh() except +

        string get_roadmark_type(const size_t vert_idx) const
        array[size_t, dim_2] get_idx_interval_roadmark(const size_t vert_idx) const
        vector[size_t] get_roadmark_outline_indices() const

        map[size_t, string] roadmark_type_start_indices

    cdef cppclass RoadObjectsMesh(RoadsMesh):
        string get_road_object_id(const size_t vert_idx) const
        array[size_t, dim_2] get_idx_interval_road_object(const size_t vert_idx) const

        map[size_t, string] road_object_start_indices

    cdef cppclass RoadSignalsMesh(RoadsMesh):
        string get_road_signal_id(const size_t vert_idx) const
        array[size_t, dim_2] get_idx_interval_signal(const size_t vert_idx) const

        map[size_t, string] road_signal_start_indices

    cdef cppclass RoadNetworkMesh:
        Mesh3D get_mesh() const

        LanesMesh lanes_mesh
        RoadmarksMesh roadmarks_mesh
        RoadObjectsMesh road_objects_mesh
        RoadSignalsMesh road_signals_mesh

cdef class PyRoadNetworkMesh:
    @staticmethod
    cdef inline PyRoadNetworkMesh wrap(const RoadNetworkMesh& c_obj):
        temp = PyRoadNetworkMesh()
        temp.c_self = make_shared[RoadNetworkMesh](c_obj)
        return temp

    cdef inline RoadNetworkMesh* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[RoadNetworkMesh] c_self