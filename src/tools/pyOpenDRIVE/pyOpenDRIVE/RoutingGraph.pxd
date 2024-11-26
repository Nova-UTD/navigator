# distutils: language=c++

cdef extern from "../src/RoutingGraph.cpp":
    pass

from libcpp.vector cimport vector
from libcpp.unordered_map cimport unordered_map
from libcpp.unordered_set cimport unordered_set
from libcpp.string cimport string
from libcpp cimport bool
from libcpp.memory cimport make_shared, shared_ptr

from pyOpenDRIVE.Lane cimport LaneKey

cdef extern from "RoutingGraph.h" namespace "odr":
    cdef cppclass RoutingGraphEdge:
        RoutingGraphEdge(LaneKey f, LaneKey to, double length) except +

        LaneKey f "from"
        LaneKey to
        double weight

    cdef cppclass WeightedLaneKey(LaneKey):
        WeightedLaneKey(const LaneKey& lane_key, double weight) except +
        WeightedLaneKey(string road_id, double lanesection_s0, int lane_id, double weight) except +

        double weight
    
    cdef cppclass RoutingGraph:
        RoutingGraph() except +
        void add_edge(const RoutingGraphEdge& edge)

        vector[LaneKey] get_lane_successors(const LaneKey& lane_key) const
        vector [LaneKey] get_lane_predecessors(const LaneKey& lane_key) const
        vector[LaneKey] shortest_path(const LaneKey& f, const LaneKey& to) const

        unordered_set[RoutingGraphEdge] edges
        unordered_map[LaneKey, unordered_set[WeightedLaneKey]] lane_key_to_successors
        unordered_map[LaneKey, unordered_set[WeightedLaneKey]] lane_key_to_predecessors

cdef class PyRoutingGraph:
    @staticmethod
    cdef inline PyRoutingGraph wrap(const RoutingGraph& c_obj):
        temp = PyRoutingGraph()
        temp.c_self = make_shared[RoutingGraph](c_obj)
        return temp

    cdef inline RoutingGraph* unwrap(this):
        return this.c_self.get()

    cdef shared_ptr[RoutingGraph] c_self