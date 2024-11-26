# distutils: language=c++

from cython.operator cimport dereference, postincrement

from pyOpenDRIVE cimport Road

from pyOpenDRIVE.CubicSpline import PyCubicSpline
from pyOpenDRIVE.CubicSpline cimport PyCubicSpline

from pyOpenDRIVE.RefLine import PyRefLine
from pyOpenDRIVE.RefLine cimport PyRefLine

from pyOpenDRIVE.LaneSection import PyLaneSection
from pyOpenDRIVE.LaneSection cimport PyLaneSection

from pyOpenDRIVE.RoadObject import PyRoadObject
from pyOpenDRIVE.RoadObject cimport PyRoadObject

from pyOpenDRIVE.RoadSignal import PyRoadSignal
from pyOpenDRIVE.RoadSignal cimport PyRoadSignal

from pyOpenDRIVE.Math cimport Vec3D
from pyOpenDRIVE.Math import PyVec3D
from pyOpenDRIVE.Math cimport PyVec3D

from pyOpenDRIVE.Math import PyLine3D
from pyOpenDRIVE.Math cimport PyLine3D

from pyOpenDRIVE.Mesh import PyMesh3D
from pyOpenDRIVE.Mesh cimport PyMesh3D

from pyOpenDRIVE.Lane import PyLane
from pyOpenDRIVE.Lane cimport PyLane

from pyOpenDRIVE.RoadMark import PyRoadMark
from pyOpenDRIVE.RoadMark cimport PyRoadMark

cdef class PyCrossfall:
    def __cinit__(self):
        self.c_self = make_shared[Crossfall]()

    def get_crossfall(self, const double s, const bool on_left_side):
        return self.unwrap().get_crossfall(s, on_left_side)

    @property
    def sides(self):
        return self.unwrap().sides

cdef class PyRoadLink:
    def __cinit__(self, string id = "", int type = 0, int contact_point = 0):
        if id != "":
            self.c_self = make_shared[RoadLink](id, <RoadLink.Type>type, <RoadLink.ContactPoint>contact_point)
        else:
            self.c_self = make_shared[RoadLink]()

    @property
    def id(self):
        return self.unwrap().id

    @property
    def type(self):
        return <int>self.unwrap().type

    @property
    def contact_point(self):
        return <int>self.unwrap().contact_point

cdef class PyRoadNeighbor:
    def __cinit__(self, string id = "", string side = "", string direction = ""):
        if id != "":
            self.c_self = make_shared[RoadNeighbor](id, side, direction)

    @property
    def id(self):
        return self.unwrap().id

    @property
    def side(self):
        return self.unwrap().side

    @property
    def direction(self):
        return self.unwrap().direction

cdef class PySpeedRecord:
    def __cinit__(self, string max = "", string unit = ""):
        if id != "":
            self.c_self = make_shared[SpeedRecord](max, unit)

    @property
    def max(self):
        return self.unwrap().max

    @property
    def unit(self):
        return self.unwrap().unit

cdef class PyRoad:
    def __cinit__(self, string id = "", double length = 0, string junction = "", string name = "", bool left_hand_traffic = False):
        if id != "":
            self.c_self = make_shared[Road](id, length, junction, name, left_hand_traffic)

    def get_lanesections(self):
        out_arr = []
        cdef vector[LaneSection] c_objs = self.unwrap().get_lanesections()
        for i in range(c_objs.size()):
            out_val = PyLaneSection.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    def get_road_objects(self):
        out_arr = []
        cdef vector[RoadObject] c_objs = self.unwrap().get_road_objects()
        for i in range(c_objs.size()):
            out_val = PyRoadObject.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    def get_road_signals(self):
        out_arr = []
        cdef vector[RoadSignal] c_objs = self.unwrap().get_road_signals()
        for i in range(c_objs.size()):
            out_val = PyRoadSignal.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    def get_lanesection_s0(self, const double s):
        return self.unwrap().get_lanesection_s0(s)

    def get_lanesection(self, const double s):
        return PyLaneSection(self.id, self.unwrap().get_lanesection_s0(s))

    def get_lanesection_end(self, lanesection):
        if isinstance(lanesection, PyLaneSection):
            return self.unwrap().get_lanesection_end((<PyLaneSection>lanesection).unwrap()[0])
        else:
            return self.unwrap().get_lanesection_end(<double>lanesection)    

    def get_lanesection_length(self, lanesection):
        if isinstance(lanesection, PyLaneSection):
            return self.unwrap().get_lanesection_length((<PyLaneSection>lanesection).unwrap()[0])
        else:
            return self.unwrap().get_lanesection_length(<double>lanesection)

    def get_xyz(self, const double s, const double t, const double h, PyVec3D e_s = None, PyVec3D e_t = None, PyVec3D e_h = None):
        return PyVec3D.wrap(self.unwrap().get_xyz(s, t, h, e_s.unwrap() if e_s != None else NULL, e_t.unwrap() if e_t != None else NULL, e_h.unwrap() if e_h != None else NULL))

    def get_surface_pt(self, double s, const double t, PyVec3D vn = None):
        return PyVec3D.wrap(self.unwrap().get_surface_pt(s, t, vn.unwrap() if vn != None else NULL))

    def get_lane_border_line(self, PyLane lane, const double s_start = 0, const double s_end = 0, const double eps = 0, const bool outer = True):
        if s_start != 0 and s_end != 0:
            return PyLine3D.wrap(self.unwrap().get_lane_border_line(lane.unwrap()[0], s_start, s_end, eps, outer))
        else:
            return PyLine3D.wrap(self.unwrap().get_lane_border_line(lane.unwrap()[0], eps, outer))

    def get_lane_mesh(self, PyLane lane, const double s_start = 0, const double s_end = 0, const double eps = 0, outline_indices = []):
        cdef vector[uint32_t] c_vec
        for i in range(len(outline_indices)):
            c_vec.push_back(<uint32_t>outline_indices[i])
        if s_start != 0 and s_end != 0:
            return PyMesh3D.wrap(self.unwrap().get_lane_mesh(lane.unwrap()[0], s_start, s_end, eps, &c_vec))
        else:
            return PyMesh3D.wrap(self.unwrap().get_lane_mesh(lane.unwrap()[0], eps, &c_vec))     

    def get_roadmark_mesh(self, PyLane lane, PyRoadMark roadmark, const double eps):
        return PyMesh3D.wrap(self.unwrap().get_roadmark_mesh(lane.unwrap()[0], roadmark.unwrap()[0], eps))

    def get_road_signal_mesh(self, PyRoadSignal road_signal):
        return PyMesh3D.wrap(self.unwrap().get_road_signal_mesh(road_signal.unwrap()[0]))

    def get_road_object_mesh(self, PyRoadObject road_object, const double eps):
        return PyMesh3D.wrap(self.unwrap().get_road_object_mesh(road_object.unwrap()[0], eps))

    def approximate_lane_border_linear(self, PyLane lane, const double s_start = 0, const double s_end = 0, const double eps = 0, const bool outer = True):
        if s_start != 0 and s_end != 0:
            return self.unwrap().approximate_lane_border_linear(lane.unwrap()[0], s_start, s_end, eps, outer)
        else:
            return self.unwrap().approximate_lane_border_linear(lane.unwrap()[0], eps, outer)

    @property
    def length(self):
        return self.unwrap().length

    @property
    def id(self):
        return self.unwrap().id

    @property
    def junction(self):
        return self.unwrap().junction

    @property
    def name(self):
        return self.unwrap().name

    @property
    def left_hand_traffic(self):
        return self.unwrap().left_hand_traffic

    @property
    def predecessor(self):
        return PyRoadLink.wrap(self.unwrap().predecessor)

    @property
    def successor(self):
        return PyRoadLink.wrap(self.unwrap().successor)

    @property
    def neighbors(self):
        out_arr = []
        cdef vector[RoadNeighbor] c_objs = self.unwrap().neighbors
        for i in range(c_objs.size()):
            out_val = PyRoadNeighbor.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    @property
    def lane_offset(self):
        return PyCubicSpline.wrap(self.unwrap().lane_offset)

    @property
    def superelevation(self):
        return PyCubicSpline.wrap(self.unwrap().superelevation)

    @property
    def crossfall(self):
        return PyCrossfall.wrap(self.unwrap().crossfall)

    @property
    def ref_line(self):
        return PyRefLine.wrap(self.unwrap().ref_line)

    @property
    def s_to_lanesection(self):
        out_dict = {}
        cdef map[double, LaneSection] c_map = self.unwrap().s_to_lanesection
        cdef map[double, LaneSection].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyLaneSection.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict

    @property
    def s_to_type(self):
        out_dict = {}
        cdef map[double, string] c_map = self.unwrap().s_to_type
        cdef map[double, string].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = dereference(iter).second
            postincrement(iter)
        return out_dict

    @property
    def s_to_speed(self):
        out_dict = {}
        cdef map[double, SpeedRecord] c_map = self.unwrap().s_to_speed
        cdef map[double, SpeedRecord].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PySpeedRecord.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict

    @property
    def id_to_object(self):
        out_dict = {}
        cdef map[string, RoadObject] c_map = self.unwrap().id_to_object
        cdef map[string, RoadObject].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyRoadObject.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict

    @property
    def id_to_signal(self):
        out_dict = {}
        cdef map[string, RoadSignal] c_map = self.unwrap().id_to_signal
        cdef map[string, RoadSignal].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyRoadSignal.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict