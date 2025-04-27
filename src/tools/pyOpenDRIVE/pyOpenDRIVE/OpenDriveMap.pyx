# distutils: language=c++

from cython.operator cimport dereference, postincrement

from pyOpenDRIVE.OpenDriveMap cimport OpenDriveMap

from pyOpenDRIVE.Road import PyRoad
from pyOpenDRIVE.Road cimport PyRoad

from pyOpenDRIVE.Junction import PyJunction
from pyOpenDRIVE.Junction cimport PyJunction

from pyOpenDRIVE.RoadNetworkMesh import PyRoadNetworkMesh
from pyOpenDRIVE.RoadNetworkMesh cimport PyRoadNetworkMesh

from pyOpenDRIVE.RoutingGraph import PyRoutingGraph
from pyOpenDRIVE.RoutingGraph cimport PyRoutingGraph

from pyOpenDRIVE.XmlNode import Py_xml_document
from pyOpenDRIVE.XmlNode cimport Py_xml_document

import shapely as sh
import math

cdef class PyOpenDriveMap:

    def __cinit__(self, const string& xodr_file, bool center_map = False, bool with_road_objects = True, bool with_lateral_profile = True, bool with_lane_height = True, bool abs_z_for_for_local_road_obj_outline = False, bool fix_spiral_edge_cases = True, bool with_road_signals = True):
        self.c_self = make_shared[OpenDriveMap](xodr_file, center_map, with_road_objects, with_lateral_profile, with_lane_height, abs_z_for_for_local_road_obj_outline, fix_spiral_edge_cases, with_road_signals)

    def get_roads(self):
        out_arr = []
        cdef vector[Road] c_objs = self.unwrap().get_roads()
        for i in range(c_objs.size()):
            out_val = PyRoad.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    def get_junctions(self):
        out_arr = []
        cdef vector[Junction] c_objs = self.unwrap().get_junctions()
        for i in range(c_objs.size()):
            out_val = PyJunction.wrap(c_objs[i])
            out_arr.append(out_val)
        return out_arr

    def get_road_network_mesh(self, const double eps):
        return PyRoadNetworkMesh.wrap(self.unwrap().get_road_network_mesh(eps))

    def get_routing_graph(self):
        return PyRoutingGraph.wrap(self.unwrap().get_routing_graph())

    @property
    def proj4(self):
        return self.unwrap().proj4

    @property
    def x_offs(self):
        return self.unwrap().x_offs

    @property
    def y_offs(self):
        return self.unwrap().y_offs

    @property
    def xodr_file(self):
        return self.unwrap().xodr_file

    @property
    def xml_doc(self):
        return Py_xml_document.wrap(self.unwrap().xml_doc)
    
    @property
    def id_to_road(self):
        out_dict = {}
        cdef map[string, Road] c_map = self.unwrap().id_to_road
        cdef map[string, Road].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyRoad.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict

    @property
    def id_to_junction(self):
        out_dict = {}
        cdef map[string, Junction] c_map = self.unwrap().id_to_junction
        cdef map[string, Junction].iterator iter = c_map.begin()
        while iter != c_map.end():
            out_dict[dereference(iter).first] = PyJunction.wrap(dereference(iter).second)
            postincrement(iter)
        return out_dict

    def generate_mesh_tree(self):
        print("Start of generate_mesh_tree()")

        polys = self.get_lane_polygons(1.0, False)
        print("get_lane_polygons returned %i shapes\n" % len(polys))

        boxes = []

        # fill the spatial index
        for idx, poly in enumerate(polys):
            # calculate polygon bounding box
            box = sh.envelope(poly[1])
            # insert new value
            boxes.append(box)

        # Create RTree with max of 16 geoms/node
        rtree = sh.STRtree(boxes, 16)

        print("End of generate_mesh_tree(), tree has %i shapes\n" % len(rtree.geometries));
        return rtree

    def get_lane_polygons(self, float res, bool drivable_only):
        if drivable_only and self.drivable_lane_polygons_:
            return self.drivable_lane_polygons_
        if self.lane_polygons_:
            return self.lane_polygons_

        polys = []

        idx = 0
        for road in self.get_roads():
            for lsec in road.get_lanesections():
                for lane in lsec.get_lanes():
                    if drivable_only and lane.type != "driving":
                        continue
                    s_end = road.get_lanesection_end(lane.key.lanesection_s0)
                    s_start = lane.key.lanesection_s0

                    s_vals = road.ref_line.approximate_linear(res, s_start, s_end)
                    s_vals_outer_brdr = lane.outer_border.approximate_linear(res, s_start, s_end)
                    s_vals.update(s_vals_outer_brdr)
                    s_vals_inner_brdr = lane.inner_border.approximate_linear(res, s_start, s_end)
                    s_vals.update(s_vals_inner_brdr)
                    s_vals_lane_offset = road.lane_offset.approximate_linear(res, s_start, s_end)
                    s_vals.update(s_vals_lane_offset)

                    s_vals_lane_height = lane.s_to_height_offset.keys()
                    s_vals.update(s_vals_lane_height)

                    t_max = lane.outer_border.get_max(s_start, s_end)
                    if t_max != 0:
                        s_vals_superelev = road.superelevation.approximate_linear(math.atan(res / abs(t_max)), s_start, s_end)
                    elif res >= 0:
                        s_vals_superelev = road.superelevation.approximate_linear(math.atan(float('inf')), s_start, s_end)
                    else:
                        s_vals_superelev = road.superelevation.approximate_linear(math.atan(float('-inf')), s_start, s_end)
                    s_vals.update(s_vals_superelev)

                    # make s_vals a list now so it's indexable
                    s_vals = list(s_vals)

                    # thin out s_vals array, be removing s vals closer than res to each other
                    svals_idx = 1
                    while svals_idx < len(s_vals)-1:
                        if s_vals[svals_idx] - s_vals[svals_idx-1] <= res:
                            del s_vals[svals_idx]
                        else:
                            svals_idx = svals_idx+1

                    outer_pts = []
                    inner_pts = []

                    start_pt_added = False
                    ring_pts = []

                    for s in s_vals:
                        t_inner_brdr = lane.inner_border.get(s)
                        inner_border_pt = road.get_surface_pt(s, t_inner_brdr).array
                        
                        ring_pts.append(sh.Point(inner_border_pt[0], inner_border_pt[1]))
                        if not start_pt_added:
                            start_pt = sh.Point(inner_border_pt[0], inner_border_pt[1])
                            start_pt_added = True
                    for s in s_vals:
                        t_outer_brdr = lane.outer_border.get(s_end - s)
                        outer_border_pt = road.get_surface_pt(s_end - s, t_outer_brdr).array

                        ring_pts.append(sh.Point(outer_border_pt[0], outer_border_pt[1]))
                    ring_pts.append(start_pt) # close the ring
                    lane_ring = sh.LinearRing(ring_pts)
                    polys.append((lane, lane_ring))
                    idx = idx+1

        if drivable_only:
            self.drivable_lane_polygons_ = polys
        else:
            self.lane_polygons_ = polys
        return polys