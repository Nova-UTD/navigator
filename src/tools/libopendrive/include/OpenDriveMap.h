#pragma once
#include "Junction.h"
#include "Road.h"
#include "RoadNetworkMesh.h"
#include "RoutingGraph.h"

#include <pugixml.hpp>

#include <map>
#include <string>
#include <vector>
#include <unordered_map>


// Boost headers
#include <boost/foreach.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

namespace odr
{
    typedef bg::model::point<float, 2, bg::cs::cartesian> point;
    typedef bg::model::box<point> box;
    typedef bg::model::polygon<point> polygon;
    typedef bg::model::multi_polygon<polygon> multipolygon;
    typedef bg::model::ring<point> ring;
    typedef std::pair<box, unsigned> value;
    typedef std::pair<Lane, ring> LanePair;

    struct OpenDriveMapConfig
    {
        bool with_lateralProfile = true;
        bool with_laneHeight = true;
        bool with_road_objects = true;
        bool center_map = false;
        bool abs_z_for_for_local_road_obj_outline = false;
    };

    class OpenDriveMap
    {
    public:
        OpenDriveMap(const std::string &xodr_file, bool from_string = true, const OpenDriveMapConfig &config = OpenDriveMapConfig{});

        std::vector<Road> get_roads() const;
        std::vector<Junction> get_junctions() const;

        RoutingGraph get_routing_graph() const;

        std::string proj4 = "";
        double x_offs = 0;
        double y_offs = 0;
        const std::string xodr_file = "";
        pugi::xml_document xml_doc;

        std::map<std::string, Road> id_to_road;
        std::map<std::string, Junction> id_to_junction;
        bgi::rtree<value, bgi::rstar<16, 4>> generate_mesh_tree();
        bgi::rtree<value, bgi::rstar<16, 4>> generate_object_tree();
        std::unique_ptr<RoadNetworkMesh> road_mesh_;

        std::vector<ring> get_drivable_lane_polygons(float res);
        std::vector<std::pair<Lane, ring>> get_lane_polygons(float res = 1.0, bool drivable_only = true);
        std::vector<std::pair<RoadObject, point>> get_road_object_centers();

        RoadNetworkMesh get_road_network_mesh(double eps);

    private:
        std::unique_ptr<std::vector<std::pair<Lane, ring>>> drivable_lane_polygons_;
        std::unique_ptr<std::vector<std::pair<Lane, ring>>> lane_polygons_;
        std::vector<std::pair<RoadObject, point>> object_centers_;
        std::unique_ptr<std::vector<ring>> road_polygons_;
        std::unique_ptr<bgi::rtree<value, bgi::rstar<16, 4>>> rtree_;
    };

} // namespace odr