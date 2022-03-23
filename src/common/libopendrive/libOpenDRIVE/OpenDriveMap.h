#pragma once

#include "Mesh.h"
#include "Road.h"
#include "Lanes.h"
#include "RoadNetworkMesh.h"

#include "pugixml/pugixml.hpp"

#include <map>
#include <memory>
#include <string>

namespace odr
{
class OpenDriveMap
{
public:
    OpenDriveMap(
        std::string xodr_file, bool with_lateralProfile = true, bool with_laneHeight = true, bool center_map = true, bool with_objects = true);

    ConstRoadSet get_roads() const;
    RoadSet      get_roads();

    std::shared_ptr<odr::Lane> get_lane_from_xy(double x, double y);
    std::shared_ptr<odr::Road> get_road_from_xy(double x, double y);
    std::shared_ptr<odr::Lane> get_lane_from_xy_with_route(double x, double y, std::set<std::string> rs);

    bool road_has_signals(std::shared_ptr<const Road> road);

    Mesh3D          get_refline_lines(double eps) const;
    RoadNetworkMesh get_mesh(double eps) const;

    std::string xodr_file = "";
    std::string proj4 = "";

    pugi::xml_document xml_doc;

    double x_offs = 0;
    double y_offs = 0;

    std::map<std::string, std::shared_ptr<Road>> roads;
};

} // namespace odr