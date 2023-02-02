#pragma once
#include "Math.hpp"
#include "Mesh.h"
#include "XmlNode.h"

#include <string>
#include <vector>

namespace odr
{

    struct RoadSignal : public XmlNode
    {
        RoadSignal(std::string road_id,
                   std::string id,
                   double s,
                   double t,
                   double z0,
                   bool dynamic,
                   double width,
                   double height,
                   double hdg,
                   double pitch,
                   double roll,
                   std::string type,
                   std::string subtype,
                   std::string name,
                   std::string orientation,
                   std::string country);

        static Mesh3D get_cylinder(const double eps, const double radius, const double height);
        static Mesh3D get_box(const double width, const double length, const double height);

        std::string road_id;
        std::string id;
        double s;
        double t;
        double z0;
        bool dynamic;
        double width;
        double height;
        double hdg;
        double pitch;
        double roll;
        std::string type;
        std::string subtype;
        std::string name;
        std::string orientation;
        std::string country;
    };

} // namespace odr