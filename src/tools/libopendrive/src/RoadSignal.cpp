#include "RoadSignal.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace odr
{
    RoadSignal::RoadSignal(std::string road_id,
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
                           std::string country) : road_id(road_id),
                                                  id(id), type(type), subtype(subtype), name(name), orientation(orientation), s(s), t(t), z0(z0), width(width),
                                                  height(height), hdg(hdg), pitch(pitch), roll(roll), country(country)
    {
    }

    Mesh3D RoadSignal::get_cylinder(const double eps, const double radius, const double height)
    {
        Mesh3D cylinder_mesh;
        cylinder_mesh.vertices.push_back({0, 0, 0});
        cylinder_mesh.vertices.push_back({0, 0, height});

        const double eps_adj = 0.5 * eps; // reduce eps a bit, cylinders more subsceptible to low resolution
        const double eps_angle =
            (radius <= eps_adj) ? M_PI / 6 : std::acos((radius * radius - 4 * radius * eps_adj + 2 * eps_adj * eps_adj) / (radius * radius));

        std::vector<double> angles;
        for (double alpha = 0; alpha < 2 * M_PI; alpha += eps_angle)
            angles.push_back(alpha);
        angles.push_back(2 * M_PI);

        for (const double &alpha : angles)
        {
            const Vec3D circle_pt_bottom = {radius * std::cos(alpha), radius * std::sin(alpha), 0};
            const Vec3D circle_pt_top = {radius * std::cos(alpha), radius * std::sin(alpha), height};
            cylinder_mesh.vertices.push_back(circle_pt_bottom);
            cylinder_mesh.vertices.push_back(circle_pt_top);

            if (cylinder_mesh.vertices.size() > 5)
            {
                const std::size_t cur_idx = cylinder_mesh.vertices.size() - 1;
                std::array<size_t, 6> top_bottom_idx_patch = {0, cur_idx - 1, cur_idx - 3, 1, cur_idx - 2, cur_idx};
                cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), top_bottom_idx_patch.begin(), top_bottom_idx_patch.end());
                std::array<size_t, 6> wall_idx_patch = {cur_idx, cur_idx - 2, cur_idx - 3, cur_idx, cur_idx - 3, cur_idx - 1};
                cylinder_mesh.indices.insert(cylinder_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
            }
        }

        return cylinder_mesh;
    }

    Mesh3D RoadSignal::get_box(const double w, const double l, const double h)
    {
        Mesh3D box_mesh;
        box_mesh.vertices = {Vec3D{l / 2, w / 2, 0},
                             Vec3D{-l / 2, w / 2, 0},
                             Vec3D{-l / 2, -w / 2, 0},
                             Vec3D{l / 2, -w / 2, 0},
                             Vec3D{l / 2, w / 2, h},
                             Vec3D{-l / 2, w / 2, h},
                             Vec3D{-l / 2, -w / 2, h},
                             Vec3D{l / 2, -w / 2, h}};
        box_mesh.indices = {0, 3, 1, 3, 2, 1, 4, 5, 7, 7, 5, 6, 7, 6, 3, 3, 6, 2, 5, 4, 1, 1, 4, 0, 0, 4, 7, 7, 3, 0, 1, 6, 5, 1, 2, 6};

        return box_mesh;
    }

} // namespace odr
