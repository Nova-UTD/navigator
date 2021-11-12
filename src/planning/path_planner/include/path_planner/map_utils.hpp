#include "autoware_auto_msgs/msg/trajectory.hpp"
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/geometry/LineString.h>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/had_map_route.hpp>
#include <had_map_utils/had_map_utils.hpp>
#include <path_planner/lane_points.hpp>
#include <common/types.hpp>
#include <geometry/common_2d.hpp>

using namespace autoware::common::types;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::Trajectory;

namespace navigator
{
    namespace path_planner
    {
        namespace map_utils {
            LanePoints sample_center_line_and_boundaries(
            const lanelet::ConstLanelet &lanelet_obj, const float64_t resolution);
        }
    }
}