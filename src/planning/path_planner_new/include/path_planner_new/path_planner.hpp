#ifndef PATH_PLANNER__PATH_PLANNER_HPP_
#define PATH_PLANNER__PATH_PLANNER_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <voltron_msgs/msg/route_cost.hpp>


namespace navigator
{
    namespace path_planner
    {
        
        using LaneletSequence = std::vector<lanelet::Id>;

        class PathPlanner{
            public:

            PathPlanner();

            void set_map(lanelet::LaneletMapPtr map);
            void set_routing_costs(std::vector<voltron_msgs::msg::RouteCost> costs);
            
            // designed for recursion
            void get_lanelet_sequences(double remaining_distance,
                LaneletSequence previous, std::vector<LaneletSequence> &results);

            // Todo: return types
            void getCruisePaths(); // for normal driving: drive forward and lane change
            void getSpecialPaths(); // for special paths like pull over, maybe not needed

            private:
            lanelet::LaneletMapPtr map;
            std::map<lanelet::Id, double> route_costs;
        };
    }
}

#endif