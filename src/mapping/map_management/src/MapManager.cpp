/*
 * Package:   map_management
 * Filename:  MapManager.cpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include "rclcpp/rclcpp.hpp"

#include "map_management/MapManager.hpp"

#include <algorithm>
#include <chrono>
#include <list>

#include <fstream>

using namespace navigator::planning;

struct RoiIndices
{
    int start = -1;
    int center = -1;
    int end = -1;
};

struct Arc
{
    odr::LaneKey sourceKey;
    odr::LaneKey targetKey;
    double sourceLength;
};

MapManagementNode::MapManagementNode() : Node("map_management_node")
{
    // Callback groups & options
    mutex_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    parallel_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions mutex_sub_option;
    mutex_sub_option.callback_group = mutex_group_;
    rclcpp::SubscriptionOptions parallel_sub_option;
    parallel_sub_option.callback_group = parallel_group_;

    rclcpp::PublisherOptions mutex_pub_option;
    mutex_pub_option.callback_group = mutex_group_;
    rclcpp::PublisherOptions parallel_pub_option;
    parallel_pub_option.callback_group = parallel_group_;

    // Params
    this->declare_parameter("from_file", false);
    this->declare_parameter("data_path", "/home/nova/navigator/data");
    this->declare_parameter("route_path", "/home/nova/navigator/data/maps/route_wkt.txt");

    // Load map from file if from_file is true
    bool do_load_from_file = this->get_parameter("from_file").as_bool();

    std::string data_path = get_parameter("data_path").as_string();
    std::string route_path = this->get_parameter("route_path").as_string();

    // Publishers and subscribers
    drivable_grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/drivable", 10, parallel_pub_option);
    junction_grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/junction", 10, parallel_pub_option);
    //route_dist_grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/route_distance", 10);
    route_path_pub_ = this->create_publisher<Path>("/planning/smoothed_route", 10, parallel_pub_option);
    goal_pose_pub_ = this->create_publisher<PoseStamped>("/planning/goal_pose", 1, parallel_pub_option);
    //route_progress_pub_ = this->create_publisher<std_msgs::msg::Float32>("/route_progress", 1);
    clock_sub = this->create_subscription<Clock>("/clock", 10, bind(&MapManagementNode::clockCb, this, std::placeholders::_1), parallel_sub_option);
    clicked_point_sub_ = this->create_subscription<PointStamped>("/clicked_point", 1, bind(&MapManagementNode::clickedPointCb, this, std::placeholders::_1), mutex_sub_option);

    // Services
    route_set_service_ = this->create_service<navigator_msgs::srv::SetRoute>("set_route", bind(&MapManagementNode::setRoute, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, mutex_group_);

    // only need route timer if we want the route distance grid, but that has been commented out
    //route_timer_ = this->create_wall_timer(LOCAL_ROUTE_LS_FREQ, bind(&MapManagementNode::updateLocalRouteLinestring, this));
    
    smooth_route_timer_ = this->create_wall_timer(SMOOTH_ROUTE_LS_FREQ, bind(&MapManagementNode::publishSmoothRoute, this), mutex_group_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (do_load_from_file)
    {
        RCLCPP_INFO(get_logger(), "Loading map from %s/maps/campus.xodr", data_path.c_str());
        this->map_ = new odr::OpenDriveMap(data_path+"/maps/campus.xodr", false);
        this->lane_polys_ = map_->get_lane_polygons(1.0, false);

        RCLCPP_INFO(get_logger(), "Map loaded with %i roads", map_->get_roads().size());

        if (this->map_wide_tree_.size() == 0)
        {
            RCLCPP_INFO(get_logger(), "map_wide_tree_ was empty. Generating.");
            this->map_wide_tree_ = this->map_->generate_mesh_tree();
        }

        buildTrueRoutingGraph();

        setPredeterminedRoute();

        // setRouteFromClickedPt(PointStamped());

        // Temporary linestring from file
        RCLCPP_INFO(get_logger(), "Loading route from %s", route_path.c_str());
        std::ifstream ifs(route_path);
        std::string wkt_str( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()) );
        
        boost::geometry::read_wkt<bg::model::linestring<odr::point>>(wkt_str.c_str(), route_linestring_);
        // boost::geometry::read_wkt<bg::model::linestring<odr::point>>("LINESTRING (-1785.5709300144865 500.47158225551465, -1783.51573231485 500.426477889891, -1780.6739054100492 500.3795679204954, -1778.4010968628584 500.34884278664725, -1776.1958928441231 500.31184580977424, -1773.688495800675 500.2895820615064, -1771.5040078903626 500.2583199311353, -1768.7392892089179 500.1944219703132, -1766.5241186105725 500.1426381243812, -1764.3136214656129 500.08845269001256, -1762.0990727336323 500.04313937155604, -1759.8915296822818 500.0194541509073, -1757.68507812045 499.98948504289785, -1755.497315893976 499.9754160542061, -1753.342420362035 499.96357149904264, -1751.2876863581755 499.9373276887229, -1748.5438410566176 499.86974231084747, -1746.2394450887848 499.64900290332207, -1744.2142545991442 499.0397787080288, -1742.5008434688584 497.9376404137962, -1741.183330800356 496.3178341177956, -1740.322162930486 494.1337905419717, -1739.9896603515517 492.01823157983966, -1739.8773787302384 489.2223135223487, -1739.913493575646 486.7134791086838, -1739.908414630678 484.60555657054573, -1739.8622249629875 482.42886607977044, -1739.8361276821756 480.26197569941684, -1739.800055413954 478.12780033877357, -1739.7586833017647 476.0145110290066, -1739.704693551921 473.90325274094494, -1739.6633174454214 471.8123291509038, -1739.6046548519844 469.6995913235776, -1739.567803256984 467.5676339455309, -1739.5437210553405 465.4561961332065, -1739.514340480081 463.357326546673, -1739.4995945118299 461.30171224370235, -1739.4645911946768 459.2878683253042, -1739.417509521431 456.86903664856015, -1739.3232198651872 454.50657299504644, -1739.2594491459688 452.2104725029271, -1739.2071941227584 449.977589519413, -1739.2910660890177 447.83031178945316, -1739.7030389504323 445.39381665915545, -1740.5380355508755 443.21395508362457, -1741.8912084972449 441.418099154745, -1743.7622291392117 440.10347514617996, -1746.1286470860184 439.38024432790803, -1748.3332725265468 439.2444085434031, -1750.6894357796405 439.2527758501802, -1753.1675477799545 439.365045844081, -1755.2358394257853 439.4301089706463, -1757.3748612514996 439.43363327118027, -1759.564048357831 439.39465368999623, -1761.7443538557068 439.3711996081456, -1763.90036842507 439.2972801863837, -1766.0417490553843 439.17696363509856, -1768.189679554088 439.0148747653776, -1770.3334129749385 438.8049119106266, -1772.4678089635186 438.5470741396735, -1774.60189879693 438.2600319737102, -1776.7264915419319 437.94692603578164, -1778.852643719421 437.6249484953247, -1781.0035722123616 437.25399315875796, -1783.1780264433644 436.8586436080242, -1785.4057608073906 436.4254118985683, -1788.2620448678192 435.878802881793, -1790.6128468357235 435.3936545420111, -1793.033421294417 434.9763559635073, -1795.520660846115 434.582914580696, -1798.0655185296062 434.2807955886815, -1800.618620771449 434.0407851837233, -1803.1445983067208 433.9151867624361, -1805.5997000045643 433.79438182887833, -1807.9858035213863 433.63253022527863, -1810.3162852350167 433.53721155516985, -1812.5526928790355 433.3847603711799, -1814.7298966355513 433.29532939216966, -1816.8307692851881 433.22862018183355, -1818.8401774652443 433.31290922012056, -1821.2366386834458 433.57859891017904, -1823.4246158153837 434.1930446213779, -1825.28055744744 435.29744145197907, -1826.6058636952457 436.85922039426265, -1827.4924584066912 438.7710059646407, -1827.8984478098546 440.9899072531039, -1827.9993995491411 443.4322240753073, -1827.8935545968318 445.5264477283775, -1827.7563795661563 447.73120020937483, -1827.6293116139889 450.0326261793703, -1827.579802487167 452.4104057180048, -1827.5385356031736 454.86101397029, -1827.4927740715855 458.0280633452006, -1827.470019576219 460.60861796167427, -1827.456916953241 463.22041243191495, -1827.453167882708 465.7922830943315, -1827.4664022747277 468.33901860668254, -1827.483847189667 470.86006207510894, -1827.4812057373863 473.33914301537476, -1827.4896274959874 475.79863292259535, -1827.5067805340334 478.2137627289414, -1827.5521353626518 480.587493500622, -1827.6017024782875 482.9259205198625, -1827.6375645915152 485.25011227640493, -1827.6983559216255 487.5404828533384, -1827.768495006748 489.8217979889334, -1827.8534363140877 492.0801956554021, -1827.8273242241976 494.2877415343229, -1827.7507596790954 496.4067394188003, -1827.459557173411 498.4683968484606, -1826.8895520078909 500.40985613436163, -1825.6598362068603 502.5267914916609, -1823.9104113483388 504.165263241579, -1821.7884973306147 505.2227124260856, -1819.3789976867308 505.6442573407121, -1817.3692903647297 505.57290695757285, -1815.297601863844 505.42261862443434, -1813.2094200422746 505.1738076699607, -1811.1013280494194 504.7715758325278, -1809.0021121189377 504.3832091173471, -1806.937318575842 504.0068638148298, -1804.8840410014557 503.68893067579495, -1802.8768755844571 503.34420453758554, -1800.8922914031307 503.02683934821647, -1798.4957194611536 502.6044097665408, -1796.14213440751 502.208975343921, -1793.9041200239078 501.8636543540316, -1791.8578507456577 501.54794330183046, -1789.7726435242073 501.20893567743997, -1787.6620374411332 500.91668853927246, -1785.640526958567 500.6447904936965)", route_linestring_);

        std::printf("Route LS has %i pts\n", route_linestring_.size());

        // Read junctions from file
        odr::multipolygon junction_mps;

        std::ifstream ifs_jct(data_path+"/maps/junction_wkt.txt");
        std::string wkt_str_jct( (std::istreambuf_iterator<char>(ifs_jct) ),
                       (std::istreambuf_iterator<char>()) );
        boost::geometry::read_wkt<odr::multipolygon>(wkt_str_jct.c_str(), junction_mps);

        junction_polys_.clear();
        for (odr::polygon poly : junction_mps)
        {
            junction_polys_.push_back(poly);
        }

        std::printf("Added %i junction polygons\n", junction_polys_.size());


        // bg::simplify(route_linestring_, route_linestring_, 1.0);
        std::printf("Route LS now has %i pts\n", route_linestring_.size());

    }
    else
    {
        world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::worldInfoCb, this, std::placeholders::_1));
    }

    drivable_area_grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::drivableAreaGridPubTimerCb, this));
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // Create map management node instance, spin inside multithreaded executor
    auto mgr = std::make_shared<MapManagementNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mgr);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

/**
 * Given a global (possible large) route linestring:
 * 1. Find the point closest to the car.
 * 2. Select the point following it in the linestring.
 * 3. For this point and the following points, append them until the current point's
 * distance to the car is beyond some max constant
 */
void MapManagementNode::updateLocalRouteLinestring()
{
    float MAX_DISTANCE = 50.0; // meters
    float min_distance = 999999.9;

    auto ego_transl = getEgoTf().transform.translation;
    BoostPoint ego_pos(ego_transl.x, ego_transl.y);

    local_route_linestring_.clear();

    int min_idx = 0;
    for (int i = 0; i < route_linestring_.size(); i++)
    {
        auto pt = route_linestring_[i];
        float dist = bg::distance(pt, ego_pos);
        if (dist < min_distance)
        {
            min_distance = dist;
            min_idx = i;
        }
    }

    if (min_idx>0)
    {
        min_idx--;
    }

    for (int i = min_idx; i < route_linestring_.size(); i+= 1)
    {
        if (bg::distance(ego_pos, route_linestring_[i]) > MAX_DISTANCE)
            break;
        bg::append(local_route_linestring_, route_linestring_[i]);
    }
}

/**
 * Once a route is requested and the route_linestring_ variable is defined,
 * this method will regularly publish the global smoothed route.
 * TODO: the map manager should keep track of progress along the route and shorten it
*/
void MapManagementNode::publishSmoothRoute()
{
    // Do not try to deref clock before it's initialized
    if (!this->clock_)
        return;

    auto clock = this->clock_->clock;
    // if we have already created the smoothed route message
    if(smoothed_route_msg_.poses.size() > 0)
    {
        smoothed_route_msg_.header.stamp = clock;
        for (unsigned int i = 0; i < smoothed_route_msg_.poses.size(); i++) {
            smoothed_route_msg_.poses[i].header.stamp = clock;
        }
        route_path_pub_->publish(smoothed_route_msg_);
    }
    // if the route has been defined, but the message hasn't been made
    else if(route_linestring_.size() > 0)
    {
        // Publish smoothed route
        smoothed_route_msg_.poses.resize(route_linestring_.size());
        smoothed_route_msg_.header.frame_id = "map";
        smoothed_route_msg_.header.stamp = clock;
        for (unsigned int i = 0; i < route_linestring_.size(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = clock;
            pose.pose.position.x = route_linestring_[i].get<0>();
            pose.pose.position.y = route_linestring_[i].get<1>();
            smoothed_route_msg_.poses[i] = pose;
        }
        route_path_pub_->publish(smoothed_route_msg_);
    }
}

void MapManagementNode::clickedPointCb(PointStamped::SharedPtr msg)
{
    setRouteFromClickedPt(*msg);
}

std::vector<odr::LaneKey> laneKeysFromPoint(odr::point pt, bgi::rtree<odr::value, bgi::rstar<16, 4>> lane_tree, std::vector<odr::LanePair> lane_polys)
{
    std::vector<odr::value> query_results;

    lane_tree.query(bgi::intersects(pt), std::back_inserter(query_results));

    std::vector<odr::LaneKey> lane_keys;

    // Move through our tree search results, extracting the lane key and
    // adding it to our result list.
    for (auto result : query_results)
    {
        odr::ring ring = lane_polys.at(result.second).second;
        bool point_is_within_shape = bg::within(pt, ring);
        odr::LanePair result_pair = lane_polys[result.second];
        if (point_is_within_shape)
            lane_keys.push_back(result_pair.first.key);
    }

    if (lane_keys.size() > 0)
        return lane_keys;

    // At this point, our search could not find any lanes intersecting the point.
    // The fallback is to find the nearest lane to the point, even if they don't touch.
    lane_tree.query(bgi::nearest(pt, 1), std::back_inserter(query_results));

    odr::LanePair result_pair = lane_polys[query_results.front().second];
    lane_keys.push_back(result_pair.first.key);

    double dist = bg::distance(pt, query_results.front().first);
    std::printf("Lane key does not touch current pos. Distance: %f \n", dist);

    return lane_keys;
}

void MapManagementNode::setPredeterminedRoute()
{
    std::vector<odr::LaneKey> to_park_keys;
    to_park_keys.push_back(odr::LaneKey("14", 313.39848024248613, 2));
    to_park_keys.push_back(odr::LaneKey("14", 48.122505395107886, 1));
    to_park_keys.push_back(odr::LaneKey("14", 0.0, 1));
    to_park_keys.push_back(odr::LaneKey("105", 0.0, 1));
    to_park_keys.push_back(odr::LaneKey("0", 0.0, -3));
    to_park_keys.push_back(odr::LaneKey("0", 53.384839499112189, -3));
    to_park_keys.push_back(odr::LaneKey("0", 61.099814596270349, -4));
    to_park_keys.push_back(odr::LaneKey("0", 84.213823743447264, -3));
    to_park_keys.push_back(odr::LaneKey("220",0.0, -1));
    to_park_keys.push_back(odr::LaneKey("33",0.0, 3));
    to_park_keys.push_back(odr::LaneKey("4",0.0, 3));
    to_park_keys.push_back(odr::LaneKey("52",0.0, 3));
    to_park_keys.push_back(odr::LaneKey("84",0.0, -3));
    to_park_keys.push_back(odr::LaneKey("62",0.0, -3));
    to_park_keys.push_back(odr::LaneKey("42",0.0, -3));
    to_park_keys.push_back(odr::LaneKey("484",0.0, -1));
    to_park_keys.push_back(odr::LaneKey("30",0.0, -3));
    to_park_keys.push_back(odr::LaneKey("68",0.0, -3));

    route_linestrings_.clear();

    for (odr::LaneKey k : to_park_keys)
    {
        std::printf("Appending %s to route linestrings.\n", k.to_string().c_str());
        LineString key_ls = getLaneCenterline(k);
        std::printf("Key LS has %i points\n", key_ls.size());

        // bg::append(route_linestring_, getLaneCenterline(k));
        LineString key_ls_simplified;
        bg::simplify(key_ls, key_ls_simplified, 2.0);

        route_linestrings_.push_back(key_ls_simplified);
        // route_linestrings_.push_back(getLaneCenterline(k));
    }

    // bg::simplify(route_linestring_, route_linestring_, 1.0);

}

void MapManagementNode::setRouteFromClickedPt(const PointStamped clicked_pt)
{

    // For the first two points, get their lanekeys
    // TODO: Extend this to n points
    // auto vehicle_tf = getEgoTf();
    // auto vehicle_pos = vehicle_tf.transform.translation;
    // odr::point from_pt(vehicle_pos.x, vehicle_pos.y);
    // RCLCPP_INFO(get_logger(), "Setting from clicked point.");

    // if (clicked_pt.header.frame_id != "base_link")
    // {
    //     RCLCPP_ERROR(get_logger(), "Clicked point should be in base_link.");
    //     return;
    // }

    // // Transform clicked point to map frame
    // float yaw = acos(vehicle_tf.transform.rotation.w) * 2;
    // float x_ = clicked_pt.point.x;
    // float y_ = clicked_pt.point.y;
    // float clicked_x = (x_ * cos(yaw) - y_ * sin(yaw)) + vehicle_pos.x;
    // float clicked_y = (y_ * cos(yaw) + x_ * sin(yaw)) + vehicle_pos.y;
    // odr::point to_pt(clicked_x, clicked_y);

    // auto from_keys = laneKeysFromPoint(from_pt, map_wide_tree_, lane_polys_);
    // auto to_keys = laneKeysFromPoint(to_pt, map_wide_tree_, lane_polys_);
    // if (from_keys.size() > 1)
    // {
    //     RCLCPP_ERROR(get_logger(), "Route start falls within a junction.");
    //     return;
    // }
    // if (to_keys.size() > 1)
    // {
    //     RCLCPP_ERROR(get_logger(), "Route end falls within a junction.");
    //     return;
    // }
    // if (from_keys.size() < 1)
    // {
    //     RCLCPP_ERROR(get_logger(), "Route start does not fall within a lane.");
    //     return;
    // }
    // if (to_keys.size() < 1)
    // {
    //     RCLCPP_ERROR(get_logger(), "Route end does not fall within a lane.");
    // }
    // // odr::LaneKey from_key = from_keys[0];
    // // odr::LaneKey to_key = to_keys[0];
    // odr::LaneKey to_key("30", 0.0, -3);
    // odr::LaneKey from_key("0", 0.0, -2);

    // if (from_key.road_id == to_key.road_id && from_key.lanesection_s0 == to_key.lanesection_s0)
    // {
    //     RCLCPP_ERROR(get_logger(), "Cannot create a route within the same lanesection.");
    //     return;
    // }

    // SmartDigraph::Node start = g->nodeFromId(routing_nodes_->at(from_key));
    // SmartDigraph::Node end = g->nodeFromId(routing_nodes_->at(to_key));

    // SptSolver spt(*g, *costMap);

    // // std::printf("Running solver from %s to %s\n", from_key.to_string().c_str(), to_key.to_string().c_str());

    // spt.run(start, end);

    // std::vector<lemon::SmartDigraph::Node> node_route;
    // int iters = 0;
    // for (lemon::SmartDigraph::Node v = end; v != start; v = spt.predNode(v))
    // {
    //     if (v != lemon::INVALID && spt.reached(v)) // special LEMON node constant
    //     {
    //         node_route.push_back(v);
    //     }
    //     if (iters > 100)
    //     {
    //         break;
    //     }
    //     iters++;
    // }
    // node_route.push_back(start);

    // std::printf("Path has %i nodes\n", node_route.size());

    

    // for (auto p = node_route.rbegin(); p != node_route.rend(); ++p)
    // {
    //     std::printf("%s\n", (*nodeMap)[*p].to_string().c_str());
    //     bg::append(route_linestring_, getLaneCenterline((*nodeMap)[*p]));
    // }

    // if (node_route.size() < 2)
    // {
    //     RCLCPP_INFO_STREAM(get_logger(), "Path between " << from_key.to_string().c_str() << " and " << to_key.to_string().c_str() << " not found.");
    // }
    // else
    // {
    //     RCLCPP_INFO_STREAM(get_logger(), "Path between " << from_key.to_string().c_str() << " and " << to_key.to_string().c_str() << " has " << node_route.size() << " lanes and " << route_linestring_.size() << " points.");
    // }

}

void MapManagementNode::setRoute(const std::shared_ptr<navigator_msgs::srv::SetRoute::Request> request, std::shared_ptr<navigator_msgs::srv::SetRoute::Response> response)
{
    RCLCPP_INFO(get_logger(), "Received request to set route!");

    if (request->route_nodes.size() < 2)
    {
        RCLCPP_ERROR(get_logger(), "Service call requires at least two points.");
        response->message = "Service call requires at least two points.";
        response->success = false;
        return;
    }
    else if (request->route_nodes.size() > 2)
    {
        RCLCPP_WARN(get_logger(), "Routing only supported for two points at the moment. Remaining points will be ignored.");
    }

    // Make sure map tree isn't empty before we try setting a route
    if (map_wide_tree_.empty()) {
        RCLCPP_ERROR(get_logger(), "Cannot set a route before map_wide_tree_ has been initialized.");
        std::string message = "Cannot set a route before map_wide_tree_ has been initialized.";
        response->message = message;
        response->success = false;
        return;
    }

    // For the first two points, get their lanekeys
    // TODO: Extend this to n points
    odr::point from_pt(request->route_nodes[0].x, request->route_nodes[0].y);
    odr::point to_pt(request->route_nodes[1].x, request->route_nodes[1].y);
    auto from_keys = laneKeysFromPoint(from_pt, map_wide_tree_, lane_polys_);
    auto to_keys = laneKeysFromPoint(to_pt, map_wide_tree_, lane_polys_);
    if (from_keys.size() > 1)
    {
        RCLCPP_ERROR(get_logger(), "Route start falls within a junction.");
        std::ostringstream message_stream;
        message_stream << "Route start falls within a junction: " << from_keys.front().to_string().c_str();
        std::string message = message_stream.str();
        response->message = message;
        response->success = false;
        return;
    }
    if (to_keys.size() > 1)
    {
        RCLCPP_ERROR(get_logger(), "Route end falls within a junction.");
        response->message = "Route end falls within a junction.";
        response->success = false;
        return;
    }
    if (from_keys.size() < 1)
    {
        RCLCPP_ERROR(get_logger(), "Route start does not fall within a lane.");
        response->message = "Route start does not fall within a lane.";
        response->success = false;
        return;
    }
    if (to_keys.size() < 1)
    {
        RCLCPP_ERROR(get_logger(), "Route end does not fall within a lane.");
        response->message = "Route end does not fall within a lane.";
        response->success = false;
        return;
    }
    odr::LaneKey from_key = from_keys[0];
    odr::LaneKey to_key = to_keys[0];
    SmartDigraph::Node start = g->nodeFromId(routing_nodes_->at(from_key));
    SmartDigraph::Node end = g->nodeFromId(routing_nodes_->at(to_key));

    SptSolver spt(*g, *costMap);

    // std::printf("Running solver from %s to %s\n", from_key.to_string().c_str(), to_key.to_string().c_str());

    spt.run(start, end);

    std::vector<lemon::SmartDigraph::Node> node_route;
    int iters = 0;
    for (lemon::SmartDigraph::Node v = end; v != start; v = spt.predNode(v))
    {
        if (v != lemon::INVALID && spt.reached(v)) // special LEMON node constant
        {
            node_route.push_back(v);
        }
        if (iters > 100)
        {
            break;
        }
        iters++;
    }
    node_route.push_back(start);

    std::printf("Path has %i nodes\n", node_route.size());

    route_linestring_.clear();

    for (auto p = node_route.rbegin(); p != node_route.rend(); ++p)
    {
        std::printf("%s\n", (*nodeMap)[*p].to_string().c_str());
        bg::append(route_linestring_, getLaneCenterline((*nodeMap)[*p]));
    }

    std::printf("route_linestring_ has length %i\n", route_linestring_.size());

    if (node_route.size() < 2)
    {
        std::ostringstream message_stream;
        message_stream << "Path between " << from_key.to_string().c_str() << " and " << to_key.to_string().c_str() << " not found.";
        std::string message = message_stream.str();
        response->message = message;
        response->success = false;
    }
    else
    {
        std::ostringstream message_stream;
        message_stream << "Path between " << from_key.to_string().c_str() << " and " << to_key.to_string().c_str() << " has " << node_route.size() << " lanes and " << route_linestring_.size() << " points.";
        std::string message = message_stream.str();
        response->message = message;
        response->success = true;
    }
}

/**
 * @brief Returns an OccupancyGrid for lanes of type 'driving'
 *
 * General steps:
 * 1. Query the map-wide R-tree to find all lanes within range (https://www.boost.org/doc/libs/1_72_0/libs/geometry/doc/html/geometry/spatial_indexes/introduction.html)
 * 2. Create a second, local R-tree and insert all nearby lanes, which were found from (1)
 * 3. For each row j and column i, query the local R-tree to see if that cell (i,j) is within a lane's bounding box
 *  a.If yes, find out if it is within the actual lane, not just its bounding box. R-trees only calculate for bounding boxes.
 *      i. If yes again, the cell is truly occupied. Append '100' ("occupied") to OccupancyGrid. Otherwise '0'.
 * 4. Set OccupancyGrid metadata and return.
 *
 * @param center Center of the grid
 * @param top_dist Distance from car to top edge
 * @param bottom_dist Distance from car to bottom edge
 * @param side_dist Distance from car to left and right edges
 * @param res Side length of grid cells (meters)
 * @return OccupancyGrid
 */
void MapManagementNode::publishGrids(int top_dist, int bottom_dist, int side_dist, float res)
{
    if (!this->clock_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Clock not yet initialized. Drivable area grid is unavailable.");
        return;
    }

    if (this->map_ == nullptr)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Map not yet loaded. Drivable area grid is unavailable.");
        return;
    }

    // printf("Publish grids... ");

    // Used to calculate function runtime
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    OccupancyGrid drivable_area_grid;
    OccupancyGrid junction_grid;
    OccupancyGrid route_dist_grid;
    MapMetaData grid_info;
    std::vector<int8_t> drivable_grid_data;
    std::vector<int8_t> junction_grid_data;
    std::vector<int8_t> route_dist_grid_data;

    float y_min = side_dist * -1;
    float y_max = side_dist;
    float x_min = bottom_dist * -1;
    float x_max = top_dist;

    // std::printf("[%f, %f], [%f, %f], %f\n", x_min, x_max, y_min, y_max, res);

    if (this->map_wide_tree_.size() == 0)
        this->map_wide_tree_ = this->map_->generate_mesh_tree();

    // Get the search region
    TransformStamped vehicle_tf = getEgoTf();
    auto vehicle_pos = vehicle_tf.transform.translation;
    double range_plus = top_dist * 1.4; // This is a little leeway to account for map->base_link rotation
    odr::point bounding_box_min = odr::point(vehicle_pos.x - range_plus, vehicle_pos.y - range_plus);
    odr::point bounding_box_max = odr::point(vehicle_pos.x + range_plus, vehicle_pos.y + range_plus);
    odr::box search_region(bounding_box_min, bounding_box_max);

    // Find all lanes within the search region
    std::vector<odr::value> lane_shapes_in_range;
    map_wide_tree_.query(bgi::intersects(search_region), std::back_inserter(lane_shapes_in_range));


    bool has_nearby_lanes = lane_shapes_in_range.size() > 0;
    // Warn if there are no nearby lanes.
    // This may simply because we are testing in an area that is not on the campus.xodr map.
    if (!has_nearby_lanes)
    {
        RCLCPP_WARN(get_logger(), "There are no lane shapes nearby.");
    }

    int idx = 0;

    // This was commented out...
    bgi::rtree<odr::value, bgi::rstar<16, 4>> local_tree;
    std::unordered_map< unsigned int, odr::polygon> box_to_poly_map;
    for (unsigned i = 0; i < lane_shapes_in_range.size(); ++i){
        std::deque<odr::polygon> output;
        odr::polygon a;
        odr::polygon b;

        bg::assign(a, lane_shapes_in_range.at(i).first);
        bg::assign(b, search_region);
        bg::intersection(a,b, output);
        int count = 0;
        if(output.size()>1){
            local_tree.insert(lane_shapes_in_range.at(i));
            odr::polygon poly;
            bg::convert(lane_shapes_in_range.at(i).first, poly);
            box_to_poly_map[lane_shapes_in_range.at(i).second] = poly;
            
        } else{
            local_tree.insert(lane_shapes_in_range.at(i));
            box_to_poly_map[lane_shapes_in_range.at(i).second] = output[0];
        }         
    }
    // down to here

    int area = 0;
    int height = 0;

    BoostPoint goal_pt;
    bool goal_is_set = false;
    auto q = vehicle_tf.transform.rotation;
    float h;

    if (q.z < 0)
        h = abs(2 * acos(q.w) - 2 * M_PI);
    else
        h = 2 * acos(q.w);

    if (h > M_PI)
        h -= 2 * M_PI;

    std::vector<odr::polygon> nearby_junctions;

    for (odr::polygon junction_poly : junction_polys_)
    {
        odr::point vehicle_pos_pt(vehicle_pos.x, vehicle_pos.y);
        float dist = bg::distance(vehicle_pos_pt, junction_poly);

        if (dist < 40.0)
            nearby_junctions.push_back(junction_poly);
    }

    for (float j = y_min; j <= y_max; j += res)
    {
        for (float i = x_min; i <= x_max; i += res)
        {

            i = static_cast<float>(i);
            j = static_cast<float>(j);
            bool cell_is_drivable = false;
            bool cell_is_in_junction = false;

            // Transform this query into the map frame
            // First rotate, then translate. 2D rotation eq:
            // x' = xcos(h) - ysin(h)
            // y' = ycos(h) + xsin(h)

            // if (h < 0)
            //     h += 2 * M_PI;
            float i_in_map = i * cos(h) - j * sin(h) + vehicle_pos.x;
            float j_in_map = j * cos(h) + i * sin(h) + vehicle_pos.y;

            odr::point p(i_in_map, j_in_map);
            // This block was commented out: 
            std::vector<odr::value> local_tree_query_results;
            local_tree.query(bgi::contains(p), std::back_inserter(local_tree_query_results));

            if (local_tree_query_results.size() > 0)
            {
                for (auto pair : local_tree_query_results)
                {
                    // auto pair = local_tree_query_results.front();
                    odr::ring ring;
                    bg::convert(this->lane_polys_.at(pair.second).second, ring);
                    odr::Lane lane = this->lane_polys_.at(pair.second).first;
                    // odr::Road road = this->map_->id_to_road.at(lane.key.road_id);
                    bool point_is_within_shape = bg::within(p, ring);
                    if (point_is_within_shape)
                    {

                        cell_is_in_junction = this->road_in_junction_map_[lane.key];
                        if (lane.type == "driving")
                        {
                            cell_is_drivable = true;
                            break;
                        }
                    }
                }
            }
            // Down to here.
            for (auto poly : nearby_junctions)
            {
                if (bg::within(p, poly))
                {
                    cell_is_in_junction = true;
                }
            }

            // If no lanes are are nearby, then the vehicle is assumed to be testing on a route
            // area which is not on the campus.xodr map. 
            // In this case, make the entire drivable grid 0 cost. 
            if (!has_nearby_lanes) {
                drivable_grid_data.push_back(0);
            } else {
                drivable_grid_data.push_back(cell_is_drivable ? 0 : 100);
            }

            junction_grid_data.push_back(cell_is_in_junction ? 100 : 0);

            /*  Taking route distance out of the pipeline
            // // Get closest route point
            if (i < -10)
                route_dist_grid_data.push_back(100);
            else if (local_route_linestring_.size() > 0)
            {
                int dist = static_cast<int>((1-(1/(1+bg::distance(local_route_linestring_, p))))*100);

                if (dist < 1.0 && !goal_is_set && abs(i) + abs(j) > 30)
                {
                    goal_is_set = true;
                    goal_pt = BoostPoint(i, j);
                }

                const int SCALE = 24;

                // Distances > 10 are set to 100
                if (dist > 80)
                    route_dist_grid_data.push_back(100);
                else 
                {
                    route_dist_grid_data.push_back(dist);
                    // dist *= 10;
                }
                
                //RCLCPP_INFO(get_logger(), "dist value %i", dist);

            }
            else
            {
                route_dist_grid_data.push_back(100);
            }*/

            area += 1;
        }
        height += 1;
    }

    auto clock = this->clock_->clock;
    drivable_area_grid.data = drivable_grid_data;
    drivable_area_grid.header.frame_id = "base_link";
    drivable_area_grid.header.stamp = clock;

    junction_grid.data = junction_grid_data;
    junction_grid.header.frame_id = "base_link";
    junction_grid.header.stamp = clock;

    //route_dist_grid.data = route_dist_grid_data;
    //route_dist_grid.header.frame_id = "base_link";
    //route_dist_grid.header.stamp = clock;

    grid_info.width = area / height;
    grid_info.height = height;
    grid_info.map_load_time = clock;
    grid_info.resolution = res;
    grid_info.origin.position.x = x_min;
    grid_info.origin.position.y = y_min;

    // Set the grids' metadata
    drivable_area_grid.info = grid_info;
    junction_grid.info = grid_info;
    //route_dist_grid.info = grid_info;

    // Output function runtime
    drivable_grid_pub_->publish(drivable_area_grid);
    junction_grid_pub_->publish(junction_grid);
    //route_dist_grid_pub_->publish(route_dist_grid); // Route distance grid

    PoseStamped goal_pose;
    goal_pose.pose.position.x = goal_pt.get<0>();
    goal_pose.pose.position.y = goal_pt.get<1>();
    goal_pose.header.frame_id = "base_link";
    goal_pose.header.stamp = clock_->clock;
    goal_pose_pub_->publish(goal_pose);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "publishGrids(): " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
}

/**
 * @brief Return the (x,y) point at the start of a lane's outer border.
 *
 * @param key
 * @return odr::point
 */
odr::point MapManagementNode::getLaneStart(odr::LaneKey key)
{
    // Get the current lane's start point (x,y)
    odr::Road current_road = map_->id_to_road.at(key.road_id);
    odr::LaneSection current_lsec = current_road.s_to_lanesection.at(key.lanesection_s0);
    odr::Lane current_lane = current_lsec.id_to_lane.at(key.lane_id);

    double s_start;
    if (key.lane_id < 0)
    {
        s_start = key.lanesection_s0;
    }
    else
    {
        s_start = current_road.get_lanesection_end(key.lanesection_s0);
    }
    double t = current_lane.outer_border.get(s_start);
    odr::Vec3D start_vec = current_road.get_surface_pt(s_start, t);
    odr::point start_xy(start_vec[0], start_vec[1]);

    return start_xy;
}

/**
 * @brief Given a key, find true successors: immediately connected lanes
 * ahead of the given one with respect to traffic flow.
 *
 * Assuming right-hand driving rules, we treat the "start" of a lane as having:
 * - Either s = lanesection_s0 and ID < 0, OR
 * - s = lanesection_s_max and ID > 0.
 *
 * Recall that lanesections are identified by their start and end s values.
 *
 * @param key
 * @return std::vector<odr::LaneKey>
 */
std::vector<odr::LaneKey> MapManagementNode::getTrueSuccessors(odr::LaneKey key)
{
    std::vector<odr::LaneKey> results;

    // We assume that each lane's listed successors and predecessors could be
    // actual successors, and this function verifies or refutes each one,
    // returning the verified ones.

    if (this->faulty_routing_graph.edges.size() < 1)
    {
        RCLCPP_ERROR_ONCE(get_logger(), "Routing graph was empty!");
    }

    // Get the current lane's endpoint (x,y)
    odr::Road current_road = map_->id_to_road.at(key.road_id);
    odr::LaneSection current_lsec = current_road.s_to_lanesection.at(key.lanesection_s0);
    odr::Lane current_lane = current_lsec.id_to_lane.at(key.lane_id);

    double s_end;
    if (key.lane_id > 0)
    {
        s_end = key.lanesection_s0;
    }
    else
    {
        s_end = current_road.get_lanesection_end(key.lanesection_s0);
    }
    double t = current_lane.outer_border.get(s_end);
    odr::Vec3D end_vec = current_road.get_surface_pt(s_end, t);
    odr::point end_xy(end_vec[0], end_vec[1]);

    auto predecessors = faulty_routing_graph.get_lane_predecessors(key);

    // For each predecessor, get their lane's START (x,y)
    for (odr::LaneKey predecessor : predecessors)
    {
        odr::point start_xy = getLaneStart(predecessor);
        float dist = bg::distance(end_xy, start_xy);
        // std::printf("Dist: %f\n", dist);

        if (dist < 0.05)
            results.push_back(predecessor);
    }

    auto successors = faulty_routing_graph.get_lane_successors(key);

    // For each predecessor, get their lane's START (x,y)
    for (odr::LaneKey successor : successors)
    {
        odr::point start_xy = getLaneStart(successor);
        float dist = bg::distance(end_xy, start_xy);
        // std::printf("Dist: %f\n", dist);

        if (dist < 0.05)
            results.push_back(successor);
    }

    // std::printf("Found %i successors\n", results.size());

    return results;
}

/**
 * @brief Build a proper routing graph of the map
 * using LEMON's SmartDigraph.
 *
 */
void MapManagementNode::buildTrueRoutingGraph()
{
    if (this->lane_polys_.size() < 1 || this->map_ == nullptr)
    {
        RCLCPP_ERROR(get_logger(), "Map not yet loaded. Routing graph could not be built.");
        return;
    }

    if (faulty_routing_graph.edges.size() < 1)
    {
        faulty_routing_graph = this->map_->get_routing_graph();
        RCLCPP_INFO(get_logger(), "Built rough routing graph.");
    }
    g = new lemon::SmartDigraph();
    nodeMap = new lemon::SmartDigraph::NodeMap<odr::LaneKey>(*g);
    std::printf("469\n");

    costMap = new lemon::SmartDigraph::ArcMap<double>(*g);
    routing_nodes_ = new std::map<odr::LaneKey, int>();
    std::vector<Arc> arcs;
    int node_idx = 0;
    std::printf("476\n");

    for (odr::LanePair pair : lane_polys_)
    {
        odr::LaneKey from_key = pair.first.key;
        odr::Road from_road = map_->id_to_road.at(from_key.road_id);
        double from_length = from_road.get_lanesection_end(from_key.lanesection_s0) - from_key.lanesection_s0;
        routing_nodes_->insert(std::pair<odr::LaneKey, int>(from_key, node_idx));
        node_idx++;

        auto successors = getTrueSuccessors(from_key);

        for (odr::LaneKey to_key : successors)
            arcs.push_back(Arc{from_key, to_key, from_length});
    }

    std::printf("487\n");

    // populate graph
    // nodes first
    lemon::SmartDigraph::Node currentNode;
    for (auto nodesIter = routing_nodes_->begin(); nodesIter != routing_nodes_->end(); ++nodesIter)
    {
        odr::LaneKey key = nodesIter->first;
        currentNode = g->addNode();
        (*nodeMap)[currentNode] = key;
    }
    // then the arcs with the costs through the cost map
    lemon::SmartDigraph::Arc currentArc;
    for (auto arcsIter = arcs.begin(); arcsIter != arcs.end(); ++arcsIter)
    {
        int sourceIndex = routing_nodes_->at(arcsIter->sourceKey);
        int targetIndex = routing_nodes_->at(arcsIter->targetKey);

        SmartDigraph::Node sourceNode = g->nodeFromId(sourceIndex);
        SmartDigraph::Node targetNode = g->nodeFromId(targetIndex);

        currentArc = g->addArc(sourceNode, targetNode);
        (*costMap)[currentArc] = arcsIter->sourceLength;
    }

    RCLCPP_INFO(get_logger(), "Built complete routing graph.");
}

void MapManagementNode::recursiveSearch(std::vector<odr::LaneKey> predecessors, odr::LaneKey target)
{
    odr::LaneKey current_lane = predecessors.back();
    if (current_lane.to_string() == target.to_string())
    {
        RCLCPP_INFO(get_logger(), "FOUND TARGET");
        return;
    }
    else
    {
        std::printf("%s != %s\n", current_lane.to_string().c_str(), target.to_string().c_str());
    }

    auto successors = getTrueSuccessors(current_lane);

    // for (auto successor : successors)
    // {
    //     const odr::LaneKey const_succ = odr::LaneKey(successor);
    //     if (std::find(predecessors.begin(), predecessors.end(), const_succ) != predecessors.end())
    //         return;
    //     std::vector<odr::LaneKey> new_predecessors(predecessors);
    //     new_predecessors.push_back(successor);
    //     recursiveSearch(new_predecessors, target);
    // }
}

std::vector<odr::LaneKey> MapManagementNode::calculateRoute(odr::LaneKey start, odr::LaneKey end)
{
    if (this->map_wide_tree_.size() == 0)
    {
        RCLCPP_INFO(get_logger(), "map_wide_tree_ was empty. Generating.");
        this->map_wide_tree_ = this->map_->generate_mesh_tree();
    }
    std::vector<odr::LaneKey> route;
    recursiveSearch(getTrueSuccessors(start), end);
    return route;
}

/**
 * @brief Caches the latest clock from CARLA. Usand published for message timestamps.
 *
 * @param msg
 */
void MapManagementNode::clockCb(Clock::SharedPtr msg)
{
    this->clock_ = msg;
}

/**
 * @brief Gets latest map->base_link transform and returns it
 *
 * @return TransformStamped
 */
TransformStamped MapManagementNode::getEgoTf()
{
    TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(
            "map", "base_link",
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Could not get base_link->map tf: %s. This will republish every 5 seconds.", ex.what());
        return TransformStamped();
    }

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    //                          "Returning empty transform.");
    return t;
}

/**
 * @brief Gets the drivable area OccupancyGrid and publishes it.
 *
 */
void MapManagementNode::drivableAreaGridPubTimerCb()
{
    // std::printf("Publishing grids\n");
    publishGrids(40, 20, 30, 0.4);
}

// CPP code for printing shortest path between
// two vertices of unweighted graph
#include <bits/stdc++.h>
using namespace std;

// utility function to form edge between two vertices
// source and dest
void add_edge(vector<int> adj[], int src, int dest)
{
    adj[src].push_back(dest);
    adj[dest].push_back(src);
}

LineString getSmoothSection(LineString full_route, BoostPoint ego, int &start_idx, int &end_idx)
{

    // Get closest waypoint in full_route
    float closest_dist = 999.9;
    int closest_idx = -1;

    for (int i = 0; i < full_route.size(); i++)
    {
        auto wp = full_route[i];
        float dist_to_ego = bg::distance(ego, wp);
        if (dist_to_ego < closest_dist)
        {
            closest_dist = dist_to_ego;
            closest_idx = i;
        }
    }

    end_idx = closest_idx < 1 ? closest_idx : closest_idx - 1;

    LineString rough_section;
    start_idx = closest_idx + 1;
    float edge_distance = closest_dist;

    // Find the "edge" waypoint
    // That's the divider between the smooth and rough route sections
    // It should be > 40m from the ego
    while (edge_distance < 40.0)
    {
        auto wp = full_route[start_idx];
        edge_distance = bg::distance(ego, wp);
        start_idx++;
    }

    return rough_section;
}

LineString MapManagementNode::getLaneCenterline(odr::LaneKey key)
{
    LineString centerline;

    odr::Road road = map_->id_to_road.at(key.road_id);
    odr::LaneSection lsec = road.s_to_lanesection.at(key.lanesection_s0);
    odr::Lane lane = lsec.id_to_lane.at(key.lane_id);

    odr::Line3D outer_border = road.get_lane_border_line(lane, 1.0, true);
    odr::Line3D inner_border = road.get_lane_border_line(lane, 1.0, false);

    for (int i = 0; i < outer_border.size(); i++)
    {
        odr::Vec3D outer_pt = outer_border[i];
        odr::Vec3D inner_pt = inner_border[i];
        BoostPoint center_pt((outer_pt[0] + inner_pt[0]) / 2, (outer_pt[1] + inner_pt[1]) / 2);
        bg::append(centerline, center_pt);
    }

    return centerline;
}

LineString getReorientedRoute(std::vector<LineString> centerlines)
{
    LineString result;

    auto iter = centerlines.begin();
    while (iter != centerlines.end() - 1)
    {
        // Get end of this segment and the beginning of the next one
        // Check the distance between them.
        // If distance is small, the orientation is correct. Continue.
        // If distance is large, reverse order of next segment

        const float MAX_ENDPOINT_GAP = 2.0; // meters

        auto current_end = iter->back();
        auto next_begin = (iter + 1)->front();
        float dist = bg::distance(current_end, next_begin);

        // std::printf("Distance was %f. ", dist);

        if (dist > MAX_ENDPOINT_GAP)
        {
            bg::reverse(*(iter + 1));
        }

        next_begin = (iter + 1)->front();
        // dist = bg::distance(current_end, next_begin);
        // std::printf("Distance is now %f.\n", dist);

        bg::append(result, *iter);

        iter++;
    }
    bg::append(result, centerlines.back());

    return result;
}

void MapManagementNode::updateRouteWaypoints(Path::SharedPtr msg)
{
    if (rough_route_tree_.size() > 0 && rough_route_tree_.size() == msg->poses.size())
    {
        // It looks like we've already processed a route and the new route as the same size.
        // If the new route's size and the current tree's size are the same,
        // we can assume that the incoming route is not new.
        return;
    }

    // Let's build an RTree to efficiently represent the new waypoints.

    rough_route_tree_.clear();
    rough_route_.clear();

    for (unsigned i = 0; i < msg->poses.size(); ++i)
    {
        PoseStamped wp_pose = msg->poses[i];

        BoostPoint wp(wp_pose.pose.position.x, wp_pose.pose.position.y);
        rough_route_tree_.insert(std::make_pair(bg::return_envelope<odr::box>(wp), i));
        bg::append(rough_route_, wp);
    }

    RCLCPP_INFO(get_logger(), "%i waypoints added to tree", rough_route_tree_.size());
}

RoiIndices getWaypointsInROI(LineString waypoints, bgi::rtree<odr::value, bgi::rstar<16, 4>> tree, BoostPoint ego_pos)
{
    std::vector<odr::value> returned_values;
    // Query our tree. "1" means get the single nearest waypoint
    tree.query(bgi::nearest(ego_pos, 1), std::back_inserter(returned_values));
    int nearest_idx = returned_values.front().second;

    // Go backward in rough route until either
    // a) We hit the route's start or
    // b) we're >40m from ego
    // Get idx of last waypoint visited
    auto iter = waypoints.begin() + nearest_idx;
    int start_idx = nearest_idx;
    while (iter != waypoints.begin())
    {
        start_idx--;
        iter--;
        if (bg::distance(ego_pos, *iter) > 40.0)
            break;
    }

    // Go forward in rough route until either
    // a) We hit the route's end or
    // b) we're >40m from ego
    // Get idx of last waypoint visited
    iter = waypoints.begin() + nearest_idx;
    int end_idx = nearest_idx;
    while (iter != waypoints.end())
    {
        end_idx++;
        iter++;
        if (bg::distance(ego_pos, *iter) > 40.0)
            break;
    }

    // RCLCPP_INFO(get_logger(), "(%i, %i, %i)", start_idx, nearest_idx, end_idx);

    RoiIndices result;
    result.center = nearest_idx;
    result.start = start_idx;
    result.end = end_idx;

    return result;
}

std::vector<odr::LaneKey> getLaneKeysFromIndices(RoiIndices indices, LineString waypoints, bgi::rtree<odr::value, bgi::rstar<16, 4>> lane_tree, std::vector<odr::LanePair> lane_polys)
{
    LineString waypoints_within_roi;
    // bg::simplify(waypoints, simplified_wps, 5.0);

    for (int i = indices.start; i <= indices.end; i++)
    {
        bg::append(waypoints_within_roi, waypoints[i]);
    }

    // Simplification downsamples the linestring, speeding us up a bit
    LineString simplified_wpts;
    bg::simplify(waypoints_within_roi, simplified_wpts, 5.0);

    std::vector<odr::value> query_results;

    // For each wp, find it in the lane_tree
    for (auto pt : simplified_wpts)
    {
        lane_tree.query(bgi::intersects(pt), std::back_inserter(query_results));
    }

    std::vector<odr::LaneKey> lane_keys;

    // Move through our tree search results, extracting the lane key and
    // adding it to our result list.
    for (auto result : query_results)
    {
        odr::LanePair result_pair = lane_polys[result.second];
        lane_keys.push_back(result_pair.first.key);
        // std::printf("%s\n", result_pair.first.key.to_string().c_str());
    }

    return lane_keys;
}

/**
 * 
 * NOTE: CURRENTLY UNUSED
 * 
 * @brief Given a destination point (x,y), set our route linestring
 * to be the continuous centerlines of the lanes connecting
 * the current location to the destination. Should be fast.
 */
void MapManagementNode::updateRouteGivenDestination(odr::point destination)
{
    RCLCPP_INFO(get_logger(), "Updating route from destination (%f, %f)", destination.get<0>(), destination.get<1>());
    auto current_tf = getEgoTf();
    odr::point current_position(current_tf.transform.translation.x, current_tf.transform.translation.y);

    RCLCPP_INFO(get_logger(), "Current pos: (%f, %f)", current_position.get<0>(), current_position.get<1>());

    if (this->map_wide_tree_.size() == 0)
    {
        RCLCPP_INFO(get_logger(), "map_wide_tree_ was empty. Generating.");
        this->map_wide_tree_ = this->map_->generate_mesh_tree();
    }

    odr::LaneKey from = laneKeysFromPoint(current_position, map_wide_tree_, lane_polys_).front();
    odr::LaneKey to = laneKeysFromPoint(destination, map_wide_tree_, lane_polys_).front();
    RCLCPP_INFO(get_logger(), "From %s to %s", from.to_string().c_str(), to.to_string().c_str());

    std::vector<odr::LaneKey> complete_segment; // keys, but with gaps in lanes filled.

    // THIS FUNCTION LOOKS HALF COMPLETE.... TRYING TO FILL IT IN....
    // auto from_keys = laneKeysFromPoint(from_pt, map_wide_tree_, lane_polys_);
    // auto to_keys = laneKeysFromPoint(to_pt, map_wide_tree_, lane_polys_);
    //complete_segment = this->calculateRoute(from_keys,to_keys)
    // loop 
    //route_linestring_ = getLaneCenterline(key)
}

std::map<odr::LaneKey, bool> MapManagementNode::getJunctionMap(std::vector<odr::LanePair> lane_polys)
{
    std::map<odr::LaneKey, bool> map;
    for (auto pair : lane_polys)
    {
        odr::Lane lane = pair.first;
        odr::Road road = this->map_->id_to_road.at(lane.key.road_id);
        if (road.junction != "-1" && lane.type == "driving")
            map[lane.key] = true;
        else
            map[lane.key] = false;
    }

    return map;
}

/**
 * @brief When map data is received from a CarlaWorldInfo message, load and process it
 *
 * @param msg The incoming CarlaWorldInfo message
 */
void MapManagementNode::worldInfoCb(CarlaWorldInfo::SharedPtr msg)
{
    if (this->map_ != nullptr)
        return; // Our map is already loaded. No need to continue.
    if (msg->opendrive == "")
    {
        RCLCPP_INFO(this->get_logger(), "Received empty map string from world_info topic. Waiting for real data.");
        return;
    }

    // Ask libopendrive to parse the map string
    this->map_ = new odr::OpenDriveMap(msg->opendrive, true);
    // Get lane polygons as pairs (Lane object, ring polygon)
    this->lane_polys_ = map_->get_lane_polygons(1.0, false);

    this->road_in_junction_map_ = this->getJunctionMap(this->lane_polys_);

    RCLCPP_INFO(this->get_logger(), "Loaded %s", msg->map_name.c_str());

    buildTrueRoutingGraph();
}