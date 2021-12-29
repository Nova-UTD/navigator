#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/*

PSEUDOCODE
On init, we want to:
1. Read filename from param
2. Read pub period from param
3. Load map using lanelet2_io into memory
4. Convert each lanelet into an array of triangles
5. Save all triangles as a Triangle List https://wiki.ros.org/rviz/DisplayTypes/Marker#Triangle_List_.28TRIANGLE_LIST.3D11.29_.5B1.1.2B-.5D

Every n seconds (pub period from param):
1. Publish the triangle list to /viz/lanelets

*/

class LaneletLoader : public rclcpp::Node
{
  public:
    LaneletLoader()
    : Node("lanelet_loader_node")
    {
        this->declare_parameter<std::string>("osm_path", "/home/main/voltron/assets/grand_loop.osm");
        load_from_osm();
    }

  private:
    void load_from_osm() {
        using namespace lanelet;
        this->get_parameter("osm_path", osm_path_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Reading map from "<<osm_path_);
        projection::UtmProjector projector(Origin({32.9, -96.7}));  // we will go into details later
        LaneletMapPtr map = load(osm_path_, projector);
    }

    std::string osm_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneletLoader>());
  rclcpp::shutdown();
  return 0;
}