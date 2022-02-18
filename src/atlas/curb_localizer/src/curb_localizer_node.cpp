#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/geometry/Polygon.h>
#include "boost/polygon/voronoi.hpp"
#include "boost/polygon/point_data.hpp"
#include "boost/polygon/segment_data.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using sensor_msgs::msg::PointCloud2;

/*

PSEUDOCODE
On init:
1. Load lanelet map binary and store as pointer
2. Add tf listener, buffer

Every 100ms:
1. Get translation (inverse transfrom) from map to base_link.
	- This is the robot's position on the map.
2. Get nearest lanelet
3. Get nearest point on lanelet centerline
4. Find distance from robot_pos to centerline_point
5. Add distance to total_deviation
6. Increment deviation_count

On shutdown:
1. Print (total_deviation/deviation_count)
	- This is the average deviation from the centerline
*/

class CurbLocalizerNode : public rclcpp::Node
{
	public:
		CurbLocalizerNode()
		: Node("curb_localizer_node")
		{
				this->declare_parameter<std::string>("osm_path", "/home/main/voltron/assets/grand_loop.osm");
				_map = load_from_osm();
				timer_ = this->create_wall_timer(100ms, std::bind(&CurbLocalizerNode::checkDeviation, this));
                curb_detection_sub = this->create_subscription<PointCloud2>("/lidar_front/curb_points")

				// TF2
				tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
				transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

				RCLCPP_INFO_STREAM(this->get_logger(), "Map has "<<_map->laneletLayer.size()<<" lanelets");
		}

	private:

        void getNearestLanelet() {

        }

		lanelet::LaneletMapPtr load_from_osm() {
				using namespace lanelet;
				this->get_parameter("osm_path", osm_path_);
				RCLCPP_INFO_STREAM(this->get_logger(), "Reading map from "<<osm_path_);
				projection::UtmProjector projector(Origin({32.9903275000000, -96.7522478333333}));
				return load(osm_path_, projector);
		}

		void checkDeviation() {
			/*
			1. Get translation (inverse transfrom) from map to base_link.
				- This is the robot's position on the map.
			2. Get nearest lanelet
			3. Get nearest point on lanelet centerline
			4. Find distance from robot_pos to centerline_point
			5. Add distance to total_deviation
			6. Increment deviation_count
			*/

			geometry_msgs::msg::TransformStamped currentTf;

			try {
				currentTf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
			} catch (tf2::TransformException & ex) {
				RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "base_link", "map", ex.what());
				return;
			}

			double x = currentTf.transform.translation.x;
			double y = currentTf.transform.translation.y;
		}

		std::string osm_path_;
		lanelet::LaneletMapPtr _map;
		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		rclcpp::TimerBase::SharedPtr timer_{nullptr};
        rclcpp::Subscription<PointCloud2>::SharedPtr curb_detection_sub;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CurbLocalizerNode>());
	rclcpp::shutdown();
	return 0;
}