// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "floam/LaserProcessingNode.hpp"

using namespace Nova::Atlas;

LaserProcessingNode::LaserProcessingNode() : Node("laser_processing_node") {

        lidar_sub_ = this->create_subscription<PointCloud2>("/velodyne_points", 10, [this](PointCloud2::SharedPtr msg) {velodyneHandler(msg);});
        filtered_pcd_pub_ = this->create_publisher<PointCloud2>("/velodyne_points_filtered", 100);
        edge_pcd_pub_ = this->create_publisher<PointCloud2>("/laser_cloud_edge", 100);
        surf_pcd_pub_ = this->create_publisher<PointCloud2>("/laser_cloud_surf", 100);

    // surf_pcd_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100); 

        this->declare_parameter<double>("scan_period", 0.1);
        this->declare_parameter<double>("vertical_angle", 2.0);
        this->declare_parameter<double>("max_dis", 90.0);
        this->declare_parameter<double>("min_dis", 3.0);
        this->declare_parameter<int>("scan_line", 16); // For VLP-16

        lidar_param_.setScanPeriod(this->get_parameter("scan_period").as_double());
        lidar_param_.setVerticalAngle(this->get_parameter("vertical_angle").as_double());
        lidar_param_.setLines(this->get_parameter("scan_line").as_int());
        lidar_param_.setMaxDistance(this->get_parameter("max_dis").as_double());
        lidar_param_.setMinDistance(this->get_parameter("min_dis").as_double());

        laser_processor_.init(lidar_param_);

        this->create_wall_timer(2ms, [this]() {processCloud();});
    }

void LaserProcessingNode::processCloud(){
        if(!pcd_queue_.empty()){
            RCLCPP_INFO(this->get_logger(), "Processing cloud.");
            // Pop latest PointCloud2 from queue and convert to PCL cloud
            mutex_lock_.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pcd_queue_.front(), *pointcloud_in);
            auto pointcloud_time = (pcd_queue_.front())->header.stamp;
            pcd_queue_.pop();
            mutex_lock_.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            // Extract edges and surfaces from pcd, and time the operation.
            // std::chrono::time_point<std::chrono::system_clock> start, end;
            // start = std::chrono::system_clock::now();

            // MAGIC HERE v
            RCLCPP_INFO(this->get_logger(), "Extracting features...");
            laser_processor_.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            // end = std::chrono::system_clock::now();
            // std::chrono::duration<float> elapsed_seconds = end - start;
            // total_frame++;
            // float time_temp = elapsed_seconds.count() * 1000;
            // total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            // Form a "filtered" cloud, which just has surfaces and edges, e.g. no noise
            PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";
            filtered_pcd_pub_->publish(laserCloudFilteredMsg);

            // Pub edge cloud
            PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            edge_pcd_pub_->publish(edgePointsMsg);

            // Pub surf cloud
            PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            surf_pcd_pub_->publish(surfPointsMsg);

        } else {
            RCLCPP_WARN(this->get_logger(), "PCD buffer empty. Is Lidar data being published?");
        }
    }

    /**
     * @brief Add Lidar message to queue
     * 
     * @param laserCloudMsg Message received from Lidar subscriber
     */
    void LaserProcessingNode::velodyneHandler(const PointCloud2::SharedPtr &laserCloudMsg)
    {
        mutex_lock_.lock();
        pcd_queue_.push(laserCloudMsg);
        mutex_lock_.unlock();
    }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LaserProcessingNode>());

    rclcpp::shutdown();

    return 0;
}

