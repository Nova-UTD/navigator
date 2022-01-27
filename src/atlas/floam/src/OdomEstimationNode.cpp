// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "floam/OdomEstimationNode.hpp"

using namespace Nova::Atlas;

OdomEstimationNode::OdomEstimationNode() : Node("odom_estimation_node") {
    edge_pcd_sub_ = this->create_subscription<PointCloud2>("/laser_cloud_edge", 100, [this](PointCloud2::SharedPtr msg) {velodyneEdgeHandler(msg);});
    surf_pcd_sub_ = this->create_subscription<PointCloud2>("/laser_cloud_surf", 100, [this](PointCloud2::SharedPtr msg) {velodyneSurfHandler(msg);});
    laser_odom_pub_ = this->create_publisher<Odometry>("/odom", 100);
    setLidarParams();
    
    this->create_wall_timer(2ms, [this]() {estimateOdometry();});
}

void OdomEstimationNode::setLidarParams() {
        this->declare_parameter<double>("scan_period", 0.1);
        this->declare_parameter<double>("vertical_angle", 2.0);
        this->declare_parameter<double>("max_dis", 90.0);
        this->declare_parameter<double>("min_dis", 3.0);
        this->declare_parameter<int>("scan_line", 16); // For VLP-16
        this->declare_parameter<double>("map_resolution", 0.4);

        lidar_param_.setScanPeriod(this->get_parameter("scan_period").as_double());
        lidar_param_.setVerticalAngle(this->get_parameter("vertical_angle").as_double());
        lidar_param_.setLines(this->get_parameter("scan_line").as_int());
        lidar_param_.setMaxDistance(this->get_parameter("max_dis").as_double());
        lidar_param_.setMinDistance(this->get_parameter("min_dis").as_double());

        double map_res = this->get_parameter("map_resolution").as_double();

        odom_estimator_.init(lidar_param_, map_res);
    }

    void OdomEstimationNode:: estimateOdometry(){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            mutex_lock_.lock();
            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.sec < pointCloudEdgeBuf.front()->header.stamp.sec-0.5*lidar_param_.scan_period)){
                pointCloudSurfBuf.pop();
                RCLCPP_WARN_ONCE(this->get_logger(), "time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock_.unlock();
                return;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.sec < pointCloudSurfBuf.front()->header.stamp.sec-0.5*lidar_param_.scan_period)){
                pointCloudEdgeBuf.pop();
                RCLCPP_WARN_ONCE(this->get_logger(), "time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock_.unlock();
                return;  
            }
            //if time aligned 

            // Read from surf and edge queues and convert to PCL format
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            auto pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock_.unlock();

            if(is_odom_inited == false){
                odom_estimator_.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                RCLCPP_INFO(this->get_logger(), "odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

                // MAGIC v WSH
                odom_estimator_.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                // total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                RCLCPP_INFO(this->get_logger(), "average odom estimation time %f ms \n \n", total_time/total_frame);
            }


            // Get odom from actual algorithm v WSH
            Eigen::Quaterniond q_current(odom_estimator_.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = odom_estimator_.odom.translation();

            // We don't want to publish a TF here. WSH.
            // static tf::TransformBroadcaster br;
            // tf::Transform transform;
            // transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            // tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            // transform.setRotation(q);
            // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // publish odometry
            Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            laser_odom_pub_->publish(laserOdometry);
        }
    }

    void OdomEstimationNode::velodyneSurfHandler(const PointCloud2::SharedPtr &laserCloudMsg)
    {
        mutex_lock_.lock();
        pointCloudSurfBuf.push(laserCloudMsg);
        mutex_lock_.unlock();
    }
    void OdomEstimationNode::velodyneEdgeHandler(const PointCloud2::SharedPtr &laserCloudMsg)
    {
        mutex_lock_.lock();
        pointCloudEdgeBuf.push(laserCloudMsg);
        mutex_lock_.unlock();
    }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OdomEstimationNode>());

    rclcpp::shutdown();

    return 0;
}




