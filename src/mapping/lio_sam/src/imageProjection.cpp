#include "utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"
// #include <math.h> // Trig for ring numbers. WSH

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D // Adds XYZ and padding https://pointclouds.org/documentation/tutorials/adding_custom_ptype.html#id6
        PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct RinglessVelodynePointXYZIT
{
    PCL_ADD_POINT4D // Adds XYZ and padding https://pointclouds.org/documentation/tutorials/adding_custom_ptype.html#id6
        PCL_ADD_INTENSITY;
    float timestamp; // "timestamp" is what SVL calls it
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RinglessVelodynePointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, timestamp, timestamp))

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000; // !!! This is big

class ImageProjection : public ParamServer
{
private:
    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<lio_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    sensor_msgs::msg::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<RinglessVelodynePointXYZIT>::Ptr inputCloud;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

public:
    ImageProjection(const rclcpp::NodeOptions &options) : ParamServer("lio_sam_imageProjection", options), deskewFlag(0)
    {
        callbackGroupLidar = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupImu = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto lidarOpt = rclcpp::SubscriptionOptions();
        lidarOpt.callback_group = callbackGroupLidar;
        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = callbackGroupImu;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;

        subImu = create_subscription<sensor_msgs::msg::Imu>( // Topic should be /imu/imu_raw
            imuTopic, qos_imu,
            std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1),
            imuOpt);
        subOdom = create_subscription<nav_msgs::msg::Odometry>( // IMU pre-integration topic
            "/gps/odom", qos_imu,
            std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1),
            odomOpt);
        subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>( // /lidar_right/points_raw
            pointCloudTopic, qos_lidar,
            std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
            lidarOpt);

        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            "lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
            "lio_sam/deskew/cloud_info", qos);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        inputCloud.reset(new pcl::PointCloud<RinglessVelodynePointXYZIT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        cloudInfo.start_ring_index.assign(N_SCAN, 0);
        cloudInfo.end_ring_index.assign(N_SCAN, 0);

        cloudInfo.point_col_ind.assign(N_SCAN * Horizon_SCAN, 0);
        cloudInfo.point_range.assign(N_SCAN * Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~ImageProjection() {}

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
    {
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg); // Perform extrinsic rotation on IMU reading

        const int MAX_ACCEL = 10;

        if (thisImu.linear_acceleration.x > MAX_ACCEL)
        {
            RCLCPP_WARN_STREAM(get_logger(), "x linear accel out of bounds! Was " << thisImu.linear_acceleration.x);
            thisImu.linear_acceleration.x = MAX_ACCEL;
        }
        if (thisImu.linear_acceleration.x < MAX_ACCEL * -1)
        {
            RCLCPP_WARN_STREAM(get_logger(), "x linear accel out of bounds! Was " << thisImu.linear_acceleration.x);
            thisImu.linear_acceleration.x = MAX_ACCEL * -1;
        }
        if (thisImu.linear_acceleration.y > MAX_ACCEL)
        {
            RCLCPP_WARN_STREAM(get_logger(), "y linear accel out of bounds! Was " << thisImu.linear_acceleration.y);
            thisImu.linear_acceleration.y = MAX_ACCEL;
        }
        if (thisImu.linear_acceleration.y < MAX_ACCEL * -1)
        {
            RCLCPP_WARN_STREAM(get_logger(), "y linear accel out of bounds! Was " << thisImu.linear_acceleration.y);
            thisImu.linear_acceleration.y = MAX_ACCEL * -1;
        }
        // if (thisImu.linear_acceleration.z > MAX_ACCEL) {
        //     RCLCPP_WARN_STREAM(get_logger(), "z linear accel out of bounds! Was "<<thisImu.linear_acceleration.z);
        //     thisImu.linear_acceleration.z = MAX_ACCEL;
        // }
        // if (thisImu.linear_acceleration.z < MAX_ACCEL*-1) {
        //     RCLCPP_WARN_STREAM(get_logger(), "z linear accel out of bounds! Was "<<thisImu.linear_acceleration.z);
        //     thisImu.linear_acceleration.z = MAX_ACCEL*-1;
        // }

        std::lock_guard<std::mutex> lock1(imuLock); // ???
        // int imuSmoothSize = 5;
        // if (imuQueue.size() > imuSmoothSize) { // Perform moving average with window size=5
        //     sensor_msgs::msg::Imu smoothImu;
        //     smoothImu.header = thisImu.header;
        //     smoothImu.orientation_covariance = thisImu.orientation_covariance;
        //     smoothImu.linear_acceleration_covariance = thisImu.linear_acceleration_covariance;
        //     double avg_angular_v_x, avg_angular_v_y, avg_angular_v_z = 0;
        //     double avg_linear_a_x, avg_linear_a_y,avg_linear_a_z = 0;

        //     for (int i = 0; i < imuSmoothSize-1; i++) {
        //         avg_angular_v_x += imuQueue[i].angular_velocity.x;
        //         avg_angular_v_y += imuQueue[i].angular_velocity.y;
        //         avg_angular_v_z += imuQueue[i].angular_velocity.z;
        //         avg_linear_a_x += imuQueue[i].linear_acceleration.x;
        //         avg_linear_a_y += imuQueue[i].linear_acceleration.y;
        //         avg_linear_a_z += imuQueue[i].linear_acceleration.z;
        //     }
        //     avg_angular_v_x = avg_angular_v_x/imuSmoothSize;
        //     avg_angular_v_y = avg_angular_v_y/imuSmoothSize;
        //     avg_angular_v_z = avg_angular_v_z/imuSmoothSize;
        //     smoothImu.angular_velocity.x = avg_angular_v_x;
        //     smoothImu.angular_velocity.y = avg_angular_v_y;
        //     smoothImu.angular_velocity.z = avg_angular_v_z;
        //     imuQueue.push_back(smoothImu); // Add IMU msg to our queue
        // }
        // else {
        //     imuQueue.push_back(thisImu);
        // }
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x <<
        //       ", y: " << thisImu.linear_acceleration.y <<
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x <<
        //       ", y: " << thisImu.angular_velocity.y <<
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf2::Quaternion orientation;
        // tf2::fromMsg(thisImu.orientation, orientation);
        // tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg); // Add
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        // Fails if cloudqueue is <= 2 or
        if (!cachePointCloud(laserCloudMsg))
            return;

        // Fails if IMU queue is empty OR latest reading is newer than the point cloud scan OR oldest is older
        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2) // Skip pointcloud if cache is too small (<=2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front()); // Take cloud from front
        cloudQueue.pop_front();                          // Delete front
        if (sensor == SensorType::VELODYNE)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn); // Convert from ROS PointCloud2 to pcl::PointCloud<PointXYZIRT>::Ptr
        }

        // We don't care about Ouster sensors... WSH
        // else if (sensor == SensorType::OUSTER)
        // {
        //     // Convert to Velodyne format
        //     pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
        //     laserCloudIn->points.resize(tmpOusterCloudIn->size());
        //     laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
        //     for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
        //     {
        //         auto &src = tmpOusterCloudIn->points[i];
        //         auto &dst = laserCloudIn->points[i];
        //         dst.x = src.x;
        //         dst.y = src.y;
        //         dst.z = src.z;
        //         dst.intensity = src.intensity;
        //         dst.ring = src.ring;
        //         dst.time = src.t * 1e-9f;
        //     }
        // }

        // Here we cover for a lack of ring data by adding some fake (manually calculated) values
        else if (sensor == SensorType::RINGLESS_VLP_16)
        {
            // pcl::PointCloud<RinglessVelodynePointXYZIT>::Ptr inputCloud;
            // RCLCPP_INFO(get_logger(),"A");
            pcl::moveFromROSMsg(currentCloudMsg, *inputCloud);
            // RCLCPP_INFO(get_logger(),"B");
            laserCloudIn->points.resize(inputCloud->size());
            // RCLCPP_INFO(get_logger(),"C");

            for (size_t i = 0; i < inputCloud->size(); i++)
            {
                int ring_count = 16;
                // RCLCPP_INFO(get_logger(),"D");

                auto &src = inputCloud->points[i];
                auto &dst = laserCloudIn->points[i];

                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;

                dst.time = src.timestamp;
                // Fake a ring value
                // Ring value is from 0 to 15, with 0 being lowest and 15 being highest

                // Find pitch of point relative to sensor height, = atan(x/sqrt(x^2+y^2))
                double pitch_degrees = atan2(src.z, sqrt(pow(src.x, 2) + pow(src.y, 2))) * 180 / 3.14159265;
                // Find pitch of point from bottom of FOV
                double pitch_from_bottom = pitch_degrees + fov_below_middle_deg;
                // RCLCPP_DEBUG_STREAM(get_logger(), "Center ("<<point.x<<","<<point.y<<","<<point.z<<") is "<<pitch_degrees<<"; "<<pitch_from_bottom);
                // Map this from [0,FOV]->[0,15]
                int ring_number = round((pitch_from_bottom / fov_deg) * 16); // This will always round DOWN.
                // int ring_number = floor(pitch_from_bottom * (ring_count) / fov_deg);
                // RCLCPP_INFO_STREAM(get_logger(), "("<<src.x<<","<<src.y<<","<<src.z<<")-> atan("<<src.z<<"/"<<sqrt(pow(src.x,2)+pow(src.y,2))<<"+bot="<<pitch_from_bottom<<" "<<ring_number);
                if (ring_number >= (ring_count - 1))
                {
                    // RCLCPP_ERROR_STREAM(get_logger(), "Ring number "<<ring_number<<" is above "<<(ring_count-1)<<". Rounding down.");
                    ring_number = 15;
                }

                dst.ring = ring_number;
            }
            // Debug stuff
            // RCLCPP_INFO_STREAM(get_logger(),laserCloudIn->points[0].ring);
            // pcl::io::savePCDFile ("test.pcd", *laserCloudIn);
            // for (size_t i = 0; i < laserCloudIn->size(); i++)
            // {
            //     auto &dst = laserCloudIn->points[i];
            // RCLCPP_INFO_STREAM(get_logger(), "("<<dst.x<<","<<dst.y<<","<<dst.z<<","<<dst.intensity<<","<<dst.ring<<","<<dst.time<<")");
            // }
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor type: " << int(sensor));
            rclcpp::shutdown();
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = stamp2Sec(cloudHeader.stamp);
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            RCLCPP_ERROR(get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }

        // check ring channel
        // static int ringFlag = 0;
        // if (ringFlag == 0)
        // {
        //     ringFlag = -1;
        //     for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
        //     {
        //         if (currentCloudMsg.fields[i].name == "ring")
        //         {
        //             ringFlag = 1;
        //             break;
        //         }
        //     }
        //     if (ringFlag == -1)
        //     {
        //         RCLCPP_ERROR(get_logger(), "Point cloud ring channel not available. Is your sensor type set to 'ringless'?"); // WSH
        //         rclcpp::shutdown();
        //     }
        // }

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t" || field.name == "timestamp")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                RCLCPP_WARN(get_logger(), "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        // if (imuQueue.empty() ||
        //     stamp2Sec(imuQueue.front().header.stamp) > timeScanCur ||
        //     stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd)
        // {
        //     RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");
        //     return false;
        // }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imu_available = false;

        while (!imuQueue.empty())
        {
            if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01) // ???
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i) // These are the bracketed measurements from paper's Fig. 1
        {
            sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

            // get roll, pitch, and yaw estimation for this scan

            // If the IMU scan that we're inspecting is older than the start of the lidar scan, save to cloudInfo's initial guess
            if (currentImuTime <= timeScanCur)
                // Convert IMU msg orientation (Quaternion) to RPY, then assign to cloudInfo's vars for orientation initial guess
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            // Reset our queues  for IMU (X, Y, Z, time) and prepare our IMU offset estimate
            if (imuPointerCur == 0)
            {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // Copy current IMU msg's angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z); // Should this be outside of LIO-SAM?

            // "integrate" rotation from previous IMU queue elem to now, then set current elem accordingly
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imu_available = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odom_available = false;

        while (!odomQueue.empty())
        {
            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::msg::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (stamp2Sec(startOdomMsg.header.stamp) < timeScanCur)
                continue;
            else
                break;
        }

        tf2::Quaternion orientation;
        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw = yaw;

        cloudInfo.odom_available = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (stamp2Sec(odomQueue.back().header.stamp) < timeScanEnd)
            return;

        nav_msgs::msg::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (stamp2Sec(endOdomMsg.header.stamp) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        else
        {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imu_available == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            static float ang_res_x = 360.0 / float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.start_ring_index[i] = count - 1 + 5;
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i, j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.point_col_ind[count] = j;
                    // save range info
                    cloudInfo.point_range[count] = rangeMat.at<float>(i, j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.end_ring_index[i] = count - 1 - 5;
        }
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo->publish(cloudInfo);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto IP = std::make_shared<ImageProjection>(options);
    exec.add_node(IP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Image Projection Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
