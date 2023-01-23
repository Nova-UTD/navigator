#include <ros/ros.h>
#include "occupancy_grid_generation.h" //"occupancy_grid_generation.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(lidar_pkg::OccupancyGridGeneration, nodelet::Nodelet)

namespace lidar_pkg
{

  void OccupancyGridGeneration::onInit()
  {
    ROS_INFO("Occupancy Grid nodelet has been initialized.");

    //Setting up the nodelet.
    ros::NodeHandle& private_nh = getMTPrivateNodeHandle();

    //Advertising the topics published by this node.
    occ_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("occupancy", 1000);
    masses_pub = private_nh.advertise<lidar_msgs::masses>("masses", 1000);

    //Subscrbing to the required information.

    // Filtered occupancy grid.
    sub_2 = private_nh.subscribe ("mrf_filtered_points", 1, &OccupancyGridGeneration::cloud_cb, this);

    // Ros time. Subscribes to vehilce location every 0.01s.
    timer = private_nh.createTimer(ros::Duration(0.01), &OccupancyGridGeneration::timer_cb, this);
  }

  void OccupancyGridGeneration::timer_cb(const ros::TimerEvent&)
  {
    /*Make sure the frames are correct here.
    /world for old rosbags and /map for new ones. /map should be a global cs and /vehicle_cg_cartesian a local vehicle cg.
    We are looking for the translation of the vehicle wrt the global cs.
    */

    // tf::TransformListener listener;
    // tf::StampedTransform transform;
    //
    // while(true){
    //   try{
    //     listener.lookupTransform("/map", "/vehicle_ground_cartesian", ros::Time(0), transform);
    //     break;
    //   }
    //   catch (tf::TransformException ex){
    //    ros::Duration(0.01).sleep();
    //    continue;
    //   }
    // }
    // prev_vehicle_x = vehicle_x;
    // prev_vehicle_y = vehicle_y;
    // vehicle_x = transform.getOrigin().x();
    // vehicle_y = transform.getOrigin().y();

    // Bernard: I don't need frames ????? Are you sure?
    change_x = 0.0; // vehicle_x - prev_vehicle_x;
    change_y = 0.0; // vehicle_y - prev_vehicle_y;

  }


  void OccupancyGridGeneration::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    /*
    PC processing.
    */

    double centerpoint_x = 64;
    double centerpoint_y = 64;
    double xstart = -1;
    double ystart = -1;
    double xend = -1;
    double yend = -1;

    // Converts the PCL ros message using pcl_conversions.
    pcl::PointCloud<DDLPointType> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // 1. Convert new measurement into a DST grid.
    create_DST_grid(cloud);

    x_new_low = change_x - 64 * res;
    x_new_high = change_x + 64 * res;
    y_new_low = change_y - 64 * res;
    y_new_high = change_y + 64 * res;

    if(initialization_phase == false) {
      if ((x_new_low >= x_old_low) && (x_old_high >= x_new_low)) {
        xstart = x_new_low;
        xend = x_old_high;
      }

      if ((y_new_low >= y_old_low) && (y_old_high >= y_new_low)) {
        ystart = y_new_low;
        yend = y_old_high;
      }

      if ((x_new_low < x_old_low) && (x_new_high >= x_old_low)) {
        xstart = x_old_low;
        xend = x_new_high;
      }

      if ((y_new_low < y_old_low) && (y_new_high >= y_old_low)) {
        ystart = y_old_low;
        yend = y_new_high;
      }

      if ((xstart != -1) && (ystart != -1)) {

        int indx_nl = find_nearest(grid_size, xstart, x_new_low, x_new_high, res);
        int indx_nh = find_nearest(grid_size, xend, x_new_low, x_new_high, res);
        int indy_nl = find_nearest(grid_size, ystart, y_new_low, y_new_high, res);
        int indy_nh = find_nearest(grid_size, yend, y_new_low, y_new_high, res);

        int indx_ol = find_nearest(grid_size, xstart, x_old_low, x_old_high, res);
        int indx_oh = find_nearest(grid_size, xend, x_old_low, x_old_high, res);
        int indy_ol = find_nearest(grid_size, ystart, y_old_low, y_old_high, res);
        int indy_oh = find_nearest(grid_size, yend, y_old_low, y_old_high, res);

        for (unsigned int i=0; i < indx_oh - indx_ol + 1; i++) {
          for (unsigned int j=0; j < indy_oh - indy_ol + 1; j++) {
            prev_free[indx_nl + i][indy_nl + j] = up_free[indx_ol + i][indy_ol + j];
            prev_occ[indx_nl + i][indy_nl + j] = up_occ[indx_ol + i][indy_ol + j];
          }
        }
      }
    }
    mass_update();
    get_mass();
    plotting();
    clear();

    initialization_phase = false;
    x_old_low = x_new_low;
    x_old_high = x_new_high;
    y_old_low = y_new_low;
    y_old_high = y_new_high;
  }

  void OccupancyGridGeneration::create_DST_grid(pcl::PointCloud<DDLPointType>& cloud)
  {
    /*
    Generate the occupancy grid.
    Works pretty well.
    Further improvments/simplifications can be done, but it is not a priority now.
    */

    // ROS_INFO("Inside create DST grid");
    //1.Projects the pcl points onto the 2D-plane/occupancy grid. Performs ray tracing to identify visible spaces.
    add_points_to_the_DST(cloud);
    //2.Performs ray tracing to identify visible spaces to angles around a vehicle which are not represented in the PC or out of range.
    add_free_spaces_to_the_DST();
    //3.Add an ego vehicle to our grid.
    add_ego_vehicle_to_the_DST();
  }

  void OccupancyGridGeneration::add_points_to_the_DST(pcl::PointCloud<DDLPointType>& cloud)
  {
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
      int x = (int)(cloud.points[i].x/res);
      int y = (int)(cloud.points[i].y/res);
      double z = cloud.points[i].z;

      //FIX: Quick hack for Ford dataset.
      if ((-1)*z>0.5){
        continue;
      }

      // Checks if the pcl point is within our grid range.
      // TODO: Replace 64 with some grid size parameter set once.
      if (x >= -64 && y>= -64 && x <= 63 && y <= 63) {
        int angle;

        // Angles vector contains which angles from 0 deg to 360 deg have been represented.
        // It is used to identify free spaces for angles not covered by PC or out of range points.
        if (cloud.points[i].y > 0 && cloud.points[i].x < 0){
          angle = 180 - (int)(atan(std::abs(cloud.points[i].y)/std::abs(cloud.points[i].x))*180.0/3.14159265);
        }
        else if (cloud.points[i].y < 0 && cloud.points[i].x < 0){
          angle = 180 + (int)(atan(std::abs(cloud.points[i].y)/std::abs(cloud.points[i].x))*180.0/3.14159265);
        }
        else if (cloud.points[i].y < 0 && cloud.points[i].x > 0){
          angle = 360 - (int)(atan(std::abs(cloud.points[i].y)/std::abs(cloud.points[i].x))*180.0/3.14159265);
        }
        else{
          angle = (int)(atan(std::abs(cloud.points[i].y)/std::abs(cloud.points[i].x))*180.0/3.14159265);
        }

        angles[angle] = true;
        double slope = (double)(y)/(x);

        // We perform ray/point tracing from the origin to the point to identify the free space using Bresenham's line algorthim.
        // It is a simple algorithm but does a job pretty well.
        if (slope > 0 && slope <= 1 && x>0){
          ray_tracing_approximation_y_increment(0,0,x,y,1,1,false);
        }
        else if (slope > 1 && x>0){
          ray_tracing_approximation_x_increment(0,0,x,y,1,1,false);
        }
        else if (slope < 0 && slope >= -1 && x>0){
          ray_tracing_approximation_y_increment(0,0,x,(-1)*y,1,-1,false);
        }
        else if (slope < -1 && x>0){
          ray_tracing_approximation_x_increment(0,0,x,(-1)*y,1,-1,false);
        }
        else if (slope > 1 && x<0){
          ray_tracing_approximation_x_increment(0,0,(-1)*x,(-1)*y,-1,-1,false);
        }
        else if (slope > 0 && slope <= 1 && x<0){
          ray_tracing_approximation_y_increment(0,0,(-1)*x,(-1)*y,-1,-1,false);
        }
        else if (slope < 0 && slope >= -1 && x<0){
          ray_tracing_approximation_y_increment(0,0,(-1)*x,y,-1,1,false);
        }
        else if (slope < -1 && x<0){
          ray_tracing_approximation_x_increment(0,0,(-1)*x,y,-1,1,false);
        }
      }
    }
  }

  void OccupancyGridGeneration::add_free_spaces_to_the_DST()
  {
    double i = 0.0;
    float ang = 0.0f;

    /*
    TODO: It is a naive approach to fill the free spaces and it should be changed.
    However, it is not the bottleneck right now.
    I'll comment it/clean it tmr.
    */
    for (unsigned int i = 0; i<3600;i++){
    ang = (i*0.1f);
    //std::cout << ang << std::endl;
      if (angles[(int)(ang)] == false){
        int x,y;
        if (ang>0.0f && ang<=45.0f){
          x = 64;
          y = (int)(tan(ang*PI/180.0f)*x);
        }
        else if (ang>45.0f && ang<90.0f){
          y  = 64;
          x = (int)(y/tan(ang*PI/180.0f));
        }
        else if (ang>90.0f && ang<=135.0f){
          y  = 64;
          x = (int)(y/tan((ang-180.0f)*PI/180.0f));
        }
        else if (ang>135.0f && ang<180.0f){
          x = -64;
          y = (int)(tan((ang-180.0)*PI/180.0f)*x);
        }
        else if (ang>180.0f && ang<=225.0f){
          x = -64;
          y = (int)(tan((ang-180.0f)*PI/180.0f)*x);
        }
        else if (ang>225.0f && ang<270.0f){
          y  = -64;
          x = (int)(y/tan((ang-180.0f)*PI/180.0f));
        }
        else if  (ang>270.0f && ang<= 315.0f){
          y = -64;
          x = (int)(y/tan((ang-360.0f)*PI/180.0f));
        }
        else if (ang>315.0f && ang<360.0f){
          x = 64;
          y = (int)(tan((ang-360.0f)*PI/180.0f)*x);
        }
        else if (ang == 0.0f || ang == 360.0f){
          ray_tracing_horizontal(64);
          continue;
        }
        else if (ang == 90.0f){
          ray_tracing_vertical(64);
          continue;
        }
        else if (ang == 180.0f){
          ray_tracing_horizontal_n(-64);
          continue;
        }
        else if (ang == 270.0f){
          ray_tracing_vertical_n(-64);
          continue;
        }

        if (x >= -64 && y>= -64 && x <= 64 && y <= 64) {

          double slope = (double)(y)/(x);
          if (slope > 0 && slope <= 1 && x>0){
            ray_tracing_approximation_y_increment(0,0,x,y,1,1,true);
          }
          else if (slope > 1 && x>0){
            ray_tracing_approximation_x_increment(0,0,x,y,1,1,true);
          }
          else if (slope < 0 && slope >= -1 && x>0){
            ray_tracing_approximation_y_increment(0,0,x,(-1)*y,1,-1,true);
          }
          else if (slope < -1 && x>0){
            ray_tracing_approximation_x_increment(0,0,x,(-1)*y,1,-1,true);
          }
          else if (slope > 1 && x<0){
            ray_tracing_approximation_x_increment(0,0,(-1)*x,(-1)*y,-1,-1,true);
          }
          else if (slope > 0 && slope <= 1 && x<0){
            ray_tracing_approximation_y_increment(0,0,(-1)*x,(-1)*y,-1,-1,true);
          }
          else if (slope < 0 && slope >= -1 && x<0){
            ray_tracing_approximation_y_increment(0,0,(-1)*x,y,-1,1,true);
          }
          else if (slope < -1 && x<0){
            ray_tracing_approximation_x_increment(0,0,(-1)*x,y,-1,1,true);
          }
        }
      }
      angles[(int)(ang)] = false;
    }
  }

  void OccupancyGridGeneration::add_ego_vehicle_to_the_DST()
  {
    // Vehicle shape.
    for(unsigned int j = 60;j<68;j++){
      for(unsigned int i = 62;i<67;i++){
        meas_occ[j][i] = 1.0;
        meas_free[j][i] = 0.0;
      }
    }
  }

  void OccupancyGridGeneration::plotting()
  {
    occupancy_msg.data.clear();
    masses_msg.occ.clear();
    masses_msg.free.clear();
    occupancy_msg.header.frame_id = "/body"; //TODO: Make sure the frame is the correct one.
    occupancy_msg.info.resolution = res;
    occupancy_msg.info.width = grid_size;
    occupancy_msg.info.height = grid_size;
    occupancy_msg.info.origin.position.z = 0.2;
    occupancy_msg.info.origin.position.x = -64.0*(1./3.);
    occupancy_msg.info.origin.position.y = -64.0*(1./3.);
    masses_msg.width = grid_size;
    masses_msg.height = grid_size;

    for (unsigned int i = 0; i < grid_size; i++){
      for (unsigned int j = 0; j < grid_size; j++){
        occupancy_msg.data.push_back(prob_O_plot[j][i]);
        masses_msg.occ.push_back(up_occ[j][i]);
        masses_msg.free.push_back(up_free[j][i]);
      }
    }
    occ_pub.publish(occupancy_msg);
    masses_pub.publish(masses_msg);
  }

  void OccupancyGridGeneration::mass_update()
  {
    for (unsigned int i = 0; i < grid_size; i++){
      for (unsigned int j = 0; j < grid_size; j++){
        up_occ_pred[i][j] = std::min(alpha * prev_occ[i][j], 1.0 - prev_free[i][j]);
        up_free_pred[i][j] = std::min(alpha * prev_free[i][j], 1.0 - prev_occ[i][j]);
      }
    }
    //Combine measurement nad prediction to form posterior occupied and free masses.
    update_of();
  }

  void OccupancyGridGeneration::update_of()
  {
    for (unsigned int i = 0; i < grid_size; i++){
      for (unsigned int j = 0; j < grid_size; j++){
        double unknown_pred = 1.0 - up_free_pred[i][j] - up_occ_pred[i][j];
        double meas_cell_unknown = 1.0 - meas_free[i][j] - meas_occ[i][j];
        double k_value = up_free_pred[i][j] * meas_occ[i][j] + up_occ_pred[i][j] * meas_free[i][j];
        up_occ[i][j] = (up_occ_pred[i][j] * meas_cell_unknown + unknown_pred * meas_occ[i][j] + up_occ_pred[i][j] * meas_occ[i][j])/ (1.0 - k_value);
        up_free[i][j] = (up_free_pred[i][j] * meas_cell_unknown + unknown_pred * meas_free[i][j] + up_free_pred[i][j] * meas_free[i][j])/ (1.0 - k_value);
      }
    }
  }

  void OccupancyGridGeneration::get_mass()
  {
    for (unsigned int i = 0; i < grid_size; i++){
      for (unsigned int j = 0; j < grid_size; j++){
        prob_O[i][j] = (0.5 * up_occ[i][j] + 0.5*(1.0 - up_free[i][j]));
        prob_O_plot[i][j] = 100*(0.5 * up_occ[i][j] + 0.5*(1.0 - up_free[i][j])); //meas_occ[i][j] * 100;
      }
    }
  }

  int OccupancyGridGeneration::find_nearest(int n, double v, double v0, double vn, double res)
  {
    int idx = std::floor(n*(v-v0+res/2.)/(vn-v0+res));
    return idx;
  }


  void OccupancyGridGeneration::ray_tracing_approximation_y_increment(int x1, int y1, int x2, int y2, int flip_x, int flip_y, bool inclusive)
  {
    int slope = 2 * (y2 - y1);
    int slope_error = slope - (x2 - x1);
    int x_sample, y_sample;
    for (int x = x1, y = y1; x < x2; x++){
      if (meas_occ[flip_x*x+64][flip_y*y+64] == meas_mass) {
        break;
      }

      meas_free[flip_x*x+64][flip_y*y+64] = meas_mass;

      slope_error += slope;
      if (slope_error >= 0) {
        y += 1;
        slope_error -= 2 * (x2 - x1);
      }
    }

    if (inclusive==false) {
      meas_occ[flip_x*x2+64][flip_y*y2+64] = meas_mass;
      meas_free[flip_x*x2+64][flip_y*y2+64] = 0.0;
    }
  }

  void OccupancyGridGeneration::ray_tracing_approximation_x_increment(int x1, int y1, int x2, int y2, int flip_x, int flip_y, bool inclusive)
  {
    int slope = 2 * (x2 - x1);
    int slope_error = slope - (y2 - y1);
    int x_sample, y_sample;
    for (int x = x1, y = y1; y < y2; y++){

      if (meas_occ[flip_x*x+64][flip_y*y+64] == meas_mass) {
        break;
      }

      meas_free[flip_x*x+64][flip_y*y+64] = meas_mass;

      slope_error += slope;
      if (slope_error >= 0) {
        x += 1;
        slope_error -= 2 * (y2 - y1);
      }
    }

    if (inclusive==false) {
      meas_occ[flip_x*x2+64][flip_y*y2+64] = meas_mass;
      meas_free[flip_x*x2+64][flip_y*y2+64] = 0.0;
    }
  }

  void OccupancyGridGeneration::ray_tracing_horizontal(int x2)
  {
    int x1 = 0;
    int y1 = 0;
    x2 = x2 - 1;

    for (int x = x1; x <= x2; x++){

      if (meas_occ[x+64][64] == meas_mass) {
        break;
      }

      meas_free[x+64][64] = meas_mass;
    }

    meas_free[x2+64][64] = 0.0;
  }

  void OccupancyGridGeneration::ray_tracing_horizontal_n(int x1)
  {
    int x2 = 0;
    int y2 = 0;
    x1 = x1+1;

    for (int x = x1; x <= x2; x++){

      if (meas_occ[x+64][64] == meas_mass) {
        break;
      }

      meas_free[x+64][64] = meas_mass;
    }

    meas_free[x2+64][64] = 0.0;
  }

  void OccupancyGridGeneration::ray_tracing_vertical(int y2)
  {
    int x1 = 0;
    int y1 = 0;
    y2 = y2-1;

    for (int y = y1; y <= y2; y++){

      if (meas_occ[64][y+64] == meas_mass) {
        break;
      }
      meas_free[64][y+64] = meas_mass;
    }
  }

  void OccupancyGridGeneration::ray_tracing_vertical_n(int y1)
  {
    int x1 = 0;
    int y2 = 0;
    y1 = y1+1;

    for (int y = y1; y <= y2; y++){

      if (meas_occ[64][y+64] == meas_mass) {
        break;
      }
      meas_free[64][y+64] = meas_mass;
    }

  meas_free[64][y2+64] = 0.0;
  }

  void OccupancyGridGeneration::clear()
  {
    for (unsigned int i =0; i<grid_size;i++){
      for (unsigned int j = 0; j<grid_size;j++){
        meas_occ[i][j]= 0.0;
        meas_free[i][j] = 0.0;
      }
    }
  }
}
