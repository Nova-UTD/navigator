#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <lidar_msgs/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <lidar_msgs/masses.h>

typedef lidar_msgs::PointXYZIDBS DDLPointType;

#define PI 3.14159

/*
TODO:BERNARD
Write a documentation/Comment.
*/


namespace lidar_pkg
{
class OccupancyGridGeneration : public nodelet::Nodelet
{
public:
  OccupancyGridGeneration(){};
private:

  bool initialization_phase = true;
  double vehicle_x;
  double vehicle_y;
  double prev_vehicle_x;
  double prev_vehicle_y;
  double change_x;
  double change_y;
  double x_new_low;
  double x_new_high;
  double y_new_low;
  double y_new_high;
  double x_old_low;
  double x_old_high;
  double y_old_low;
  double y_old_high;

  // There are only two events: 0 = Occupied and 1 = Free.
  const static int event_num = 2;

  // Grid size.
  const static int grid_size = 128;

  // Resolution.
  constexpr static double res = 1./3.;

  // Measurement mass.
  constexpr static double meas_mass = 0.95;

  // Occupancy measurement.
  constexpr static double alpha = 0.9;

  // Place holders vehicle positions.
  constexpr static double vehicle_pos_x = 0;
  constexpr static double vehicle_pos_y = 0;
  constexpr static double prev_vehicle_pos_x = 0;
  constexpr static double prev_vehicle_pos_y = 0;

  ros::Subscriber sub_2;
  ros::Timer timer;
  ros::Publisher occ_pub;
  ros::Publisher masses_pub;
  nav_msgs::OccupancyGrid occupancy_msg;
  lidar_msgs::masses masses_msg;

  // Array with the DST data.
  double meas_grids[event_num][grid_size][grid_size];

  // "Freeness" measurement.
  double meas_free[grid_size][grid_size] = {{0}};

  // Occupancy measurement.
  double meas_occ[grid_size][grid_size] = {{0}};
  double prev_free[grid_size][grid_size] = {{0}};
  double prev_occ[grid_size][grid_size] = {{0}};
  double up_free_pred[grid_size][grid_size];
  double up_occ_pred[grid_size][grid_size];
  double up_free[grid_size][grid_size];
  double up_occ[grid_size][grid_size];
  double prob_O[grid_size][grid_size];
  double prob_O_plot[grid_size][grid_size];
  bool angles[360];
  bool first;

  virtual void onInit();
  void timer_cb(const ros::TimerEvent&);

  void transform_listener();
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void create_DST_grid(pcl::PointCloud<DDLPointType>& cloud);
  void ray_tracing_approximation_x_increment(int x1, int y1, int x2, int y2, int flip_x, int flip_y, bool inclusive);
  void ray_tracing_approximation_y_increment(int x1, int y1, int x2, int y2, int flip_x, int flip_y, bool inclusive);
  int find_nearest(int n, double v, double v0, double vn, double res);
  void mass_update();
  void update_of();
  void get_mass();
  void plotting();
  void clear();
  void fill(std::vector<int> flip);
  void ray_tracing_horizontal(int y);
  void ray_tracing_horizontal_n(int y);
  void ray_tracing_vertical(int x);
  void ray_tracing_vertical_n(int x);
  void add_points_to_the_DST(pcl::PointCloud<DDLPointType>& cloud);
  void add_free_spaces_to_the_DST();
  void add_ego_vehicle_to_the_DST();
};
}
