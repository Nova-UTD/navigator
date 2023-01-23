#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  ros::Rate rate(10.0);
  double change = 0;
  while (node.ok()){
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setRotation( tf::Quaternion(0, 0, 0.7071068, 0.7071068) );

    br.sendTransform(transform, ros::Time::now(), "body", "global_pcl_frame");
    rate.sleep();
  }
  return 0;
};
