#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

class TfBroad
{
public:
  TfBroad() {
    sub = n.subscribe("/odom", 1000, &TfBroad::callback, this);
  }

  void callback(const nav_msgs::Odometry::ConstPtr& msg){
    // set header
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_link";
    // set x,y
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    // set theta
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
    // send transform
    br.sendTransform(transformStamped);
  }

private:
  ros::NodeHandle n; 
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Subscriber sub;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_broadcast");
  TfBroad my_tf_broadcaster;
  ros::spin();
  return 0;
}
