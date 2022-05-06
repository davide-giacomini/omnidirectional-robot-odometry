//standard includes
#include "ros/ros.h"

//include geometry_msgs
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

//include for message_filters
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <math.h>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseStamped> MySyncPolicy;

class Calibration {
public:
    Calibration();
private:
    ros::NodeHandle nh;
    message_filters::Subscriber<nav_msgs::Odometry> * sub_odom;
    message_filters::Subscriber<geometry_msgs::PoseStamped> * sub_pose_gt;
    void callback(const nav_msgs::Odometry::ConstPtr& msg_odom, const geometry_msgs::PoseStamped::ConstPtr& msg_pose_gt);
};