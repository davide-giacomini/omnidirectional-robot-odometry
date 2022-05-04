#include "omnidirectional_robot_odometry/compute_odometry.h"

ComputeOdometry::ComputeOdometry() {
    this->sub_cmd_vel = this->nh.subscribe("cmd_vel", 1000, &ComputeOdometry::compute_odometry, this);
    this->pub_odometry = this->nh.advertise<nav_msgs::Odometry>("odom", 1000);
    this->nh.getParam("/init_pose_x", this->init_pose_x);
    this->nh.getParam("/init_pose_y", this->init_pose_y);
    this->nh.getParam("/init_pose_th", this->init_pose_th);

    this->res_odom_service = this->nh.advertiseService("reset_odom", &ComputeOdometry::reset_odometry, this);
}

void ComputeOdometry::compute_odometry(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    this->stamp_ns[0] = this->stamp_ns[1];
    this->stamp_ns[1] = msg->header.stamp.nsec;

    if (this->stamp_ns[0] == -1) {
    // Initialize the poses to the initial pose got in the parameter values.
    // TODO -> It can be done dynamically by getting the first values of the bag from `msg`. I did not do it because the assignment of the project required so.
    //ODOM
    this->current_pose.x = this->init_pose_x;
    this->current_pose.y = this->init_pose_y;
    this->current_pose.th = this->init_pose_th;

    return;
  }

  double vel_x = msg->twist.linear.x;
  double vel_y = msg->twist.linear.y;
  double vel_th = msg->twist.angular.z;

  this->compute_euler_odometry(vel_x, vel_y, vel_th);

  nav_msgs::Odometry odom_msg;

  // set header
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  // set position
  odom_msg.pose.pose.position.x = this->current_pose.x;
  odom_msg.pose.pose.position.y = this->current_pose.y;
  odom_msg.pose.pose.position.z = 0;
  // set theta
  tf2::Quaternion q;
  q.setRPY(0, 0, this->current_pose.th);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  // set covariance
  odom_msg.pose.covariance.fill(0);
  // set twist
  odom_msg.twist.twist.linear.x = vel_x;
  odom_msg.twist.twist.linear.y = vel_y;
  odom_msg.twist.twist.linear.z = 0;
  odom_msg.twist.twist.angular.x = 0;
  odom_msg.twist.twist.angular.y = 0;
  odom_msg.twist.twist.angular.z = vel_th;
  // set covariance
  odom_msg.twist.covariance.fill(0);

  this->pub_odometry.publish(odom_msg);
  this->pub_transform(odom_msg);
}

pose ComputeOdometry::compute_euler_odometry(double vel_x, double vel_y, double vel_th){
  pose previous_pose;

  previous_pose.x = this->current_pose.x;
  previous_pose.y = this->current_pose.y;
  previous_pose.th = this->current_pose.th;

  double dt = (this->stamp_ns[1] - this->stamp_ns[0]) * pow(10, -9); // The pow is to convert it in seconds.

  this->current_pose.x = previous_pose.x + dt*vel_x;
  this->current_pose.y = previous_pose.y + dt*vel_y;
  this->current_pose.th = previous_pose.th + dt*vel_th;

  return previous_pose;
}

pose ComputeOdometry::compute_rungekutta_odometry(double vel_x, double vel_y, double vel_th){
    //TODO
  return this->current_pose;
}

void ComputeOdometry::pub_transform(const nav_msgs::Odometry msg){
  // set header
  this->transformStamped.header.stamp = ros::Time::now();
  this->transformStamped.header.frame_id = msg.header.frame_id;
  this->transformStamped.child_frame_id = msg.child_frame_id;
  // set x,y
  this->transformStamped.transform.translation.x = msg.pose.pose.orientation.x;
  this->transformStamped.transform.translation.y = msg.pose.pose.orientation.y;
  this->transformStamped.transform.translation.z = msg.pose.pose.orientation.z;
  // set theta
  tf2::Quaternion q;
  q.setRPY(0, 0, msg.twist.twist.angular.z);
  this->transformStamped.transform.rotation.x = q.x();
  this->transformStamped.transform.rotation.y = q.y();
  this->transformStamped.transform.rotation.z = q.z();
  this->transformStamped.transform.rotation.w = q.w();
  // send transform
  this->br.sendTransform(transformStamped);
}

bool ComputeOdometry::reset_odometry(omnidirectional_robot_odometry::ResetOdometry::Request  &req,
                                        omnidirectional_robot_odometry::ResetOdometry::Response &res) {

    res.old_x = this->current_pose.x;
    res.old_y = this->current_pose.y;
    res.old_th = this->current_pose.th;

    this->current_pose.x = req.new_x;
    this->current_pose.y = req.new_y;
    this->current_pose.th = req.new_th;

    return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compute_odometry");

  ComputeOdometry co;
  
  ros::spin();

  return 0;
}